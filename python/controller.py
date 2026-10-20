"""100Hz receive/filter callback and independent 50Hz ONNX loop."""
import collections
import numpy as np
import json
import math
import threading
import time
from pathlib import Path
from policy import Policy
from recovery import Recovery
from proc import SignalFilter
import rl_core
from rl_core import MOTOR_IDS, HOME_DEG, DEFAULT_DEG, GRAVITY_TAU, MOUTH_INDEX, MOUTH_MAX_DEG
from wire import *
import pad
import tuning
import flight
import offsets
import blackbox

FACTORY_ZERO = [
    2236, 2005, 1912, 1866, 1945,
    2127, 2135, 1914, 2228, 2206,
    2550, 2060, 2128, 2034, 2190,
]
ZERO_PATH = Path(__file__).resolve().parents[1] / 'data' / 'zero_pos.json'
# Joints mounted the other way round from the trained model: direction +1 instead of -1.
SIGNS_PATH = Path(__file__).resolve().parents[1] / 'data' / 'joint_signs.json'
CAL_ZERO = 2048  # Finish re-centres each servo so its zero reads this (CAL_ZERO_POS)
EXCLUSIVE_MODES = ('calibrate', 'servo_debug')
PICK_PERIOD_S = 4.0
PICK_MOUTH_CLOSE_PHI = 0.4
ROULADE_S = 1.9
ROULADE_ALPHA = 0.15
CONTROL_PERIOD = 0.02
# After a fall, joints move to the get-up start pose at this rate instead of in one tick.
# A one-tick jump drives every leg servo into the ground at once, and that current spike
# can trip the battery or shield protection (the robot then stays off until the switch is
# cycled). At 90°/s a typical 30-60° move takes 0.3-0.7 s, inside the 1 s fall hold.
FALL_SLEW_DEG_S = 90.0
# Default pose (hold) has no policy, so head channels 1-4 (IDs 30-33: neck pitch, head pitch,
# head yaw, head roll) offset those joints directly: ±1 maps to ± these degrees, inside the
# model's joint ranges, and the offset moves at most HEAD_HOLD_STEP_DEG per 20 ms tick.
HEAD_HOLD_RANGE_DEG = (30.0, 35.0, 60.0, 20.0)
HEAD_HOLD_STEP_DEG = 2.0
HEAD_INDICES = [MOTOR_IDS.index(i) for i in (30, 31, 32, 33)]
# Speed-dependent lean: blend to home_lean_back_deg at this backward speed, and move the
# applied lean at most LEAN_STEP_DEG per 20 ms tick so the policy's joint observations
# never jump.
LEAN_FULL_BACK_VX = 0.2
LEAN_STEP_DEG = 0.4
PAD_ACTIVE_S = 0.3  # a pad sample this recent overrides page motion and keeps the heartbeat

class Controller:
    def __init__(self, bridge):
        self.bridge = bridge
        self.lock = threading.RLock()
        self.servo_op_lock = threading.Lock()
        self.policy_lock = threading.Lock()
        self.policy = Policy()
        ok, message = self.policy.load()
        if not ok:
            raise RuntimeError(message)
        self.model_message = message
        self.getup_policy = Policy()
        ok, message = self.getup_policy.load('xgoduck_getup.onnx')
        if not ok:
            raise RuntimeError(message)
        self.model_message += '; ' + message
        self.pick_policy = Policy()
        ok, message = self.pick_policy.load('xgoduck_pick.onnx')
        if not ok:
            raise RuntimeError(message)
        self.model_message += '; ' + message
        self.roulade_policy = Policy()
        ok, message = self.roulade_policy.load('xgoduck_roulade.onnx')
        if not ok:
            raise RuntimeError(message)
        self.model_message += '; ' + message
        self.recovery = Recovery()
        self.policy_needs_reset = True
        self.pick_active = False
        self.pick_phi = 0.0
        self.roulade_until = 0.0
        self._roulade_alpha = None
        self._warm_index = 0
        self.filter = SignalFilter()
        self.feedback = self.filtered = None
        self.rx_time = 0.0
        self.mode = 'shadow'
        self.generation = 0
        self.twist, self.head, self.mouth = [0.0]*3, [0.0]*4, 0.0
        self.web_time = 0.0
        self.hold_head = [0.0]*4  # applied head offsets in Default pose, degrees
        self.lean_now = 0.0  # home lean applied to the policies this tick, degrees
        self.pad = pad.Mapper()
        self.pad_time = 0.0
        self.pad_state = {'connected': False}
        self.pad_error = ''
        self.arm_pending = False
        self.seq = self.received = self.inferred = self.sent = self.bad_frames = self.gaps = 0
        self.last_error = ''
        self.stop_event = threading.Event()
        self.rx_times = collections.deque(maxlen=1000)
        self.infer_times = collections.deque(maxlen=500)
        self.infer_ms = collections.deque(maxlen=500)
        self.rx_intervals = collections.deque(maxlen=1000)
        # Flight recorder: the last 20 s of control ticks while position control is on.
        self.flight = collections.deque(maxlen=1000)
        self.loop_missed = 0
        self.last_action = [0.0]*15
        self.started = time.monotonic()
        self.boot_pd = {}
        self.raw_pos = list(FACTORY_ZERO)
        self.zero_pos = list(FACTORY_ZERO)
        self.cal_done = [False]*15
        self.cal_targets = [None]*15
        self.raw_mask = 0
        self.servo_volt = [0]*15  # 0.1 V, from the raw packet
        self.servo_temp = [0]*15  # °C
        self.cmd_twist_f, self.cmd_head_f = [0.0]*3, [0.0]*4  # command low-pass state
        self.tuning = tuning.load()
        if self.tuning['walk_model'] != self.policy.file:
            try:
                self._load_walk_model(self.tuning['walk_model'])
            except ValueError as exc:
                self.last_error = f'walk model {self.tuning["walk_model"]}: {exc}; using {self.policy.file}'
                self.tuning['walk_model'] = self.policy.file
        self._apply_tuning()
        self.servo_reply = None
        self.servo_reply_event = threading.Event()
        self.servo_expected_reply = None
        self._zeros_pushed = False
        self._load_zeros()
        # Saved zeros the MCU must use; zero_pos mirrors what the MCU reports.
        self.zero_target = list(self.zero_pos)
        self._zeros_sent = 0.0
        self._resync_hold_until = 0.0  # Finish changes an MCU zero before the host records it
        self._finishing = False  # no slider targets reach the MCU while a Finish is confirmed
        self.raw_seq = 0  # raw packets received, to wait for fresh readings
        self.sign_mask = self._load_signs()  # bit i: joint i uses direction +1
        self.mcu_sign_mask = None
        self._signs_sent = 0.0
        self.mcu_gains = None
        self._gains_sent = 0.0
        # Servo offset check (see offsets.py). Position modes wait for the first scan.
        self.offset_ref = offsets.load()
        self.offset_check = dict(state='pending', mismatch=[], unchecked=[], checked_at=None,
                                 message='Checking the servo offsets…')
        self._offset_scan_wanted = True
        self._offset_scanning = False
        self._offset_retry_at = 0.0
        self._fall_target = None  # ramped get-up start pose while in the fall hold
        self._offset_touched = set()  # IDs recalibrated or rewritten since the last scan
        self._servo_seen = [0.0]*15   # when each servo last answered (state packets)
        self.supply_guard = dict(level=0, events=0, last=None)  # MCU torque limit on low supply
        try:
            self.blackbox = blackbox.BlackBox()
        except OSError:
            self.blackbox = None

    def _load_zeros(self):
        try:
            data = json.loads(ZERO_PATH.read_text(encoding='utf-8'))
            values = [int(v) for v in data['zero_pos']]
            if len(values) != 15 or any(not 0 <= v <= 4095 for v in values):
                return
            self.zero_pos = values
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            pass

    def _save_zeros(self):
        ZERO_PATH.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            'version': 1,
            'ids': list(MOTOR_IDS),
            'zero_pos': list(self.zero_target),
            'saved_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        }
        ZERO_PATH.write_text(json.dumps(payload, indent=2), encoding='utf-8')

    def _load_signs(self):
        try:
            ids = json.loads(SIGNS_PATH.read_text(encoding='utf-8'))['positive_ids']
            return sum(1 << MOTOR_IDS.index(int(i)) for i in ids if int(i) in MOTOR_IDS)
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            return 0

    def _push_signs(self):
        self.bridge.notify('duck_cal', encode_cal(CAL_SET_SIGNS, mask=self.sign_mask))
        self._signs_sent = time.monotonic()

    def set_joint_sign(self, motor_id, flipped):
        motor_id = int(motor_id)
        if motor_id not in MOTOR_IDS:
            raise ValueError('unknown joint ID')
        with self.lock:
            if self.mode in ('hold', 'policy') or (self.feedback and self.feedback.enabled):
                raise ValueError('turn torque off before changing a joint direction')
            bit = 1 << MOTOR_IDS.index(motor_id)
            self.sign_mask = (self.sign_mask | bit) if flipped else (self.sign_mask & ~bit)
            self._save_signs()
            self._push_signs()
            return self.joint_signs()

    def reset_joint_signs(self):
        with self.lock:
            if self.mode in ('hold', 'policy') or (self.feedback and self.feedback.enabled):
                raise ValueError('turn torque off before changing joint directions')
            self.sign_mask = 0
            self._save_signs()
            self._push_signs()
            return self.joint_signs()

    def _save_signs(self):
        ids = [MOTOR_IDS[i] for i in range(15) if self.sign_mask & (1 << i)]
        SIGNS_PATH.parent.mkdir(parents=True, exist_ok=True)
        SIGNS_PATH.write_text(json.dumps({'positive_ids': ids}, indent=2), encoding='utf-8')

    def joint_signs(self):
        return {'flipped_ids': [MOTOR_IDS[i] for i in range(15) if self.sign_mask & (1 << i)],
                'synced': self.mcu_sign_mask == self.sign_mask}

    def _push_zeros(self):
        self.bridge.notify('duck_cal', encode_cal(CAL_SET_ZEROS, mask=ALL_MASK, raw=self.zero_target))
        self._zeros_pushed = True
        self._zeros_sent = time.monotonic()

    def _push_gains(self):
        t = self.tuning
        self.bridge.notify('duck_cal', encode_cal(CAL_SET_GAINS, mask=0,
                           raw=[t['servo_kp'], t['servo_kd'], tuning.MOUTH_KP,
                                round(t['supply_guard_v'] * 10), t['supply_guard_torque_pct']] + [0]*10))
        self._gains_sent = time.monotonic()

    def _gains_synced(self):
        t = self.tuning
        return self.mcu_gains == (t['servo_kp'], t['servo_kd'], round(t['supply_guard_v'] * 10),
                                  t['supply_guard_torque_pct'])

    def _calibration_synced(self):
        return self.zero_pos == self.zero_target and self.mcu_sign_mask == self.sign_mask

    def receive_boot_pd(self, raw):
        if not isinstance(raw, (bytes, bytearray)) or len(raw)!=32: return
        mask=int.from_bytes(raw[:2],'little')
        with self.lock:
            self.boot_pd={str(id):{'p':raw[2+i],'d':raw[17+i]} for i,id in enumerate(MOTOR_IDS) if mask&(1<<i)}

    def receive_raw(self, raw):
        try:
            pkt = decode_raw(raw)
        except ValueError:
            return
        with self.lock:
            self.raw_pos = pkt['pos']
            self.raw_seq += 1
            self.servo_volt, self.servo_temp = pkt['volt'], pkt['temp']
            self.raw_mask = pkt['mask']
            self.mcu_sign_mask = pkt['pos_sign_mask']
            self.zero_pos = pkt['zero']
            self.mcu_gains = (pkt['run_kp'], pkt['run_kd'], pkt['guard_v10'], pkt['guard_pct'])
            # Zeros and directions live in MCU RAM: re-send after an MCU reset or a lost
            # message. The MCU holds one pending calibration message, so send at most one
            # per packet: a second would overwrite the first before the MCU reads it.
            now = time.monotonic()
            if now < self._resync_hold_until:
                pass
            elif self.zero_pos != self.zero_target:
                if now-self._zeros_sent > 0.5:
                    self._push_zeros()
            elif self.mcu_sign_mask != self.sign_mask:
                if now-self._signs_sent > 0.5:
                    self._push_signs()
            elif not self._gains_synced() and now-self._gains_sent > 0.5:
                self._push_gains()

    def receive_servo_reply(self, raw):
        try:
            pkt = decode_servo_reply(raw)
        except ValueError:
            return
        with self.lock:
            if self.servo_expected_reply != (pkt['op'], pkt['target_id']):
                return
            self.servo_reply = pkt
            self.servo_reply_event.set()

    def receive(self, raw):
        try:
            fb = decode_state(raw)
        except ValueError:
            with self.lock: self.bad_frames += 1
            return
        now = time.monotonic()
        with self.lock:
            old = self.feedback
            if old:
                step = elapsed_us(fb.seq, old.seq)
                if step == 0: return
                if step > 0x80000000:
                    self.mode = 'shadow'; self.generation += 1
                    self.arm_pending = False; self.filter.reset()
                    self.recovery.reset(); self.policy_needs_reset = True
                    self._stop_pick_locked(); self._stop_roulade_locked()
                    self._zeros_pushed = False
                else:
                    self.gaps += step-1
                self.rx_intervals.append((now-self.rx_time)*1000)
            self.filtered = self.filter.update(fb.acc, fb.gyro, fb.angles, fb.velocities,
                timestamp=now, imu_ok=fb.imu_ok)
            self.feedback, self.rx_time = fb, now
            self.received += 1; self.rx_times.append(now)
            returned = False
            for i in range(15):
                if fb.mask >> i & 1:
                    # Silent for over 0.5 s, then answering again: its power came back and it
                    # reloaded its offset from EEPROM, so check it before trusting its zero.
                    returned |= now - self._servo_seen[i] > 0.5
                    self._servo_seen[i] = now
            guard = fb.supply_guard
            if guard > self.supply_guard['level']:
                self.supply_guard['events'] += 1
                self.supply_guard['last'] = time.strftime('%H:%M:%S')
            self.supply_guard['level'] = guard
            if self.blackbox:
                self.blackbox.sample(now, fb, self.mode, self.recovery.phase, self.recovery.angle_deg,
                                     self.servo_volt, self.servo_temp, self.raw_mask)
            if returned:
                self._offset_scan_wanted = True
                if self.mode in ('hold', 'policy'):
                    self.mode = 'shadow'; self.generation += 1
                    self.recovery.reset(); self.policy_needs_reset = True
                    self._stop_pick_locked(); self._stop_roulade_locked()
                    self.last_error = 'servo power was lost; disarmed while the offsets are checked'
                    self._disarm_now()

    @staticmethod
    def _imu_valid(fb):
        # The MCU also flags this now, but never trust an exactly-zero accelerometer:
        # a reset sensor answers I2C with zeros and the policy would walk blind.
        return bool(fb and fb.imu_ok and any(abs(a) > 0.01 for a in fb.acc))

    def _require_feedback(self, mask):
        fb = self.feedback
        if not fb or time.monotonic()-self.rx_time > .15:
            raise ValueError('MCU feedback is stale')
        if not self._imu_valid(fb):
            raise ValueError('IMU data invalid (accelerometer reads zero); check the IMU')
        if (fb.mask & mask) != mask:
            raise ValueError('required servo feedback missing')
        return fb

    def _disarm_now(self):
        self.arm_pending = False
        self.seq = (self.seq + 1) & 0xffffffff
        sample = self.feedback.sample_us if self.feedback else 0
        self.bridge.notify('duck_command', encode_command(self.seq, sample, DISARM, DEFAULT_DEG))

    def set_mode(self, mode, _offset_scan=False):
        if mode not in ('shadow', 'off', 'hold', 'policy', 'calibrate', 'servo_debug'):
            raise ValueError('unknown mode')
        with self.lock:
            if self._offset_scanning and not _offset_scan:
                raise ValueError('checking the servo offsets; try again in a moment')
            if mode in ('hold', 'policy'):
                if self.offset_check['state'] == 'pending':
                    raise ValueError('checking the servo offsets; try again in a moment')
                if self.offset_check['state'] == 'mismatch':
                    raise ValueError(self.offset_check['message'])
            if mode == self.mode:
                self.web_time = time.monotonic()
                return {'mode': self.mode}
            if mode in ('hold', 'policy') and self.mode in EXCLUSIVE_MODES:
                raise ValueError('exit calibrate/servo_debug first')
            if mode in EXCLUSIVE_MODES and self.mode in ('hold', 'policy'):
                raise ValueError('disarm before calibrate/servo_debug')
            fb = None
            if mode in ('hold', 'policy'):
                fb = self._require_feedback(0)
                if not self._calibration_synced():
                    raise ValueError('calibration zeros/directions are not confirmed on the MCU yet; wait a moment')
            previous = self.mode
            self.generation += 1; self.mode = mode
            self.recovery.reset(); self.policy_needs_reset = True
            self._stop_pick_locked(); self._stop_roulade_locked()
            self.last_error = ''
            self.web_time = time.monotonic()
            self.twist, self.head, self.mouth = [0.0]*3, [0.0]*4, 0.0
            self.arm_pending = fb is not None and not fb.enabled
            if previous == 'calibrate' and mode != 'calibrate':
                self.cal_done = [False]*15
                self.bridge.notify('duck_cal', encode_cal(CAL_EXIT))
            if previous == 'servo_debug' and mode != 'servo_debug':
                self.bridge.notify('duck_servo', encode_servo(SERVO_EXIT, 10))
            if previous in EXCLUSIVE_MODES and mode not in EXCLUSIVE_MODES and not _offset_scan:
                self._offset_scan_wanted = True  # save what calibration or setup changed
            if mode == 'calibrate':
                self._disarm_now()
                self.cal_targets = [None]*15
                self.cal_done = [False]*15
                self.bridge.notify('duck_cal', encode_cal(CAL_ENTER))
            elif mode == 'servo_debug':
                self._disarm_now()
                self.bridge.notify('duck_servo', encode_servo(SERVO_ENTER, 10))
            elif previous in EXCLUSIVE_MODES:
                self._disarm_now()
        with self.policy_lock:
            self.policy.reset_action_filter()
            self.getup_policy.reset_action_filter()
            self.pick_policy.reset_action_filter()
            self.roulade_policy.reset_action_filter()
        return {'mode': mode}

    def _policies(self):
        return (self.policy, self.getup_policy, self.pick_policy, self.roulade_policy)

    def _load_walk_model(self, name):
        """Swap the walk policy for another xgoduck_walk*.onnx (Walk tuning A/B)."""
        p = Policy()
        ok, message = p.load(name)
        if not ok:
            raise ValueError(message)
        old = self.policy
        p.action_alpha, p.action_alpha_head = old.action_alpha, old.action_alpha_head
        with self.policy_lock:
            self.policy = p
            p.reset_action_filter()
        self.policy_needs_reset = True
        parts = self.model_message.split('; ')
        self.model_message = '; '.join([message] + parts[1:])

    def walk_models(self):
        return sorted(f.name for f in Path(__file__).parent.glob('xgoduck_walk*.onnx'))

    def _apply_tuning(self):
        rl_core.set_imu_bias(*self.tuning['imu_bias_deg'])
        use_model = self.tuning['home_source'] == 'model'
        for p in self._policies():
            p.use_model_home = use_model
            p.lean_deg = self.lean_now = self.tuning['home_lean_deg']
        self.hold_deg = (np.asarray(DEFAULT_DEG, dtype=np.float32) +
                         rl_core.lean_offsets(MOTOR_IDS, self.tuning['home_lean_deg'])).tolist()

    def tuning_status(self):
        return {'tuning': dict(self.tuning), 'walk_models': self.walk_models(),
                'home_deg': {p.file: [round(float(v), 2) for v in p.home_deg] for p in self._policies()},
                'model_home_deg': {p.file: (None if p.model_home_deg is None else
                                            [round(float(v), 2) for v in p.model_home_deg])
                                   for p in self._policies()}}

    def set_tuning(self, values):
        with self.lock:
            for key in ('home_source', 'home_lean_deg', 'home_lean_back_deg', 'walk_model'):
                if key in values and values[key] != self.tuning[key] and self.mode == 'policy':
                    raise ValueError('change the home pose while not walking')
            new = tuning.validate(values, self.tuning)
            if new['walk_model'] != self.policy.file:
                self._load_walk_model(new['walk_model'])
            keys = ('servo_kp', 'servo_kd', 'supply_guard_v', 'supply_guard_torque_pct')
            gains_changed = any(new[k] != self.tuning[k] for k in keys)
            with self.policy_lock:
                self.tuning = new
                self._apply_tuning()
            if gains_changed:
                self._push_gains()
                for p in self._policies():
                    p.reset_action_filter()
            self.policy_needs_reset = True
            tuning.save(new)
        return self.tuning_status()

    def level_imu(self, seconds=1.0):
        """Average the accelerometer while the body is held still and level (checked with a
        spirit level on its flat top), and store the roll/pitch that maps it to [0, 0, -1].
        Default pose is not a reference: head weight and servo sag can tilt the body, and
        levelling there would hide a real lean from the policy."""
        acc, gyro, n = [0.0]*3, 0.0, 0
        deadline = time.monotonic() + max(0.5, min(3.0, float(seconds)))
        last = None
        while time.monotonic() < deadline:
            with self.lock:
                fb, age = self.feedback, time.monotonic()-self.rx_time
            if not fb or age > 0.15 or not fb.imu_ok:
                raise ValueError('no fresh IMU data')
            if fb is not last:
                last = fb
                acc = [a + b for a, b in zip(acc, fb.acc)]
                gyro = max(gyro, math.sqrt(sum(g*g for g in fb.gyro)))
                n += 1
            time.sleep(0.005)
        if n < 20:
            raise ValueError('too few IMU samples')
        mean = [a / n for a in acc]
        norm = math.sqrt(sum(a*a for a in mean))
        if gyro > 8.0:
            raise ValueError(f'robot was moving (gyro peak {gyro:.1f}°/s); hold it still and retry')
        if abs(norm / 9.8 - 1.0) > 0.1:
            raise ValueError(f'acceleration {norm:.2f} m/s² is not ~1 g; hold it still and retry')
        roll, pitch = rl_core.level_bias_from_gravity([a / norm for a in mean])
        if abs(roll) > 30 or abs(pitch) > 30:
            raise ValueError(f'tilt roll {roll:.1f}°, pitch {pitch:.1f}° is too large; is the body held level?')
        status = self.set_tuning({'imu_bias_deg': [round(roll, 2), round(pitch, 2), 0.0]})
        status.update(samples=n, gyro_peak_dps=round(gyro, 2))
        return status

    FLIGHT_FIELDS = ('t', 'mode', 'phase', 'enabled', 'grav', 'gyro_rad', 'angles_deg',
                     'target_deg', 'twist', 'head')

    def flight_log(self):
        with self.lock:
            return {'fields': self.FLIGHT_FIELDS, 'motors': MOTOR_IDS, 'samples': list(self.flight)}

    def flight_summary(self):
        with self.lock:
            samples = list(self.flight)
        result = flight.summarize(samples)
        result['gains'] = {'kp': self.tuning['servo_kp'], 'kd': self.tuning['servo_kd']}
        return result

    def flight_clear(self):
        with self.lock:
            self.flight.clear()
        return {'ok': True}

    def _models(self):
        return (self.policy, self.getup_policy, self.pick_policy, self.roulade_policy)

    def _apply_alphas_locked(self, leg, head):
        for model in self._models():
            model.set_action_alphas(leg=leg, head=head)

    def set_action_alphas(self, **values):
        with self.lock:
            with self.policy_lock:
                leg, head = self.policy.set_action_alphas(**values)
                if self._roulade_alpha is not None:
                    self._roulade_alpha = (leg, head)
                    self._apply_alphas_locked(ROULADE_ALPHA, ROULADE_ALPHA)
                else:
                    self._apply_alphas_locked(leg, head)
                return leg, head

    def _stop_pick_locked(self):
        self.pick_active = False
        self.pick_phi = 0.0

    def _stop_roulade_locked(self):
        self.roulade_until = 0.0
        saved = self._roulade_alpha
        self._roulade_alpha = None
        if saved is not None:
            with self.policy_lock:
                self._apply_alphas_locked(*saved)

    def _roulade_on(self, now):
        return now < self.roulade_until

    def start_pick(self):
        with self.lock:
            if self.mode not in ('shadow', 'policy'):
                raise ValueError('pick requires shadow/policy mode')
            if self.recovery.phase != 'walk' or self._roulade_on(time.monotonic()):
                raise ValueError('pick only while walking upright')
            self.pick_active = True
            self.pick_phi = 0.0
            self.mouth = MOUTH_MAX_DEG
            self.policy_needs_reset = True
            self.web_time = time.monotonic()
            self.last_error = ''
        with self.policy_lock:
            self.pick_policy.reset_action_filter()
            self.pick_policy.warm(2)
        return {'ok': True, 'phi': 0.0, 'model': self.pick_policy.file}

    def start_roulade(self):
        with self.lock:
            if self.mode not in ('shadow', 'policy'):
                raise ValueError('roulade requires shadow/policy mode')
            if self.recovery.phase != 'walk' or self.pick_active:
                raise ValueError('roulade only while walking')
            if self._roulade_alpha is None:
                self._roulade_alpha = (self.policy.action_alpha, self.policy.action_alpha_head)
            self.roulade_until = time.monotonic() + ROULADE_S
            self.policy_needs_reset = True
            self.web_time = time.monotonic()
            self.last_error = ''
            with self.policy_lock:
                self._apply_alphas_locked(ROULADE_ALPHA, ROULADE_ALPHA)
                self.roulade_policy.reset_action_filter()
                self.roulade_policy.warm(2)
        return {'ok': True, 'model': self.roulade_policy.file, 'seconds': ROULADE_S}

    def command(self, twist=None, head=None, mouth=None):
        def values(seq, limits):
            if len(seq)!=len(limits): raise ValueError('wrong command dimension')
            seq = [float(v) for v in seq]
            if any(not math.isfinite(v) or abs(v)>limit for v,limit in zip(seq,limits)):
                raise ValueError('command exceeds bounds')
            return seq
        t = values(twist, [.4,.3,1.0]) if twist is not None else None
        h = values(head, [1.0]*4) if head is not None else None
        m = None
        if mouth is not None:
            m = float(mouth)
            if not math.isfinite(m) or not 0 <= m <= MOUTH_MAX_DEG:
                raise ValueError('command exceeds bounds')
        with self.lock:
            if self.mode in EXCLUSIVE_MODES:
                raise ValueError('motion commands disabled in calibrate/servo_debug')
            if time.monotonic()-self.pad_time < PAD_ACTIVE_S:
                # A connected controller drives motion; the page only refreshes the heartbeat.
                self.web_time = time.monotonic()
                return {'ok': True, 'pad': True}
            if t is not None and not self.pick_active: self.twist = t
            if h is not None and not self.pick_active: self.head = h
            if m is not None and not self.pick_active: self.mouth = m
            self.web_time = time.monotonic()
        return {'ok': True}

    def pad_input(self, state):
        """One gamepad sample from host/pad_helper.py: run mapped actions, set motion."""
        state = dict(state or {})
        now = time.monotonic()
        with self.lock:
            was_active = now-self.pad_time < PAD_ACTIVE_S
            self.pad_state = {k: state.get(k) for k in ('connected', 'name', 'mac', 'buttons', 'axes')}
            if not state.get('connected'):
                self.pad_time = 0.0
                self.pad.reset()
                if was_active:
                    self.twist = [0.0]*3
                return {'ok': True}
            self.pad_time = now
            exclusive = self.mode in EXCLUSIVE_MODES
        edge, twist, head, mouth = self.pad.apply(state.get('buttons') or [], state.get('axes') or {})
        if exclusive:
            return {'ok': True}  # calibration and servo setup ignore the controller
        for action in edge:
            try:
                if action.startswith('mode:'):
                    self.set_mode(action[5:])
                elif action == 'pick':
                    self.start_pick()
                elif action == 'roulade':
                    self.start_roulade()
                self.pad_error = ''
            except ValueError as exc:
                self.pad_error = f'{pad.ACTIONS[action]}: {exc}'
        with self.lock:
            if self.mode in EXCLUSIVE_MODES or self.pick_active:
                return {'ok': True}
            if twist is not None: self.twist = twist
            if head is not None: self.head = head
            if mouth is not None: self.mouth = mouth
        return {'ok': True}

    def pad_config(self):
        return {'config': self.pad.config, 'actions': pad.ACTIONS, 'axis_targets': pad.AXIS_TARGETS,
                'defaults': pad.validate(pad.DEFAULT_CONFIG)}

    def set_pad_config(self, config):
        saved = pad.save(config)
        with self.lock:
            self.pad.config = saved
            self.pad.reset()
        return {'ok': True, 'config': saved}

    def cal_targets_set(self, targets):
        # null leaves a joint unpowered; the MCU energizes a joint on its first target.
        targets = list(targets)
        if len(targets) != 15:
            raise ValueError('15 raw targets required')
        values = [None if v is None else int(v) for v in targets]
        if any(v is not None and not 0 <= v <= 4095 for v in values):
            raise ValueError('raw targets must be in 0..4095')
        mask = sum(1 << i for i, v in enumerate(values) if v is not None)
        with self.lock:
            if self.mode != 'calibrate':
                raise ValueError('not in calibrate mode')
            self.cal_targets = values
            self.web_time = time.monotonic()
            if mask and not self._finishing:
                raw = [0 if v is None else v for v in values]
                self.bridge.notify('duck_cal', encode_cal(CAL_SET_RAW, mask=mask, raw=raw))
        return {'ok': True, 'targets': values}

    def cal_finish(self, index):
        index = int(index)
        if not 0 <= index < 15:
            raise ValueError('joint index out of range')
        with self.lock:
            if self.mode != 'calibrate':
                raise ValueError('not in calibrate mode')
            if self._finishing:
                raise ValueError('another Finish is still being confirmed')
            # The MCU re-centres the servo's position offset so this pose reads CAL_ZERO,
            # reading the register back and retrying up to three times (~25 ms each).
            self._finishing = True
            self._resync_hold_until = time.monotonic() + 2.0
            self.bridge.notify('duck_cal', encode_cal(CAL_FINISH_ONE, index=index))
            self.web_time = time.monotonic()
            seq = self.raw_seq
        try:
            # Confirm from three consecutive fresh readings of that servo after the change.
            deadline = time.monotonic() + 1.2
            good, raw = 0, None
            time.sleep(0.05)
            while good < 3:
                with self.lock:
                    fresh = self.raw_seq != seq and self.raw_mask >> index & 1
                    seq, raw = self.raw_seq, int(self.raw_pos[index])
                if fresh:
                    good = good + 1 if abs(raw - CAL_ZERO) <= 12 else 0
                if time.monotonic() > deadline:
                    raise ValueError(f'ID {MOTOR_IDS[index]} did not confirm the offset change '
                                     f'(reads {raw}); its zero was not changed. Press Finish again.')
                time.sleep(0.02)
        finally:
            with self.lock:
                self._finishing = False
        with self.lock:
            self.zero_target[index] = CAL_ZERO
            self.zero_pos[index] = CAL_ZERO
            self._resync_hold_until = 0.0
            self.cal_done[index] = True
            self._offset_touched.add(MOTOR_IDS[index])
            self._save_zeros()
            self.web_time = time.monotonic()
            return {'ok': True, 'index': index, 'zero_pos': self.zero_pos[index],
                    'cal_done': list(self.cal_done)}

    def _require_servo_debug(self):
        if self.mode != 'servo_debug':
            raise ValueError('not in servo_debug mode')
        self.web_time = time.monotonic()

    def servo_op(self, action, target_id, new_id=None, kp=5, kd=20, raw_pos=2047):
        with self.servo_op_lock:
            if action != 'set_id':
                reply = self._servo_op(action, target_id, new_id, kp, kd, raw_pos)
                if action in ('write_offset', 'set_middle'):
                    with self.lock:
                        self._offset_touched.add(int(target_id))
                return reply
            target_id, new_id = int(target_id), int(new_id)
            if not 0 <= target_id <= 253 or not 0 <= new_id <= 253:
                raise ValueError('ID must be between 0 and 253')
            if not self._servo_op('ping', target_id)['ok']:
                raise ValueError(f'No servo replied at ID {target_id}; ID was not changed')
            if target_id == new_id:
                return dict(ok=True, op=SERVO_SET_ID, target_id=target_id, new_id=new_id)
            if self._servo_op('ping', new_id)['ok']:
                raise ValueError(f'ID {new_id} is already in use; choose another ID')
            # Keep the checks, write and verification in one serialized operation.
            reply = self._servo_op(action, target_id, new_id, kp, kd, raw_pos)
            time.sleep(0.05)
            if not self._servo_op('ping', new_id)['ok']:
                raise ValueError(f'ID write sent, but ID {new_id} did not reply. Scan again')
            if self._servo_op('ping', target_id)['ok']:
                raise ValueError('Both old and new IDs replied. Connect only one servo and scan again')
            with self.lock:
                self._offset_touched.add(new_id)  # a new servo at this ID: adopt its offset
            return dict(reply, new_id=new_id)

    def _servo_op(self, action, target_id, new_id=None, kp=5, kd=20, raw_pos=2047):
        target_id = int(target_id)
        with self.lock:
            self._require_servo_debug()
            if action == 'unlock':
                op, payload = SERVO_UNLOCK, encode_servo(SERVO_UNLOCK, target_id)
            elif action == 'set_id':
                new_id = int(new_id)
                if not 0 <= new_id <= 253:
                    raise ValueError('invalid new id')
                op, payload = SERVO_SET_ID, encode_servo(SERVO_SET_ID, target_id, new_id=new_id)
            elif action == 'goto':
                raw_pos = int(raw_pos)
                op, payload = SERVO_GOTO, encode_servo(SERVO_GOTO, target_id, raw_pos=raw_pos)
            elif action == 'set_gains':
                kp, kd = int(kp), int(kd)
                if not 0 <= kp <= 255 or not 0 <= kd <= 255:
                    raise ValueError('kp/kd out of range')
                op = SERVO_SET_PERM_KP_KD
                payload = encode_servo(SERVO_SET_PERM_KP_KD, target_id, kp=kp, kd=kd)
            elif action == 'read':
                op, payload = SERVO_READ, encode_servo(SERVO_READ, target_id)
            elif action == 'ping':
                op, payload = SERVO_PING, encode_servo(SERVO_PING, target_id)
            elif action == 'read_reg':
                # new_id carries the register address.
                op, payload = SERVO_READ_REG, encode_servo(SERVO_READ_REG, target_id, new_id=int(new_id))
            elif action == 'write_limit':
                # new_id is the angle-limit register (9 or 11), raw_pos its value.
                if int(new_id) not in (ADDR_MIN_ANGLE, ADDR_MAX_ANGLE):
                    raise ValueError('only angle-limit registers 9 and 11 are writable')
                op, payload = SERVO_WRITE_LIMIT, encode_servo(
                    SERVO_WRITE_LIMIT, target_id, new_id=int(new_id), raw_pos=int(raw_pos))
            elif action == 'write_offset':
                # raw_pos is the 12-bit register value. These servos report
                # (encoder - offset) mod 4096, so (encoder - 2048) % 4096 centres a joint.
                value = int(raw_pos)
                if not 0 <= value <= 4095:
                    raise ValueError('offset register must be within 0..4095')
                op, payload = SERVO_WRITE_LIMIT, encode_servo(
                    SERVO_WRITE_LIMIT, target_id, new_id=ADDR_POS_OFFSET, raw_pos=value)
            elif action == 'set_middle':
                op, payload = SERVO_SET_MIDDLE, encode_servo(SERVO_SET_MIDDLE, target_id)
            elif action == 'torque_off':
                op, payload = SERVO_ENTER, encode_servo(SERVO_ENTER, target_id)
            else:
                raise ValueError('unknown servo action')
            self.servo_reply_event.clear()
            self.servo_reply = None
            self.servo_expected_reply = (op, target_id)
        try:
            self.bridge.notify('duck_servo', payload)
            return self._wait_servo_reply(op, target_id=target_id)
        finally:
            with self.lock:
                self.servo_expected_reply = None

    def _wait_servo_reply(self, op, timeout=0.5, target_id=None):
        if not self.servo_reply_event.wait(timeout):
            raise ValueError('servo reply timeout')
        with self.lock:
            reply = self.servo_reply
        if not reply or reply['op'] != op:
            raise ValueError('unexpected servo reply')
        if target_id is not None and reply['target_id'] != target_id:
            raise ValueError('unexpected servo reply ID')
        if not reply['ok'] and op != SERVO_PING:
            raise ValueError('servo operation failed')
        return reply

    def _offset_worker(self):
        while not self.stop_event.wait(0.2):
            try:
                self._offset_scan_if_due()
            except Exception as exc:
                with self.lock:
                    self.offset_check['message'] = f'servo offset check failed: {exc}'
                    self._offset_retry_at = time.monotonic() + 5.0

    def _offset_scan_if_due(self):
        with self.lock:
            fb = self.feedback
            if (not self._offset_scan_wanted or time.monotonic() < self._offset_retry_at
                    or self.mode not in ('shadow', 'off') or not fb or fb.enabled
                    or time.monotonic() - self.rx_time > .15 or not fb.mask):
                return
        with self.servo_op_lock:
            with self.lock:
                if self.mode not in ('shadow', 'off'):
                    return
                previous, error, mask = self.mode, self.last_error, self.feedback.mask
                self._offset_scan_wanted = False
                self._offset_scanning = True
                touched = set(self._offset_touched)
            current, missed = {}, []
            try:
                self.set_mode('servo_debug', _offset_scan=True)
                # The MCU holds one pending servo message: a read sent before it has taken
                # SERVO_ENTER would replace it, and the MCU refuses reads outside servo setup.
                deadline = time.monotonic() + 1.0
                while not (self.feedback and self.feedback.servo_debugging):
                    if time.monotonic() > deadline:
                        raise ValueError('the MCU did not enter servo setup')
                    time.sleep(0.01)
                for i, motor_id in enumerate(MOTOR_IDS):
                    if not mask >> i & 1:
                        missed.append(motor_id)
                        continue
                    for attempt in range(3):
                        try:
                            reply = self._servo_op('read_reg', motor_id, new_id=ADDR_POS_OFFSET)
                            current[motor_id] = int(reply['raw_pos'])
                            break
                        except ValueError:
                            if attempt == 2:
                                missed.append(motor_id)
            finally:
                try:
                    self.set_mode(previous, _offset_scan=True)
                finally:
                    with self.lock:
                        self._offset_scanning = False
                        self.last_error = error
        with self.lock:
            # Adopt servos changed on purpose (Finish, offset writes, a new ID) and servos with
            # no saved offset yet; every other servo must still hold its saved offset.
            changed = False
            for motor_id, value in current.items():
                if motor_id in touched or motor_id not in self.offset_ref:
                    changed |= self.offset_ref.get(motor_id) != value
                    self.offset_ref[motor_id] = value
            self._offset_touched -= set(current)
            if changed:
                offsets.save(self.offset_ref)
            mismatch = offsets.compare(self.offset_ref, current)
            unchecked = sorted(set(missed))
            if missed:
                self._offset_scan_wanted = True
                self._offset_retry_at = time.monotonic() + 5.0
            if not current:
                self.offset_check = dict(self.offset_check, state='pending', unchecked=unchecked,
                                         message='Could not read the servo offsets yet; retrying')
                return
            if mismatch:
                ids = ', '.join(str(m['id']) for m in mismatch)
                detail = '; '.join(f"ID {m['id']} reads {m['now']}, saved {m['saved']}" for m in mismatch)
                message = (f'Servo calibration changed on ID {ids} ({detail}). Recalibrate '
                           f'{"it" if len(mismatch) == 1 else "them"} on the Calibration page '
                           f'before Default pose or Walk.')
                state = 'mismatch'
            else:
                message = ('Servo offsets match calibration' +
                           (f'; not checked: ID {", ".join(map(str, unchecked))}' if unchecked else ''))
                state = 'ok'
            self.offset_check = dict(state=state, mismatch=mismatch, unchecked=unchecked,
                                     checked_at=time.strftime('%H:%M:%S'), message=message)

    def step(self):
        with self.lock:
            if self.pad_time and time.monotonic()-self.pad_time > PAD_ACTIVE_S:
                # Controller samples stopped (helper or pad gone): drop its motion.
                self.pad_time = 0.0
                self.pad.reset()
                self.twist = [0.0]*3
            fb, filt, generation, mode = self.feedback, self.filtered, self.generation, self.mode
            age = time.monotonic()-self.rx_time
            if mode in ('hold','policy'):
                lost = None
                if not fb or age>.15 or not self._imu_valid(fb):
                    lost = 'sensor feedback lost; disarmed'
                elif time.monotonic()-max(self.web_time, self.pad_time) > 1.0:
                    lost = 'web heartbeat lost; disarmed'
                if lost:
                    self.mode = mode = 'shadow'; self.generation += 1
                    self.arm_pending = False; self.last_error = lost
                    generation = self.generation
                    self.recovery.reset(); self.policy_needs_reset = True
                    self._stop_pick_locked(); self._stop_roulade_locked()
            elif mode in EXCLUSIVE_MODES:
                if time.monotonic()-self.web_time > 2.0:
                    previous = mode
                    self.mode = mode = 'shadow'; self.generation += 1
                    self.last_error = 'calibrate/servo_debug heartbeat lost; exited'
                    generation = self.generation
                    self._offset_scan_wanted = True
                    if previous == 'calibrate':
                        self.cal_done = [False]*15
                        self.bridge.notify('duck_cal', encode_cal(CAL_EXIT))
                    else:
                        self.bridge.notify('duck_servo', encode_servo(SERVO_EXIT, 10))
                    self._disarm_now()
            twist, head, mouth = list(self.twist), list(self.head), self.mouth
            tau = self.tuning['cmd_tau_s']
            if tau > 0:
                # First-order low-pass on the user command so stick steps reach the policy as ramps.
                k = 1.0 - math.exp(-CONTROL_PERIOD / tau)
                self.cmd_twist_f = [f + k*(v - f) for f, v in zip(self.cmd_twist_f, twist)]
                self.cmd_head_f = [f + k*(v - f) for f, v in zip(self.cmd_head_f, head)]
                twist, head = list(self.cmd_twist_f), list(self.cmd_head_f)
            else:
                self.cmd_twist_f, self.cmd_head_f = list(twist), list(head)
            t = self.tuning
            back = max(0.0, min(1.0, -twist[0] / LEAN_FULL_BACK_VX)) if mode == 'policy' else 0.0
            want = t['home_lean_deg'] + back * (t['home_lean_back_deg'] - t['home_lean_deg'])
            self.lean_now += max(-LEAN_STEP_DEG, min(LEAN_STEP_DEG, want - self.lean_now))
            for p in self._policies():
                p.lean_deg = self.lean_now
            valid = bool(fb and age < .15 and fb.imu_ok and filt
                         and filt['imu_fusion_ready'] and filt['imu_fusion_updated'])
            now = time.monotonic()
            if self.roulade_until and now >= self.roulade_until:
                self._stop_roulade_locked()
                self.recovery.fall_elapsed = 0.0
                self.policy_needs_reset = True
            rolling = self._roulade_on(now)
            if mode in ('shadow', 'policy') and not rolling:
                changed = self.recovery.update(filt['grav'] if filt else [0,0,0],
                                               now, valid)
                self.policy_needs_reset |= changed
                if changed and self.recovery.phase != 'walk':
                    self._stop_pick_locked()
            phase = self.recovery.phase
            if rolling and phase != 'walk':
                self._stop_roulade_locked()
                rolling = False
            picking = self.pick_active and phase == 'walk' and not rolling
            if rolling:
                twist, head = [0.0]*3, [0.0]*4
                selected = self.roulade_policy
            elif picking:
                phi = self.pick_phi
                angle = 2.0 * math.pi * phi
                twist = [math.cos(angle), math.sin(angle), 0.0]
                head = [0.0]*4
                mouth = MOUTH_MAX_DEG if phi < PICK_MOUTH_CLOSE_PHI else 0.0
                self.mouth = mouth
                selected = self.pick_policy
            else:
                selected = self.policy if phase == 'walk' else self.getup_policy
            reset_policy = self.policy_needs_reset
            inferring = mode in ('shadow','policy') and valid
            if inferring:
                self.policy_needs_reset = False
            exclusive = mode in EXCLUSIVE_MODES
        if exclusive:
            return
        # Default pose follows the same slewed lean as the policies.
        target = (np.asarray(DEFAULT_DEG, dtype=np.float32) +
                  rl_core.lean_offsets(MOTOR_IDS, self.lean_now)).tolist()
        if inferring and phase == 'default_pose':
            # Switch to getup immediately, but force action/history to zero for 1 s.
            with self.policy_lock:
                selected.reset_action_filter()
                home = selected.action_to_deg([0.0]*14)
            with self.lock:
                # Ramp from where the joints are, not from the last walk command.
                if self._fall_target is None and self.feedback:
                    self._fall_target = list(self.feedback.angles)
                if self._fall_target is not None:
                    step = FALL_SLEW_DEG_S * CONTROL_PERIOD
                    self._fall_target = [c + max(-step, min(step, h - c))
                                         for c, h in zip(self._fall_target, home)]
                    target = list(self._fall_target)
                else:
                    target = home
        else:
            with self.lock:
                self._fall_target = None
        if inferring and phase != 'default_pose':
            if phase == 'getup':
                twist, head = [0.0]*3, [0.0]*4
            begin = time.monotonic()
            with self.policy_lock:
                if reset_policy:
                    selected.reset_action_filter()
                    # One warm pass keeps the newly selected session's arenas resident.
                    selected.warm(1)
                action = selected.infer(filt['grav'], filt['gyro_rad'], filt['angles_deg'], filt['vels_deg'], twist, head)
                target = selected.action_to_deg(action)
            finish = time.monotonic()
            with self.lock:
                self.inferred += 1; self.infer_times.append(finish)
                self.infer_ms.append((finish-begin)*1000)
                if picking:
                    self.pick_phi += CONTROL_PERIOD / PICK_PERIOD_S
                    if self.pick_phi >= 1.0:
                        self._stop_pick_locked()
                        self.policy_needs_reset = True
        with self.lock:
            # A stop arriving during ONNX always wins; never send its obsolete action.
            if self.generation != generation:
                return
            mode = self.mode
            if mode in EXCLUSIVE_MODES:
                return
            command_mode = {'shadow':DISARM,'off':DISARM,'hold':HOLD,'policy':POLICY}[mode]
            if self.arm_pending:
                command_mode = ARM
                # Initial enable always uses measured angles.
                target = list(fb.angles)
                self.arm_pending = False
            else:
                target = list(target)
                target[MOUTH_INDEX] = mouth
                if mode == 'hold':
                    for k, idx in enumerate(HEAD_INDICES):
                        want = max(-1.0, min(1.0, head[k])) * HEAD_HOLD_RANGE_DEG[k]
                        step = max(-HEAD_HOLD_STEP_DEG, min(HEAD_HOLD_STEP_DEG, want - self.hold_head[k]))
                        self.hold_head[k] += step
                        target[idx] += self.hold_head[k]
            if mode != 'hold':
                self.hold_head = [0.0]*4
            if fb is None: return
            if mode in ('hold', 'policy'):
                self.flight.append((round(time.monotonic(), 3), mode, phase, int(fb.enabled),
                    [round(v, 3) for v in (filt['grav'] if filt else (0, 0, -1))],
                    [round(v, 3) for v in (filt['gyro_rad'] if filt else (0, 0, 0))],
                    [round(v, 1) for v in fb.angles], [round(float(v), 1) for v in target],
                    [round(v, 3) for v in twist], [round(v, 2) for v in head]))
            self.last_action = list(target)
            self.seq = (self.seq+1)&0xffffffff
            payload = encode_command(self.seq, fb.sample_us, command_mode, target)
            self.bridge.notify('duck_command', payload)
            self.sent += 1

    def _keep_warm_idle(self):
        """Run one dummy pass on an idle session so walk/getup/pick/roulade stay hot."""
        with self.lock:
            phase = self.recovery.phase
            rolling = self._roulade_on(time.monotonic()) and phase == 'walk'
            picking = self.pick_active and phase == 'walk' and not rolling
            if rolling:
                active = self.roulade_policy
            elif picking:
                active = self.pick_policy
            elif phase == 'walk':
                active = self.policy
            else:
                active = self.getup_policy
            candidates = [p for p in (self.policy, self.getup_policy, self.pick_policy, self.roulade_policy) if p is not active]
        if not candidates:
            return
        self._warm_index = (self._warm_index + 1) % len(candidates)
        idle = candidates[self._warm_index]
        if self.policy_lock.acquire(blocking=False):
            try:
                idle.warm(1)
            finally:
                self.policy_lock.release()

    def run(self):
        threading.Thread(target=self._offset_worker, name='offset_check', daemon=True).start()
        deadline = time.monotonic()
        while not self.stop_event.is_set():
            try:
                self.step()
            except Exception as exc:
                with self.lock:
                    self.last_error = str(exc); self.mode = 'off'; self.generation += 1; self.arm_pending = False
                    self._stop_pick_locked(); self._stop_roulade_locked()
            deadline += CONTROL_PERIOD
            now = time.monotonic()
            if deadline <= now:
                missed = int((now-deadline)/CONTROL_PERIOD)+1
                self.loop_missed += missed; deadline += missed*CONTROL_PERIOD
            else:
                # Use spare budget to keep inactive ONNX sessions warm.
                if deadline - now > 0.008:
                    try:
                        self._keep_warm_idle()
                    except Exception:
                        pass
            self.stop_event.wait(max(0.0,deadline-time.monotonic()))

    def stop(self):
        self.stop_event.set()
        with self.lock:
            previous = self.mode
            self.mode = 'off'; self.generation += 1; self.arm_pending = False
            if previous == 'calibrate':
                self.cal_done = [False]*15
                self.bridge.notify('duck_cal', encode_cal(CAL_EXIT))
            elif previous == 'servo_debug':
                self.bridge.notify('duck_servo', encode_servo(SERVO_EXIT, 10))
            self.seq += 1
            self.bridge.notify('duck_command', encode_command(self.seq, 0, DISARM, DEFAULT_DEG))

    def status(self):
        now = time.monotonic()
        def hz(times):
            recent = [t for t in times if now-t < 5]
            if len(recent) < 2:
                return 0.0
            span = recent[-1] - recent[0]
            return (len(recent)-1)/span if span > 1e-9 else 0.0
        def summary(samples):
            vals=sorted(samples)
            return {'mean':sum(vals)/len(vals), 'p99':vals[min(len(vals)-1,int(len(vals)*.99))], 'max':vals[-1]} if vals else {}
        with self.lock:
            fb = self.feedback
            result = dict(mode=self.mode, model=self.model_message, bridge_baud=2000000,
                feedback_hz=hz(self.rx_times), inference_hz=hz(self.infer_times),
                received=self.received, inferred=self.inferred, sent=self.sent,
                bad_frames=self.bad_frames, sequence_gaps=self.gaps, loop_missed=self.loop_missed,
                inference_ms=summary(self.infer_ms), receive_interval_ms=summary(self.rx_intervals),
                feedback_age_ms=(now-self.rx_time)*1000 if fb else None,
                uptime_s=now-self.started, last_error=self.last_error, command=self.twist,
                head=self.head, mouth=self.mouth, policy_targets=self.last_action, motors=MOTOR_IDS, boot_pd=self.boot_pd,
                full_robot_ready=bool(fb and fb.imu_ok and fb.mask==ALL_MASK and now-self.rx_time<.15),
                raw_pos=list(self.raw_pos), zero_pos=list(self.zero_pos),
                factory_zero_pos=list(FACTORY_ZERO), cal_done=list(self.cal_done),
                cal_targets=list(self.cal_targets), raw_mask=self.raw_mask)
            result['imu_valid']=self._imu_valid(fb)
            result['control_ready']=bool(self._imu_valid(fb) and now-self.rx_time<.15 and self.mode not in EXCLUSIVE_MODES
                                         and self._calibration_synced() and self.offset_check['state'] == 'ok')
            result['calibration_synced']=self._calibration_synced()
            result['offset_check'] = dict(self.offset_check, scanning=self._offset_scanning)
            result['supply_guard'] = dict(self.supply_guard, v=self.tuning['supply_guard_v'],
                                          torque_pct=self.tuning['supply_guard_torque_pct'])
            result['servo_scan_version'] = 1
            present = [i for i in range(15) if self.raw_mask & (1 << i) and self.servo_volt[i]]
            result['battery_v'] = min(self.servo_volt[i] for i in present) / 10 if present else None
            result['servo_volt'] = [v / 10 for v in self.servo_volt]
            result['servo_temp'] = list(self.servo_temp)
            result['servo_temp_max'] = max((self.servo_temp[i] for i in present), default=None)
            result['tuning'] = dict(self.tuning)
            result['lean_now_deg'] = round(self.lean_now, 2)
            result['joint_signs'] = self.joint_signs()
            result['servo_gains'] = {'kp': self.tuning['servo_kp'], 'kd': self.tuning['servo_kd'],
                                     'mcu': self.mcu_gains, 'synced': self._gains_synced()}
            result['pad'] = dict(self.pad_state, active=now-self.pad_time < PAD_ACTIVE_S,
                                 error=self.pad_error)
            result['deployment'] = dict(reference='rl/raspi_deploy', control_hz=50,
                home_deg=[float(v) for v in self.policy.home_deg], default_deg=DEFAULT_DEG,
                action_scale=self.policy.action_scale, action_alpha=self.policy.action_alpha,
                action_alpha_head=self.policy.action_alpha_head,
                gyro_alpha=self.filter.gyro_alpha, velocity_alpha=self.filter.vel_alpha,
                gravity_tau_s=GRAVITY_TAU, imu_bias_deg=list(rl_core.IMU_BIAS_DEG),
                filter_clock='host_receive', snapshot='latest')
            result['recovery'] = self.recovery.status()
            result['recovery']['active'] = self.mode in ('shadow','policy')
            result['pick'] = dict(active=self.pick_active, phi=self.pick_phi,
                                 period_s=PICK_PERIOD_S, mouth_close_phi=PICK_MOUTH_CLOSE_PHI,
                                 model=self.pick_policy.file)
            remain = max(0.0, self.roulade_until - now)
            result['roulade'] = dict(active=remain > 0, remaining_s=remain,
                                    seconds=ROULADE_S, model=self.roulade_policy.file)
            if self.mode in ('shadow','policy'):
                if remain > 0 and self.recovery.phase == 'walk':
                    result['active_model'] = self.roulade_policy.file
                elif self.pick_active and self.recovery.phase == 'walk':
                    result['active_model'] = self.pick_policy.file
                elif self.recovery.phase == 'walk':
                    result['active_model'] = self.policy.file
                else:
                    result['active_model'] = self.getup_policy.file
            else:
                result['active_model'] = None
            if fb:
                result.update(servo_ids=[i for j,i in enumerate(MOTOR_IDS) if fb.mask & (1<<j)],
                    imu_ok=fb.imu_ok, enabled=fb.enabled, mcu_watchdog=bool(fb.flags&4),
                    mcu_calibrating=fb.calibrating, mcu_servo_debug=fb.servo_debugging,
                    mcu_seq=fb.seq, mcu_period_us=fb.period_us, mcu_work_us=fb.work_us,
                    mcu_overruns=fb.overruns, mcu_report_drops=fb.report_drops,
                    mcu_invalid_commands=fb.invalid_commands, mcu_command_seq=fb.command_seq,
                    mcu_command_age_us=fb.command_age_us, angles=fb.angles,
                    velocities=fb.velocities, targets=fb.targets, acc=fb.acc, gyro=fb.gyro,
                    gravity=self.filtered['grav'] if self.filtered else None,
                    missing_joint_ids=[i for j,i in enumerate(MOTOR_IDS) if not fb.mask&(1<<j)])
            return result
