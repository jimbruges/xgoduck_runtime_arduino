"""Walk tuning saved on the robot in data/tuning.json, set from the Control page."""
import json
import math
import re
from pathlib import Path

PATH = Path(__file__).resolve().parents[1] / 'data' / 'tuning.json'

DEFAULTS = {
    # 'table': the hand-tuned HOME_DEG (hip/ankle 23°) for every policy, as upstream.
    # 'model': each ONNX policy's own default_joint_pos (walk/get-up/pick 20°, roulade 24°).
    'home_source': 'table',
    # Time constant of the command low-pass on twist and head, seconds; 0 = off.
    'cmd_tau_s': 0.0,
    # IMU mounting tilt (roll, pitch, yaw) cancelled before the gravity filter, degrees.
    'imu_bias_deg': [0.0, 0.0, 0.0],
    # Servo run gains for Default pose and Walk (temporary KP/KD registers 50/51).
    # Firmware defaults are KP 6, KD 20; higher KP is stiffer, lower KD is quicker.
    'servo_kp': 6,
    'servo_kd': 20,
    # Degrees the hip pitch and ankle home angles move toward upright (body forward over
    # the feet), for the policies' home and Default pose. Compensates a CoM behind the
    # trained one; about 1.3 mm per degree.
    'home_lean_deg': 0.0,
    # Lean used when walking backward; the applied lean blends from home_lean_deg to this
    # as the commanded vx goes from 0 to -0.2 m/s. Equal to home_lean_deg = no blending.
    'home_lean_back_deg': 0.0,
    # Walk policy file in python/ (xgoduck_walk*.onnx), for A/B testing retrained models.
    'walk_model': 'xgoduck_walk.onnx',
    # Supply guard: below this servo supply voltage (0 = off) the MCU caps every servo's
    # torque at supply_guard_torque_pct, so a sagging battery recovers instead of cutting
    # out. 0.6 V lower it caps at 20 % (or lower if that is set lower).
    'supply_guard_v': 6.8,
    'supply_guard_torque_pct': 50,
}
WALK_MODEL = re.compile(r'^xgoduck_walk[\w.-]*\.onnx$')
MOUTH_KP = 10  # mouth servo run KP, as in firmware


def validate(values, base=None):
    out = dict(base or DEFAULTS)
    values = dict(values or {})
    if 'home_source' in values:
        if values['home_source'] not in ('table', 'model'):
            raise ValueError("home_source must be 'table' or 'model'")
        out['home_source'] = values['home_source']
    if 'cmd_tau_s' in values:
        tau = float(values['cmd_tau_s'])
        if not math.isfinite(tau) or not 0 <= tau <= 1.0:
            raise ValueError('cmd_tau_s must be within 0..1 s')
        out['cmd_tau_s'] = tau
    if 'servo_kp' in values:
        kp = int(values['servo_kp'])
        if not 1 <= kp <= 32:
            raise ValueError('servo KP must be within 1..32')
        out['servo_kp'] = kp
    if 'servo_kd' in values:
        kd = int(values['servo_kd'])
        if not 0 <= kd <= 40:
            raise ValueError('servo KD must be within 0..40')
        out['servo_kd'] = kd
    for key in ('home_lean_deg', 'home_lean_back_deg'):
        if key in values:
            lean = float(values[key])
            if not math.isfinite(lean) or not -10 <= lean <= 15:
                raise ValueError('home lean must be within -10..15°')
            out[key] = lean
    if 'supply_guard_v' in values:
        v = round(float(values['supply_guard_v']), 1)
        if not math.isfinite(v) or not (v == 0 or 5.5 <= v <= 8.0):
            raise ValueError('supply guard must be 0 (off) or within 5.5..8.0 V')
        out['supply_guard_v'] = v
    if 'supply_guard_torque_pct' in values:
        pct = int(values['supply_guard_torque_pct'])
        if not 20 <= pct <= 100:
            raise ValueError('limited torque must be within 20..100 %')
        out['supply_guard_torque_pct'] = pct
    if 'walk_model' in values:
        name = str(values['walk_model'])
        if not WALK_MODEL.match(name) or not (Path(__file__).parent / name).is_file():
            raise ValueError(f'walk model {name!r} not found in python/')
        out['walk_model'] = name
    if 'imu_bias_deg' in values:
        bias = [float(v) for v in values['imu_bias_deg']]
        if len(bias) != 3 or any(not math.isfinite(v) or abs(v) > 30 for v in bias):
            raise ValueError('imu_bias_deg must be 3 angles within ±30°')
        out['imu_bias_deg'] = bias
    return out


def load():
    try:
        return validate(json.loads(PATH.read_text(encoding='utf-8')))
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return dict(DEFAULTS)


def save(values):
    PATH.parent.mkdir(parents=True, exist_ok=True)
    PATH.write_text(json.dumps(values, indent=2), encoding='utf-8')
