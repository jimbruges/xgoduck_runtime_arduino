"""Host-side Bluetooth gamepad helper for the XGO Duck app.

The App Lab container has no access to BlueZ (D-Bus) or /dev/input, so this runs on the
Uno Q host as the `arduino` user (groups bluetooth and input) and:

  * pairs, trusts, connects and forgets controllers through `bluetoothctl`,
  * reads the first connected gamepad from /dev/input/event*,
  * posts its normalised state to the app (POST /api/pad) at 50 Hz while connected.

HTTP API on port 9528, accepted from loopback and Docker bridge addresses only:
  GET  /bt/status                      adapter state, devices, active pad
  POST /bt/scan {"seconds": 20}        discover nearby devices
  POST /bt/pair|connect|disconnect|remove {"mac": "AA:BB:..."}

Standard library only: python3 host/pad_helper.py
"""
import fcntl
import ipaddress
import json
import os
import re
import select
import struct
import subprocess
import threading
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PORT = int(os.environ.get('PAD_HELPER_PORT', '9528'))
APP_URL = os.environ.get('PAD_APP_URL', 'http://127.0.0.1:9527/api/pad')
SEND_HZ = 50
ALLOWED = [ipaddress.ip_network(n) for n in ('127.0.0.0/8', '172.16.0.0/12', '::1/128')]
MAC = re.compile(r'^[0-9A-F]{2}(:[0-9A-F]{2}){5}$')

# struct input_event on 64-bit Linux: timeval (2 x long), type, code, value.
EVENT = struct.Struct('llHHi')
EV_KEY, EV_ABS = 0x01, 0x03
BTN_GAMEPAD, BTN_JOYSTICK = 0x130, 0x120
BUTTON_NAMES = {
    0x130: 'south', 0x131: 'east', 0x132: 'c', 0x133: 'north', 0x134: 'west', 0x135: 'z',
    0x136: 'l1', 0x137: 'r1', 0x138: 'l2', 0x139: 'r2', 0x13a: 'select', 0x13b: 'start',
    0x13c: 'mode', 0x13d: 'thumbl', 0x13e: 'thumbr',
    0x220: 'dpad_up', 0x221: 'dpad_down', 0x222: 'dpad_left', 0x223: 'dpad_right',
}
AXIS_NAMES = {0x00: 'lx', 0x01: 'ly', 0x02: 'lz', 0x03: 'rx', 0x04: 'ry', 0x05: 'rz',
              0x09: 'gas', 0x0a: 'brake'}
TRIGGERS = {0x09, 0x0a}  # ABS_GAS/ABS_BRAKE are one-way: reported 0 (released) .. 1
HAT_X, HAT_Y = 0x10, 0x11


def eviocgabs(code):
    # _IOR('E', 0x40 + abs, struct input_absinfo), sizeof = 6 x int32 = 24.
    return (2 << 30) | (24 << 16) | (ord('E') << 8) | (0x40 + code)


def _bits(path):
    try:
        words = open(path).read().split()
    except OSError:
        return set()
    bits = set()
    for i, word in enumerate(reversed(words)):
        value = int(word, 16)
        for b in range(64):
            if value >> b & 1:
                bits.add(i * 64 + b)
    return bits


def find_gamepads():
    pads = []
    for name in sorted(os.listdir('/sys/class/input')):
        if not name.startswith('event'):
            continue
        base = f'/sys/class/input/{name}/device'
        keys = _bits(f'{base}/capabilities/key')
        if not _bits(f'{base}/capabilities/abs'):
            continue
        if not any(BTN_JOYSTICK <= k < BTN_JOYSTICK + 0x20 for k in keys):
            continue
        try:
            label = open(f'{base}/name').read().strip()
        except OSError:
            label = name
        try:
            uniq = open(f'{base}/uniq').read().strip().upper()
        except OSError:
            uniq = ''
        pads.append({'path': f'/dev/input/{name}', 'name': label, 'mac': uniq})
    return pads


class Pad:
    """Reads one evdev gamepad and keeps a normalised snapshot."""

    def __init__(self, info):
        self.info = info
        self.fd = os.open(info['path'], os.O_RDONLY | os.O_NONBLOCK)
        self.ranges = {}
        for code in list(AXIS_NAMES) + [HAT_X, HAT_Y]:
            buf = bytearray(24)
            try:
                fcntl.ioctl(self.fd, eviocgabs(code), buf)
            except OSError:
                continue
            _value, lo, hi = struct.unpack_from('iii', buf)
            if hi > lo:
                self.ranges[code] = (lo, hi)
        self.buttons = set()
        self.axes = {AXIS_NAMES[c]: 0.0 for c in self.ranges if c in AXIS_NAMES}
        self.hat = [0, 0]

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass

    def _norm(self, code, value):
        lo, hi = self.ranges[code]
        unit = max(0.0, min(1.0, (value - lo) / (hi - lo)))
        return unit if code in TRIGGERS else 2.0 * unit - 1.0

    def read(self):
        """Drain pending events. Raises OSError when the device disappears."""
        while True:
            try:
                data = os.read(self.fd, EVENT.size * 64)
            except BlockingIOError:
                return
            if not data:
                raise OSError('device closed')
            for off in range(0, len(data) - EVENT.size + 1, EVENT.size):
                _s, _us, etype, code, value = EVENT.unpack_from(data, off)
                if etype == EV_KEY:
                    name = BUTTON_NAMES.get(code, f'key_{code:#x}')
                    (self.buttons.add if value else self.buttons.discard)(name)
                elif etype == EV_ABS and code in (HAT_X, HAT_Y):
                    self.hat[code - HAT_X] = (value > 0) - (value < 0)
                elif etype == EV_ABS and code in self.ranges and code in AXIS_NAMES:
                    self.axes[AXIS_NAMES[code]] = round(self._norm(code, value), 4)

    def snapshot(self):
        buttons = set(self.buttons)
        hx, hy = self.hat
        if hx < 0: buttons.add('dpad_left')
        if hx > 0: buttons.add('dpad_right')
        if hy < 0: buttons.add('dpad_up')
        if hy > 0: buttons.add('dpad_down')
        return {'connected': True, 'name': self.info['name'], 'mac': self.info['mac'],
                'buttons': sorted(buttons), 'axes': dict(self.axes)}


def _post(url, payload):
    req = urllib.request.Request(url, json.dumps(payload).encode(),
                                 {'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=0.5) as resp:
        resp.read()


class PadLoop(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True, name='pad')
        self.pad = None
        self.lock = threading.Lock()
        self.last_error = ''

    def state(self):
        with self.lock:
            return self.pad.snapshot() if self.pad else {'connected': False}

    def run(self):
        next_scan = next_send = 0.0
        while True:
            now = time.monotonic()
            if self.pad is None and now >= next_scan:
                next_scan = now + 1.0
                pads = find_gamepads()
                if pads:
                    try:
                        pad = Pad(pads[0])
                        with self.lock:
                            self.pad = pad
                    except OSError as exc:
                        self.last_error = f'open {pads[0]["path"]}: {exc}'
            if self.pad is None:
                time.sleep(0.2)
                continue
            readable, _, _ = select.select([self.pad.fd], [], [], 1.0 / SEND_HZ)
            try:
                if readable:
                    with self.lock:
                        self.pad.read()
            except OSError:
                with self.lock:
                    self.pad.close()
                    self.pad = None
                self._send({'connected': False})
                continue
            if time.monotonic() >= next_send:
                next_send = time.monotonic() + 1.0 / SEND_HZ
                self._send(self.state())

    def _send(self, payload):
        try:
            _post(APP_URL, payload)
            self.last_error = ''
        except OSError as exc:
            self.last_error = f'app unreachable: {exc}'


def log(message):
    # The service runs under systemd, so this lands in `journalctl --user -u xgoduck-pad`.
    print(time.strftime('%Y-%m-%d %H:%M:%S'), message, flush=True)


def rfkill_state():
    """'soft-blocked', 'hard-blocked' or '' for the Bluetooth radio (readable without root)."""
    base = '/sys/class/rfkill'
    try:
        names = os.listdir(base)
    except OSError:
        return ''
    for name in names:
        try:
            if open(f'{base}/{name}/type').read().strip() != 'bluetooth':
                continue
            if open(f'{base}/{name}/hard').read().strip() == '1':
                return 'hard-blocked'
            if open(f'{base}/{name}/soft').read().strip() == '1':
                return 'soft-blocked'
        except OSError:
            continue
    return ''


def btctl(*args, timeout=30):
    out = subprocess.run(['bluetoothctl', *args], capture_output=True, text=True,
                         timeout=timeout)
    return out.stdout + out.stderr


def device_info(mac):
    info = {'mac': mac}
    for line in btctl('info', mac, timeout=5).splitlines():
        key, _, value = line.strip().partition(': ')
        if key in ('Name', 'Alias', 'Icon'):
            info[key.lower()] = value
        elif key in ('Paired', 'Trusted', 'Connected', 'Bonded'):
            info[key.lower()] = value == 'yes'
        elif key == 'RSSI':
            # "0xffffffc4 (-60)" or "-60"
            m = re.search(r'\((-?\d+)\)', value) or re.fullmatch(r'(-?\d+)', value.strip())
            info['rssi'] = int(m.group(1)) if m else None
    return info


class Bluetooth:
    def __init__(self):
        self.scan_until = 0.0
        self.scan_proc = None
        self.lock = threading.Lock()
        self.autopair_state = {'active': False, 'message': ''}
        self.power_note = ''
        threading.Thread(target=self._keep_powered, daemon=True, name='bt-power').start()

    def _keep_powered(self):
        """A bonded controller cannot reconnect to an unpowered adapter, so check every 10 s
        and power it back on, logging what was found.

        After long uptimes the Uno Q's Qualcomm Bluetooth core can also crash ("hci0: command
        tx timeout", then "Reading QCA version information failed"): the adapter then sticks
        at "PowerState: off-enabling" and power on answers org.bluez.Error.Busy, later Failed.
        Rebinding the hci_uart_qca driver did not recover it on the development robot;
        restarting the Uno Q did. That case is reported as needing a restart."""
        last, stuck_since = '', 0.0
        while True:
            try:
                show = btctl('show', timeout=8)
                if 'Powered: yes' in show:
                    if last:
                        log('bluetooth: adapter powered')
                    last, stuck_since = '', 0.0
                else:
                    why = ('no controller' if 'No default controller' in show else 'adapter off')
                    block = rfkill_state()
                    if block:
                        why += f', radio {block}'
                    reply = btctl('power', 'on', timeout=8).strip().splitlines()
                    ok = 'Powered: yes' in btctl('show', timeout=8)
                    what = (f'{why}; powered back on' if ok else
                            f'{why}; power on failed: {reply[-1] if reply else "no reply"}')
                    # Busy or Failed for a minute, with the radio not blocked: the chip is gone.
                    wedged = not ok and not block
                    stuck_since = (stuck_since or time.monotonic()) if wedged else 0.0
                    if stuck_since and time.monotonic() - stuck_since > 60:
                        what = ('the Bluetooth chip has stopped responding; restart the Uno Q '
                                'to recover it')
                    self.power_note = f'{time.strftime("%H:%M:%S")} {what}'
                    if what != last:  # once per change, not every 10 s while it stays down
                        log(f'bluetooth: {what}')
                    last = '' if ok else what
            except (OSError, subprocess.SubprocessError) as exc:
                self.power_note = f'{time.strftime("%H:%M:%S")} bluetoothctl failed: {exc}'
                log(f'bluetooth: {self.power_note}')
            time.sleep(10)

    def _listed(self, *filt):
        """MAC -> name from `bluetoothctl devices [filter]`."""
        found = {}
        for line in btctl('devices', *filt, timeout=5).splitlines():
            parts = line.split(' ', 2)
            if len(parts) >= 2 and parts[0] == 'Device' and MAC.match(parts[1]):
                found[parts[1]] = parts[2] if len(parts) > 2 else ''
        return found

    def status(self):
        powered = 'Powered: yes' in btctl('show', timeout=5)
        listed = self._listed()
        flags = {f: set(self._listed(f)) for f in ('Paired', 'Trusted', 'Connected')}
        # A controller advertises a name; bluetoothctl shows nameless devices by address.
        named = {m for m, n in listed.items() if n and n.replace('-', ':').upper() != m}
        keep = named | flags['Paired']
        # `info` (icon, RSSI) costs a subprocess per device, so only for the ones shown.
        with ThreadPoolExecutor(max_workers=8) as pool:
            devices = list(pool.map(device_info, sorted(keep)))
        for d in devices:
            d.setdefault('name', listed.get(d['mac'], ''))
            for f in flags:
                d[f.lower()] = d['mac'] in flags[f]
        devices.sort(key=lambda d: (not d.get('connected'), not d.get('paired'),
                                    d.get('icon') != 'input-gaming', -(d.get('rssi') or -999)))
        return {'powered': powered, 'power_note': self.power_note,
                'scanning': time.monotonic() < self.scan_until,
                'autopair': dict(self.autopair_state),
                'hidden_unnamed': len(listed) - len(keep),
                'scan_left_s': max(0, round(self.scan_until - time.monotonic())),
                'devices': devices}

    def scan(self, seconds):
        seconds = max(5, min(60, int(seconds)))
        with self.lock:
            if self.scan_proc and self.scan_proc.poll() is None:
                self.scan_proc.terminate()
            btctl('power', 'on', timeout=5)
            self.scan_proc = subprocess.Popen(
                ['bluetoothctl', '--timeout', str(seconds), 'scan', 'on'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.scan_until = time.monotonic() + seconds
        return {'ok': True, 'seconds': seconds}

    def autopair(self, seconds):
        """Scan and pair the first unpaired gamepad that starts advertising. Controllers such
        as the GameSir Nova Lite use a different address per mode, so match on the gamepad
        icon rather than a known address."""
        if self.autopair_state['active']:
            return dict(self.autopair_state)
        seconds = max(10, min(120, int(seconds)))
        self.autopair_state = {'active': True, 'message': 'Waiting for a controller in pairing mode…'}

        def run():
            deadline = time.monotonic() + seconds
            try:
                self.scan(min(60, seconds))
                while time.monotonic() < deadline:
                    if time.monotonic() > self.scan_until:
                        self.scan(min(60, max(5, deadline - time.monotonic())))
                    paired = set(self._listed('Paired'))
                    named = {m: n for m, n in self._listed().items()
                             if n and n.replace('-', ':').upper() != m and m not in paired}
                    live = [d for d in map(device_info, named)
                            if d.get('icon') == 'input-gaming' and d.get('rssi') is not None]
                    if live:
                        d = max(live, key=lambda d: d['rssi'])
                        name = d.get('name') or d['mac']
                        self.autopair_state = {'active': True, 'message': f'Pairing {name}…'}
                        info = self.pair(d['mac'])
                        self.autopair_state = {'active': False, 'mac': d['mac'],
                                               'message': f'Paired and connected {info.get("name") or name}'}
                        return
                    time.sleep(1)
                self.stop_scan()
                self.autopair_state = {'active': False, 'message': 'No controller in pairing mode was found'}
            except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
                self.autopair_state = {'active': False, 'message': f'Pairing failed: {exc}'}

        threading.Thread(target=run, daemon=True, name='autopair').start()
        return dict(self.autopair_state)

    def stop_scan(self):
        with self.lock:
            if self.scan_proc and self.scan_proc.poll() is None:
                self.scan_proc.terminate()
            self.scan_until = 0.0
        btctl('scan', 'off', timeout=5)

    def pair(self, mac):
        # Discovery running alongside pairing makes some adapters fail the connection.
        self.stop_scan()
        # Without "pairable on" BlueZ pairs but stores no bond, and the input profile then
        # rejects the controller's HID connection ("Rejected connection from !bonded device").
        btctl('pairable', 'on', timeout=5)
        # Gamepads use "just works" pairing; the agent accepts it without a PIN.
        log = btctl('--agent', 'NoInputNoOutput', '--timeout', '25', 'pair', mac, timeout=35)
        if 'Failed' in log and 'AlreadyExists' not in log:
            raise RuntimeError(log.strip().splitlines()[-1] +
                               ' (is the controller still in pairing mode?)')
        if not device_info(mac).get('bonded'):
            raise RuntimeError('pairing did not complete; put the controller back in '
                               'pairing mode and try again')
        btctl('trust', mac, timeout=10)
        log = btctl('connect', mac, timeout=20)
        info = device_info(mac)
        if not info.get('connected'):
            raise RuntimeError('paired, but the controller did not connect: '
                               + (log.strip().splitlines() or ['no reply'])[-1])
        return info

    def simple(self, action, mac):
        btctl(action, mac, timeout=20)
        return device_info(mac) if action != 'remove' else {'mac': mac, 'removed': True}


class Handler(BaseHTTPRequestHandler):
    bt: Bluetooth = None
    pads: PadLoop = None

    def log_message(self, *_args):
        pass

    def _allowed(self):
        try:
            ip = ipaddress.ip_address(self.client_address[0])
        except ValueError:
            return False
        return any(ip in net for net in ALLOWED)

    def _reply(self, code, payload):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if not self._allowed():
            return self._reply(403, {'detail': 'forbidden'})
        if self.path == '/bt/status':
            try:
                status = self.bt.status()
            except (OSError, subprocess.SubprocessError) as exc:
                return self._reply(500, {'detail': str(exc)})
            status['pad'] = self.pads.state()
            status['helper_error'] = self.pads.last_error
            return self._reply(200, status)
        self._reply(404, {'detail': 'not found'})

    def do_POST(self):
        if not self._allowed():
            return self._reply(403, {'detail': 'forbidden'})
        try:
            length = int(self.headers.get('Content-Length') or 0)
            body = json.loads(self.rfile.read(length) or b'{}')
            action = self.path.removeprefix('/bt/')
            if action == 'scan':
                return self._reply(200, self.bt.scan(body.get('seconds', 20)))
            if action == 'autopair':
                return self._reply(200, self.bt.autopair(body.get('seconds', 60)))
            mac = str(body.get('mac', '')).upper()
            if not MAC.match(mac):
                return self._reply(400, {'detail': 'invalid MAC address'})
            if action == 'pair':
                return self._reply(200, self.bt.pair(mac))
            if action in ('connect', 'disconnect', 'remove'):
                return self._reply(200, self.bt.simple(action, mac))
            self._reply(404, {'detail': 'not found'})
        except (ValueError, RuntimeError, OSError, subprocess.SubprocessError) as exc:
            self._reply(400, {'detail': str(exc)})


def main():
    pads = PadLoop()
    pads.start()
    Handler.bt, Handler.pads = Bluetooth(), pads
    ThreadingHTTPServer(('0.0.0.0', PORT), Handler).serve_forever()


if __name__ == '__main__':
    main()
