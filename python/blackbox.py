"""Crash-safe telemetry: the last minutes of robot state, kept on disk through a power cut.

The flight recorder lives in memory, so a power cut loses it, and the moments before a cut
are the ones that matter. This writes one compact record every 40 ms (25 Hz) to
data/blackbox/current.jsonl and syncs it to storage every 0.25 s, so at most the last
quarter second is lost. At each app start the previous run's file becomes previous.jsonl;
after an unexpected power-off, that file ends at the cut.

Each record: t (Unix s), mode, phase, en (torque enabled), tilt (largest angle from upright in the window, deg, as the fall detector sees it),
acc (peak |acceleration| in the window, m/s²; the IMU saturates near 2 g = 19.6), gyro
(peak |rate|, deg/s), srv (fewest servos answering in a window tick), lim (MCU supply
guard: 0 full torque, 1 limited to 50 %, 2 to 20 %), vmin/vmax (servo
supply, V, from the latest raw packet), tmax (hottest servo, °C).
"""
import json
import math
import os
import threading
import time
from pathlib import Path

DIR = Path(__file__).resolve().parents[1] / 'data' / 'blackbox'
WINDOW_S = 0.04
SYNC_S = 0.25
MAX_BYTES = 4_000_000  # about 25 min at 25 Hz; then it starts again (previous file kept)


class BlackBox:
    def __init__(self):
        DIR.mkdir(parents=True, exist_ok=True)
        current = DIR / 'current.jsonl'
        if current.exists():
            current.replace(DIR / 'previous.jsonl')
        self.path = current
        self.lock = threading.Lock()
        self.lines = []
        self.window = None
        self.fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
        self.written = 0
        threading.Thread(target=self._writer, daemon=True, name='blackbox').start()

    def sample(self, now, fb, mode, phase, tilt, volts, temps, servo_mask):
        """Fold one 100 Hz state packet into the current 40 ms window."""
        acc = math.sqrt(sum(a * a for a in fb.acc))
        gyro = math.sqrt(sum(g * g for g in fb.gyro))
        servos = bin(fb.mask).count('1')
        w = self.window
        if w is None:
            self.window = w = {'start': now, 'acc': 0.0, 'gyro': 0.0, 'srv': 15, 'tilt': 0.0, 'lim': 0}
        w['acc'] = max(w['acc'], acc)
        w['gyro'] = max(w['gyro'], gyro)
        w['srv'] = min(w['srv'], servos)
        w['lim'] = max(w['lim'], getattr(fb, 'supply_guard', 0))
        if tilt is not None:
            w['tilt'] = max(w['tilt'], tilt)
        if now - w['start'] < WINDOW_S:
            return
        present = [v for i, v in enumerate(volts) if servo_mask >> i & 1 and v]
        record = {'t': round(time.time(), 2), 'mode': mode, 'phase': phase, 'en': int(fb.enabled),
                  'tilt': round(w['tilt'], 1), 'acc': round(w['acc'], 1), 'gyro': round(w['gyro']),
                  'srv': w['srv'], 'lim': w['lim'],
                  'vmin': min(present) / 10 if present else None,
                  'vmax': max(present) / 10 if present else None,
                  'tmax': max((t for i, t in enumerate(temps) if servo_mask >> i & 1), default=None)}
        self.window = None
        with self.lock:
            self.lines.append(json.dumps(record, separators=(',', ':')))

    def _writer(self):
        while True:
            time.sleep(SYNC_S)
            with self.lock:
                lines, self.lines = self.lines, []
            if not lines:
                continue
            data = ('\n'.join(lines) + '\n').encode()
            try:
                if self.written + len(data) > MAX_BYTES:
                    os.close(self.fd)
                    self.path.replace(DIR / 'earlier.jsonl')
                    self.fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
                    self.written = 0
                os.write(self.fd, data)
                os.fsync(self.fd)  # on storage before the next window, so a cut loses <= 0.25 s
                self.written += len(data)
            except OSError:
                pass


def read(name='previous', seconds=20.0):
    """The last `seconds` of a run: 'previous' (the run before this app start) or 'current'."""
    path = DIR / f'{name}.jsonl'
    try:
        size = path.stat().st_size
        with open(path, 'rb') as f:
            f.seek(max(0, size - 400_000))
            lines = f.read().decode(errors='replace').splitlines()
    except OSError:
        return {'file': name, 'records': []}
    records = []
    for line in lines:
        try:
            records.append(json.loads(line))
        except ValueError:
            continue
    if records:
        end = records[-1]['t']
        records = [r for r in records if r['t'] >= end - seconds]
    return {'file': name, 'records': records,
            'ended': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(records[-1]['t'])) if records else None}
