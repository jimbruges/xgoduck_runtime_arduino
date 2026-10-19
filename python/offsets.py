"""Servo position-offset references, saved on the robot in data/servo_offsets.json.

Calibration lives in each servo's position-offset register (31, EEPROM). If a servo loses
it (a write that only reached RAM, a replaced servo, a factory reset), its zero moves and
Default pose drives the joint to the wrong place. The host saves every servo's offset when
calibration or servo setup closes, reads them again at start and after a servo's power
returns, and refuses position modes until any changed joint is recalibrated.
"""
import json
import time
from pathlib import Path

PATH = Path(__file__).resolve().parents[1] / 'data' / 'servo_offsets.json'


def load():
    """{servo ID: offset} from the last save, or {} if there is none."""
    try:
        data = json.loads(PATH.read_text(encoding='utf-8'))
        return {int(k): int(v) for k, v in data['offsets'].items() if 0 <= int(v) <= 4095}
    except (OSError, KeyError, TypeError, ValueError, AttributeError, json.JSONDecodeError):
        return {}


def save(values):
    PATH.parent.mkdir(parents=True, exist_ok=True)
    PATH.write_text(json.dumps({
        'version': 1,
        'offsets': {str(k): int(v) for k, v in sorted(values.items())},
        'saved_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
    }, indent=2), encoding='utf-8')


def compare(reference, current):
    """Joints whose offset differs from the reference, as [{id, saved, now}]."""
    return [{'id': i, 'saved': reference[i], 'now': v}
            for i, v in sorted(current.items()) if i in reference and reference[i] != v]
