"""Walk tuning saved on the robot in data/tuning.json, set from the Control page."""
import json
import math
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
}


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
