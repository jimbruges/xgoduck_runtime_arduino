"""Gamepad button/axis mapping. State arrives from host/pad_helper.py via POST /api/pad."""
import json
import math
from pathlib import Path

CONFIG_PATH = Path(__file__).resolve().parents[1] / 'data' / 'controller.json'

# Button actions. Edge actions fire once on press; hold actions apply while held.
ACTIONS = {
    '': 'Nothing',
    'mode:off': 'Torque off',
    'mode:shadow': 'Inference only',
    'mode:hold': 'Default pose',
    'mode:policy': 'Walk / get up',
    'pick': 'Pick',
    'roulade': 'Roulade',
    'mouth': 'Open mouth (hold)',
    'slow': 'Half speed (hold)',
}
HOLD_ACTIONS = {'mouth', 'slow'}
AXIS_TARGETS = {
    '': 'Nothing',
    'vx': 'Forward / back',
    'vy': 'Strafe left / right',
    'wz': 'Turn',
    'head0': 'Neck pitch', 'head1': 'Head pitch', 'head2': 'Head yaw', 'head3': 'Head roll',
    'mouth': 'Mouth',
}
TWIST_LIMITS = {'vx': 0.4, 'vy': 0.3, 'wz': 1.0}  # same bounds as Controller.command
MOUTH_MAX = 30.0

DEFAULT_CONFIG = {
    'deadzone': 0.1,
    'buttons': {
        'south': 'mode:policy', 'east': 'mode:off', 'west': 'pick', 'north': 'roulade',
        'start': 'mode:hold', 'select': 'mode:shadow', 'l1': 'mouth', 'r1': 'slow',
    },
    # evdev reports stick "up" and "left" as negative, so invert for +vx forward, +vy left.
    'axes': {
        'ly': {'target': 'vx', 'invert': True, 'trigger': False},
        'lx': {'target': 'vy', 'invert': True, 'trigger': False},
        'rx': {'target': 'wz', 'invert': True, 'trigger': False},
    },
}


def validate(config):
    if not isinstance(config, dict):
        raise ValueError('config must be an object')
    dz = float(config.get('deadzone', 0.1))
    if not 0 <= dz <= 0.5:
        raise ValueError('deadzone must be within 0..0.5')
    buttons = {}
    for name, action in dict(config.get('buttons', {})).items():
        if action not in ACTIONS:
            raise ValueError(f'unknown action {action!r}')
        if action:
            buttons[str(name)[:32]] = action
    axes = {}
    for name, spec in dict(config.get('axes', {})).items():
        spec = dict(spec)
        if spec.get('target', '') not in AXIS_TARGETS:
            raise ValueError(f'unknown axis target {spec.get("target")!r}')
        if spec.get('target'):
            axes[str(name)[:32]] = {'target': spec['target'], 'invert': bool(spec.get('invert')),
                                    'trigger': bool(spec.get('trigger'))}
    return {'deadzone': dz, 'buttons': buttons, 'axes': axes}


def load():
    try:
        return validate(json.loads(CONFIG_PATH.read_text(encoding='utf-8')))
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return validate(DEFAULT_CONFIG)


def save(config):
    config = validate(config)
    CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    CONFIG_PATH.write_text(json.dumps(config, indent=2), encoding='utf-8')
    return config


class Mapper:
    def __init__(self):
        self.config = load()
        self.pressed = set()

    def reset(self):
        self.pressed = set()

    def apply(self, buttons, axes):
        """Return (edge actions, twist|None, head|None, mouth|None) for one pad sample."""
        cfg = self.config
        buttons = set(buttons)
        new = buttons - self.pressed
        self.pressed = buttons
        edge = [cfg['buttons'][b] for b in sorted(new)
                if cfg['buttons'].get(b) and cfg['buttons'][b] not in HOLD_ACTIONS]
        held = {cfg['buttons'][b] for b in buttons if cfg['buttons'].get(b) in HOLD_ACTIONS}

        values = {}
        for name, spec in cfg['axes'].items():
            v = float(axes.get(name, 0.0))
            if not math.isfinite(v):
                continue
            if spec['trigger']:
                v = (v + 1.0) / 2.0          # an axis resting at -1 (lz/rz on some pads) -> 0 .. 1
            elif abs(v) < cfg['deadzone']:
                v = 0.0
            if spec['invert']:
                v = -v
            values[spec['target']] = values.get(spec['target'], 0.0) + v

        targets = {s['target'] for s in cfg['axes'].values()}
        clamp = lambda v, lim: max(-lim, min(lim, v))
        twist = head = mouth = None
        if targets & set(TWIST_LIMITS):
            scale = 0.5 if 'slow' in held else 1.0
            twist = [clamp(values.get(k, 0.0) * lim * scale, lim) for k, lim in TWIST_LIMITS.items()]
        if any(t.startswith('head') for t in targets):
            head = [clamp(values.get(f'head{i}', 0.0), 1.0) for i in range(4)]
        if 'mouth' in targets or any(a == 'mouth' for a in cfg['buttons'].values()):
            # gas/brake arrive as 0..1; a stick opens the mouth in its positive direction only.
            m = max(0.0, min(1.0, values.get('mouth', 0.0))) * MOUTH_MAX
            mouth = MOUTH_MAX if 'mouth' in held else min(MOUTH_MAX, m)
        return edge, twist, head, mouth
