"""Summarise the flight recorder: how closely the servos followed the policy on the last walk."""
import math

JOINTS = ['L hip yaw', 'L hip roll', 'L hip pitch', 'L knee', 'L ankle',
          'R hip yaw', 'R hip roll', 'R hip pitch', 'R knee', 'R ankle',
          'neck pitch', 'head pitch', 'head yaw', 'head roll']
LEGS = range(10)
TICK_MS = 20


def _pitch(s):
    return math.degrees(math.asin(max(-1.0, min(1.0, s[4][0]))))


def _group(s):
    vx, vy, wz = s[8]
    if abs(vx) < 0.05 and abs(vy) < 0.05 and abs(wz) < 0.1:
        return 'still'
    return 'forward' if vx >= 0.05 else 'backward' if vx <= -0.05 else 'turn'


def _follow(G, j):
    """(achieved/commanded peak-to-peak, best-fit lag in ms) for joint j over samples G."""
    tg = [s[7][j] for s in G]
    an = [s[6][j] for s in G]
    tp = max(tg) - min(tg)
    if tp < 2.0:
        return None, None
    lag = min(range(6), key=lambda k: sum((G[i][7][j] - G[i + k][6][j]) ** 2
                                          for i in range(len(G) - k)) / (len(G) - k))
    return (max(an) - min(an)) / tp, lag * TICK_MS


def _runs(samples):
    """Split Walk (policy-mode) samples into contiguous runs."""
    runs, cur = [], []
    for s in samples:
        if s[1] == 'policy' and cur and s[0] - cur[-1][0] < 0.2:
            cur.append(s)
        else:
            if cur:
                runs.append(cur)
            cur = [s] if s[1] == 'policy' else []
    if cur:
        runs.append(cur)
    return runs


def summarize(samples):
    """All Walk time in the recorder: falls, resting pitch, per-command follow ratio and lag.

    Follow ratio is the achieved swing over the commanded swing of each joint; 1.0 means the
    servo reaches what the policy asks. Only upright samples in the walk phase are counted,
    and lag is fitted within each contiguous run.
    """
    runs = _runs(samples)
    if not runs:
        return {'walk': False}
    W = [s for r in runs for s in r]
    falls = [r[next(i for i, s in enumerate(r) if s[2] != 'walk')][0] - samples[0][0]
             for r in runs if any(s[2] != 'walk' for s in r)]
    # Contiguous stretches of one command type while upright, each at least 0.3 s.
    groups = {}
    for r in runs:
        stretch, label = [], None
        for s in r + [None]:
            name = _group(s) if s and s[2] == 'walk' and abs(_pitch(s)) < 20 else None
            if name != label or s is None:
                if label and len(stretch) >= 15:
                    groups.setdefault(label, []).append(stretch)
                stretch, label = [], name
            if name:
                stretch.append(s)
    out = {'walk': True, 'seconds': round(len(W) * TICK_MS / 1000, 1), 'runs': len(runs),
           'falls': len(falls), 'window_s': round(samples[-1][0] - samples[0][0], 1),
           'groups': {}}
    still = [s for G in groups.get('still', []) for s in G]
    if still:
        out['still_pitch_deg'] = round(sum(map(_pitch, still)) / len(still), 1)
    for name, chunks in groups.items():
        G = [s for c in chunks for s in c]
        joints = {}
        for j, label in enumerate(JOINTS):
            # Tick-weighted average over the chunks, so lag is never fitted across a gap.
            parts = [(len(c), *_follow(c, j)) for c in chunks]
            parts = [p for p in parts if p[1] is not None]
            if parts:
                n = sum(p[0] for p in parts)
                joints[label] = {'ratio': round(sum(p[0] * p[1] for p in parts) / n, 2),
                                 'lag_ms': round(sum(p[0] * p[2] for p in parts) / n)}
        legs = [joints[JOINTS[j]] for j in LEGS if JOINTS[j] in joints]
        out['groups'][name] = {
            'ticks': len(G),
            'peak_vx': round(max((s[8][0] for s in G), key=abs), 2),
            'legs_ratio': round(sum(x['ratio'] for x in legs) / len(legs), 2) if legs else None,
            'legs_lag_ms': round(sum(x['lag_ms'] for x in legs) / len(legs)) if legs else None,
            'joints': joints,
        }
    return out
