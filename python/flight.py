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


def summarize(samples):
    """Last contiguous Walk run: fall, resting pitch, and per-command follow ratio and lag.

    Follow ratio is the achieved swing over the commanded swing of each joint; 1.0 means the
    servo reaches what the policy asks. Only upright samples in the walk phase are counted.
    """
    idx = [i for i, s in enumerate(samples) if s[1] == 'policy']
    if not idx:
        return {'walk': False}
    end = idx[-1]
    start = end
    while start > 0 and samples[start - 1][1] == 'policy' and \
            samples[start][0] - samples[start - 1][0] < 0.2:
        start -= 1
    W = samples[start:end + 1]
    t0 = W[0][0]
    fell = next((s for s in W if s[2] != 'walk'), None)
    up = [s for s in W if s[2] == 'walk' and abs(_pitch(s)) < 20]
    groups = {}
    for s in up:
        groups.setdefault(_group(s), []).append(s)
    out = {'walk': True, 'seconds': round(W[-1][0] - t0, 1),
           'fell_at_s': round(fell[0] - t0, 2) if fell else None,
           'groups': {}}
    still = groups.get('still', [])
    if still:
        out['still_pitch_deg'] = round(sum(map(_pitch, still)) / len(still), 1)
    for name, G in groups.items():
        if len(G) < 15:
            continue
        joints = {}
        for j, label in enumerate(JOINTS):
            ratio, lag = _follow(G, j)
            if ratio is not None:
                joints[label] = {'ratio': round(ratio, 2), 'lag_ms': lag}
        legs = [joints[JOINTS[j]] for j in LEGS if JOINTS[j] in joints]
        out['groups'][name] = {
            'ticks': len(G),
            'peak_vx': round(max((s[8][0] for s in G), key=abs), 2),
            'legs_ratio': round(sum(x['ratio'] for x in legs) / len(legs), 2) if legs else None,
            'legs_lag_ms': round(sum(x['lag_ms'] for x in legs) / len(legs)) if legs else None,
            'joints': joints,
        }
    return out
