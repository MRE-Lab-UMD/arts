"""Pose repeatability protocol helpers, pure numpy + scipy (no ROS), shared by the ARTS runner
(repeatability_real_arm.py) and the platform-neutral capture tool (tracker_pose_capture.py).

  load(path)                      the protocol (repeatability_protocol.yaml)
  pose_segment(a, b, p)           one straight move between two poses: position along the line,
                                  orientation by slerp, one trapezoid in the path parameter
  Static(p)                       the static rule over a sliding window of tracked poses
  schedule(p)                     [(cycle, pose)] for cycles x P1..Pn
"""
from collections import deque

import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def schedule(p):
    return [(c, k) for c in range(1, p['cycles'] + 1) for k in range(1, p['poses'] + 1)]


def _trapezoid(length, vmax, amax):
    """(duration, s(t), sdot(t)) for a rest-to-rest move of `length` (>= 0)."""
    if length <= 0.0:
        return 0.0, (lambda t: 0.0), (lambda t: 0.0)
    ta = vmax / amax
    if amax * ta * ta >= length:          # triangle: never reaches vmax
        ta = np.sqrt(length / amax)
        vmax = amax * ta
    tc = (length - amax * ta * ta) / vmax
    dur = 2 * ta + tc

    def s(t):
        t = min(max(t, 0.0), dur)
        if t < ta:
            return 0.5 * amax * t * t
        if t < ta + tc:
            return 0.5 * amax * ta * ta + vmax * (t - ta)
        r = dur - t
        return length - 0.5 * amax * r * r

    def sd(t):
        if t <= 0.0 or t >= dur:
            return 0.0
        if t < ta:
            return amax * t
        if t < ta + tc:
            return vmax
        return amax * (dur - t)
    return dur, s, sd


def pose_segment(a_pos, a_quat, b_pos, b_quat, p):
    """(duration, f(t) -> (pos, quat xyzw, linear vel, angular vel)) for a straight move from
    pose a to pose b. The position and the orientation share ONE normalized path parameter, so
    both arrive together; the slower of the two (speed, angular_speed) sets the duration."""
    a_pos, b_pos = np.asarray(a_pos, float), np.asarray(b_pos, float)
    ra, rb = Rotation.from_quat(a_quat), Rotation.from_quat(b_quat)
    dist = np.linalg.norm(b_pos - a_pos)
    rel = (ra.inv() * rb).as_rotvec()     # body-frame rotation vector a -> b
    ang = np.linalg.norm(rel)
    # Path parameter u in [0, 1]: cruise rate and ramp from whichever motion is longer in time.
    w = np.radians(p['angular_speed'])
    u_rate = min(p['speed'] / dist if dist > 1e-9 else np.inf, w / ang if ang > 1e-9 else np.inf)
    if not np.isfinite(u_rate):
        return 0.0, lambda t: (b_pos, rb.as_quat(), np.zeros(3), np.zeros(3))
    u_acc = u_rate * p['acceleration'] / p['speed']
    dur, s, sd = _trapezoid(1.0, u_rate, u_acc)
    slerp = Slerp([0.0, 1.0], Rotation.concatenate([ra, rb]))
    omega = ra.apply(rel)                 # world-frame angular velocity per unit u

    def f(t):
        u, ud = s(t), sd(t)
        return a_pos + u * (b_pos - a_pos), slerp([u])[0].as_quat(), ud * (b_pos - a_pos), ud * omega
    return dur, f


class Static:
    """The protocol's static rule over the last `window` s of (time, position, quaternion)."""

    def __init__(self, p, stale_s=0.1):
        self.p = p
        self.stale_s = stale_s
        self.buf = deque()

    def add(self, t, pos, quat):
        self.buf.append((t, np.asarray(pos, float), np.asarray(quat, float)))
        while self.buf and t - self.buf[0][0] > self.p['window']:
            self.buf.popleft()

    def clear(self):
        self.buf.clear()

    def test(self, now):
        """(ok, mean pos, mean quat, std mm xyz, why)."""
        if not self.buf or now - self.buf[-1][0] > self.stale_s:
            return False, None, None, None, 'marker not visible'
        ts = np.array([b[0] for b in self.buf])
        if ts[-1] - ts[0] < 0.9 * self.p['window'] or len(ts) < 10:
            return False, None, None, None, 'filling the window'
        pos = np.array([b[1] for b in self.buf])
        std = 1e3 * pos.std(axis=0)
        half = len(pos) // 2
        drift = 1e3 * np.linalg.norm(pos[:half].mean(axis=0) - pos[half:].mean(axis=0))
        quat = Rotation.from_quat(np.array([b[2] for b in self.buf])).mean().as_quat()
        mean = pos.mean(axis=0)
        if std.max() > self.p['static_std_mm'] or drift > self.p['static_drift_mm']:
            return False, mean, quat, std, f'moving: std {std.max():.4f} mm, drift {drift:.4f} mm'
        return True, mean, quat, std, (f'{len(pos)} samples, std {std.max():.4f} mm, '
                                       f'drift {drift:.4f} mm')
