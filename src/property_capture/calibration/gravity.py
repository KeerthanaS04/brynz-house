"""Estimate the world gravity direction from the accelerometer and poses.

The accelerometer is in the device frame, the poses in the camera frame,
and the device-to-camera rotation is not in the archive. For the correct
rotation M, R_wc(t) @ M @ a(t) points the same way at every t, so each of
the 24 axis-aligned rotations is scored by the angular spread of that
direction. Assumes the accelerometer reports the gravity direction
(pointing down at rest, CoreMotion convention); the floor check in the
pipeline validates the resulting sign.
"""
import itertools

import numpy as np


def axis_aligned_rotations():
    out = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            M = np.zeros((3, 3))
            M[range(3), perm] = signs
            if np.linalg.det(M) > 0:
                out.append(M)
    return out


def nearest_index(sorted_t, t):
    idx = np.clip(np.searchsorted(sorted_t, t), 1, len(sorted_t) - 1)
    left = sorted_t[idx - 1]
    return np.where(np.abs(t - left) <= np.abs(sorted_t[idx] - t), idx - 1, idx)


def estimate_gravity(cap, T_c2w, cfg):
    t = cap.imu_timestamps[::cfg['imu_stride']]
    a = cap.imu_accel[::cfg['imu_stride']]
    inside = (t >= cap.timestamps[0]) & (t <= cap.timestamps[-1])
    t, a = t[inside], a[inside]
    R = T_c2w[nearest_index(cap.timestamps, t), :3, :3]
    an = a / np.linalg.norm(a, axis=1, keepdims=True)

    scored = []
    for M in axis_aligned_rotations():
        g = np.einsum('nij,jk,nk->ni', R, M, an)
        m = g.mean(axis=0)
        mu = m / np.linalg.norm(m)
        spread = float(np.degrees(np.mean(np.arccos(np.clip(g @ mu, -1, 1)))))
        scored.append((spread, M, mu))
    scored.sort(key=lambda s: s[0])
    spread, M, mu = scored[0]
    return {
        'method': 'accelerometer rotated into world by poses; best of 24 axis-aligned device-to-camera rotations',
        'samples': int(len(t)),
        'device_to_camera_rotation': M.astype(int).tolist(),
        'gravity_direction_world': mu.tolist(),
        'up_world': (-mu).tolist(),
        'mean_angular_spread_deg': spread,
        'runner_up_spread_deg': float(scored[1][0]),
        'consistent': spread <= cfg['max_spread_deg'],
        'assumption': 'accelerometer reports gravity direction (down at rest)',
    }
