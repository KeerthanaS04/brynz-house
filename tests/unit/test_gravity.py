from types import SimpleNamespace

import numpy as np

from property_capture.calibration.gravity import axis_aligned_rotations, estimate_gravity
from property_capture.geometry.transforms import pose_matrices


def test_recovers_device_to_camera_rotation_and_up():
    rng = np.random.default_rng(3)
    n = 400
    q = rng.normal(size=(n, 4))
    T = pose_matrices(np.zeros((n, 3)), q)
    M_true = axis_aligned_rotations()[7]
    g_world = np.array([0.0, -1.0, 0.0])
    # a_device = M^T R_wc^T g_world  so that R_wc M a_device = g_world
    a = np.einsum('ji,nkj,k->ni', M_true, T[:, :3, :3], g_world)
    t = np.arange(n, dtype=float)
    cap = SimpleNamespace(timestamps=t, imu_timestamps=t, imu_accel=a + rng.normal(scale=0.01, size=a.shape))
    out = estimate_gravity(cap, T, {'imu_stride': 1, 'max_spread_deg': 10})
    assert np.allclose(out['device_to_camera_rotation'], M_true)
    assert np.allclose(out['up_world'], [0, 1, 0], atol=0.01)
    assert out['mean_angular_spread_deg'] < 2
    assert out['runner_up_spread_deg'] > 20
