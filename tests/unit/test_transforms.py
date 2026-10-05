import numpy as np
import pytest

from property_capture.geometry.transforms import (backproject, invert, pose_matrices, project,
                                                  quat_to_rotmat, scale_intrinsics, transform)


def test_identity_quaternion():
    assert np.allclose(quat_to_rotmat([0, 0, 0, 1], 'xyzw'), np.eye(3))
    assert np.allclose(quat_to_rotmat([1, 0, 0, 0], 'wxyz'), np.eye(3))


def test_90_deg_about_z_maps_x_to_y():
    s = np.sqrt(0.5)
    R = quat_to_rotmat([0, 0, s, s], 'xyzw')
    assert np.allclose(R @ [1, 0, 0], [0, 1, 0])
    assert np.allclose(quat_to_rotmat([s, 0, 0, s], 'wxyz'), R)


def test_rotation_is_orthonormal():
    q = np.random.default_rng(0).normal(size=(50, 4))
    R = quat_to_rotmat(q)
    assert np.allclose(R @ np.swapaxes(R, -1, -2), np.eye(3), atol=1e-12)
    assert np.allclose(np.linalg.det(R), 1)


def test_invert_rigid_transform():
    rng = np.random.default_rng(1)
    T = pose_matrices(rng.normal(size=(10, 3)), rng.normal(size=(10, 4)))
    assert np.allclose(invert(T) @ T, np.eye(4), atol=1e-12)


@pytest.mark.parametrize('axes', ['arkit', 'opencv'])
def test_backproject_project_round_trip(axes):
    K = np.array([[213.3, 0, 127.0], [0, 213.3, 95.3], [0, 0, 1]])
    u, v = np.meshgrid(np.arange(0, 256, 17.0), np.arange(0, 192, 13.0))
    d = np.linspace(0.5, 4.0, u.size)
    P = backproject(u.ravel(), v.ravel(), d, K, axes)
    u2, v2, d2 = project(P, K, axes)
    assert np.allclose(u2, u.ravel()) and np.allclose(v2, v.ravel()) and np.allclose(d2, d)


def test_arkit_camera_looks_along_negative_z():
    K = np.array([[200.0, 0, 100], [0, 200, 100], [0, 0, 1]])
    P = backproject(np.array([100.0]), np.array([100.0]), np.array([2.0]), K, 'arkit')
    assert np.allclose(P, [[0, 0, -2]])


def test_transform_matches_matrix_product():
    T = pose_matrices([[1, 2, 3]], [[0, 0, np.sqrt(0.5), np.sqrt(0.5)]])[0]
    assert np.allclose(transform(T, np.array([[1.0, 0, 0]])), [[1, 3, 3]])


def test_scale_intrinsics_keeps_pixel_centres():
    K = np.array([[1600.0, 0, 959.5], [0, 1600, 719.5], [0, 0, 1]])
    Ks = scale_intrinsics(K, 256 / 1920)
    # image centre stays the image centre after downsampling
    assert np.isclose(Ks[0, 2], 127.5) and np.isclose(Ks[1, 2], 95.5)
    assert np.isclose(Ks[0, 0], 1600 * 256 / 1920)
