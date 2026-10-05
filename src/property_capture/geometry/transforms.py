"""Rotations, rigid transforms and pinhole projection.

Camera axis conventions supported:
  opencv: x right, y down, z forward (looking along +z)
  arkit:  x right, y up,   z backward (looking along -z)
Pixel (u, v) = (column, row) with integer values at pixel centres.
"""
import numpy as np

CAMERA_AXES = ('arkit', 'opencv')
QUATERNION_ORDERS = ('xyzw', 'wxyz')


def quat_to_rotmat(q, order='xyzw'):
    """Unit quaternion(s) (..., 4) -> rotation matrices (..., 3, 3)."""
    q = np.asarray(q, dtype=float)
    if order == 'xyzw':
        x, y, z, w = np.moveaxis(q, -1, 0)
    elif order == 'wxyz':
        w, x, y, z = np.moveaxis(q, -1, 0)
    else:
        raise ValueError(f'unknown quaternion order {order!r}')
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    R = np.stack([
        1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
        2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
        2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
    ], axis=-1)
    return R.reshape(q.shape[:-1] + (3, 3))


def pose_matrices(positions, quaternions, order='xyzw'):
    """(N, 3) translations + (N, 4) quaternions -> (N, 4, 4) homogeneous transforms."""
    positions = np.asarray(positions, dtype=float)
    T = np.tile(np.eye(4), (len(positions), 1, 1))
    T[:, :3, :3] = quat_to_rotmat(quaternions, order)
    T[:, :3, 3] = positions
    return T


def invert(T):
    """Inverse of rigid transform(s) (..., 4, 4)."""
    R = T[..., :3, :3]
    t = T[..., :3, 3]
    Ti = np.zeros_like(T)
    Ti[..., :3, :3] = np.swapaxes(R, -1, -2)
    Ti[..., :3, 3] = -np.einsum('...ji,...j->...i', R, t)
    Ti[..., 3, 3] = 1.0
    return Ti


def transform(T, P):
    """Apply one 4x4 transform to points (N, 3)."""
    return P @ T[:3, :3].T + T[:3, 3]


def backproject(u, v, depth, K, axes):
    """Pixels + forward depth -> camera-frame points (N, 3) in the given axis convention."""
    x = (u - K[0, 2]) / K[0, 0] * depth
    y = (v - K[1, 2]) / K[1, 1] * depth
    if axes == 'opencv':
        return np.stack([x, y, depth], axis=-1)
    if axes == 'arkit':
        return np.stack([x, -y, -depth], axis=-1)
    raise ValueError(f'unknown camera axes {axes!r}')


def project(P, K, axes):
    """Camera-frame points (N, 3) -> (u, v, forward depth). Depth <= 0 means behind the camera."""
    X, Y, Z = P[:, 0], P[:, 1], P[:, 2]
    if axes == 'arkit':
        Y, Z = -Y, -Z
    elif axes != 'opencv':
        raise ValueError(f'unknown camera axes {axes!r}')
    with np.errstate(divide='ignore', invalid='ignore'):
        u = K[0, 0] * X / Z + K[0, 2]
        v = K[1, 1] * Y / Z + K[1, 2]
    return u, v, Z


def scale_intrinsics(K, scale):
    """Intrinsics for an image resampled by `scale`, keeping pixel centres aligned."""
    Ks = np.array(K, dtype=float, copy=True)
    Ks[0, 0] *= scale
    Ks[1, 1] *= scale
    Ks[0, 2] = (K[0, 2] + 0.5) * scale - 0.5
    Ks[1, 2] = (K[1, 2] + 0.5) * scale - 0.5
    return Ks
