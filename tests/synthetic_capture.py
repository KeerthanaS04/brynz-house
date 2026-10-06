"""Synthetic LiDAR captures with known geometry, in the supplied captures' format.

A box room (x in [0, width], z in [0, length], floor y = 0, ceiling y = height; world +y up) is
ray-cast from a camera walking an ellipse at 1.4 m height while it turns and tilts. Written like
the capture app's export (assumptions.md B-20): depth/ and confidence/ PNGs (256 x 192, depth in
mm), odometry.csv (camera-to-world, xyzw quaternion, OpenCV camera axes, RGB intrinsics),
imu.csv (accelerometer = gravity direction in the camera frame, in g), camera_matrix.csv and a
small rgb.mp4 whose only role is to give the RGB resolution.

Every parameter can be changed to build broken or difficult captures for the adversarial tests.
"""
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

DEPTH_SIZE = (256, 192)
RGB_SIZE = (320, 240)
FX_RGB = 250.0


def rot_to_quat_xyzw(R):
    """Rotation matrix -> unit quaternion (x, y, z, w)."""
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    return np.array([x, y, z, w])


def camera_pose(position, yaw, pitch):
    """Camera-to-world, OpenCV axes (x right, y down, z forward); world +y up.
    yaw turns about world up, pitch > 0 looks up."""
    fwd = np.array([np.sin(yaw) * np.cos(pitch), np.sin(pitch), np.cos(yaw) * np.cos(pitch)])
    right = np.cross(fwd, [0.0, 1.0, 0.0])
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([right, down, fwd])
    T[:3, 3] = position
    return T


def walk(n_frames=180, width=4.0, length=3.0, cam_height=1.4, margin=1.0, pitch_deg=25.0, turns=1.25):
    """An ellipse inside the room, looking outwards-ish while turning, tilting up and down."""
    poses = []
    for i in range(n_frames):
        s = i / n_frames
        a = 2 * np.pi * s
        pos = [width / 2 + (width / 2 - margin) * np.cos(a), cam_height, length / 2 + (length / 2 - margin) * np.sin(a)]
        yaw = 2 * np.pi * turns * s
        pitch = np.radians(pitch_deg) * np.sin(2 * np.pi * 3 * s)
        poses.append(camera_pose(pos, yaw, pitch))
    return np.array(poses)


def depth_intrinsics():
    s = DEPTH_SIZE[0] / RGB_SIZE[0]
    K = np.array([[FX_RGB * s, 0, (RGB_SIZE[0] / 2 - 0.5 + 0.5) * s - 0.5],
                  [0, FX_RGB * s, (RGB_SIZE[1] / 2 - 0.5 + 0.5) * s - 0.5], [0, 0, 1]])
    return K


def render_box(T_c2w, width, length, height, K=None, size=DEPTH_SIZE):
    """Forward (z) depth of the box interior seen from one camera."""
    K = depth_intrinsics() if K is None else K
    W, H = size
    v, u = np.mgrid[0:H, 0:W].astype(float)
    d_cam = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u)], -1)  # z = 1
    d = d_cam @ T_c2w[:3, :3].T
    o = T_c2w[:3, 3]
    t = np.full(u.shape, np.inf)
    for axis, lo, hi in ((0, 0.0, width), (1, 0.0, height), (2, 0.0, length)):
        with np.errstate(divide='ignore', invalid='ignore'):
            for plane in (lo, hi):
                ti = (plane - o[axis]) / d[..., axis]
                t = np.where((ti > 1e-6) & (ti < t), ti, t)
    return t                                            # ray parameter = z depth because d_cam z = 1


def write_capture(root, T, depth, timestamps=None, conf=None, imu_rows_per_frame=2, fps=30.0, intrinsics=None,
                  odometry_override=None, gravity_world=(0.0, -1.0, 0.0)):
    """Write a capture folder. depth: list of (H, W) metres; gravity_world: true gravity direction in
    the pose frame (differs from -y when the world frame is tilted). Returns the capture folder."""
    root = Path(root)
    (root / 'depth').mkdir(parents=True, exist_ok=True)
    (root / 'confidence').mkdir(exist_ok=True)
    n = len(T)
    ts = np.arange(n) / fps + 1000.0 if timestamps is None else np.asarray(timestamps, float)
    for i in range(n):
        Image.fromarray(np.round(np.nan_to_num(depth[i], posinf=0) * 1000).clip(0, 65535).astype(np.uint16)
                        ).save(root / 'depth' / f'{i:06d}.png')
        c = np.full(depth[i].shape, 2, np.uint8) if conf is None else conf[i]
        Image.fromarray(c).save(root / 'confidence' / f'{i:06d}.png')
    fx, fy, cx, cy = intrinsics or (FX_RGB, FX_RGB, RGB_SIZE[0] / 2 - 0.5, RGB_SIZE[1] / 2 - 0.5)
    rows = ['timestamp, frame, x, y, z, qx, qy, qz, qw, fx, fy, cx, cy, distortion_center_x, distortion_center_y']
    for i in range(n):
        q = rot_to_quat_xyzw(T[i, :3, :3])
        vals = [ts[i], f'{i:06d}', *T[i, :3, 3], *q, fx, fy, cx, cy]
        rows.append(', '.join(str(v) for v in vals) + ', , ')
    if odometry_override:
        rows = odometry_override(rows)
    (root / 'odometry.csv').write_text('\n'.join(rows) + '\n', encoding='utf-8')
    imu = ['timestamp, a_x, a_y, a_z, alpha_x, alpha_y, alpha_z']
    for i in range(n):
        g_cam = T[i, :3, :3].T @ np.asarray(gravity_world, float)   # gravity (down) in the camera frame, in g
        for k in range(imu_rows_per_frame):
            imu.append(', '.join(str(v) for v in [ts[i] + k / (fps * imu_rows_per_frame), *g_cam, 0, 0, 0]))
    (root / 'imu.csv').write_text('\n'.join(imu) + '\n', encoding='utf-8')
    (root / 'camera_matrix.csv').write_text(f'{fx}, 0.0, {cx}\n0.0, {fy}, {cy}\n0.0, 0.0, 1.0\n', encoding='utf-8')
    w = cv2.VideoWriter(str(root / 'rgb.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), fps, RGB_SIZE)
    for i in range(min(n, 5)):
        w.write(np.full((RGB_SIZE[1], RGB_SIZE[0], 3), 100 + i, np.uint8))
    w.release()
    return root


def box_capture(root, width=4.0, length=3.0, height=2.5, **walk_kw):
    T = walk(width=width, length=length, **walk_kw)
    depth = [render_box(t, width, length, height) for t in T]
    return write_capture(root, T, depth), T, depth
