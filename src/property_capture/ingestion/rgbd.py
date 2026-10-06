"""Load an extracted RGB-D capture: depth/confidence PNGs, odometry, IMU, intrinsics.

Layout (verified by scripts/audit_datasets.py, see docs/data_dictionary.md):
  <capture_id>/depth/NNNNNN.png        uint16
  <capture_id>/confidence/NNNNNN.png   uint8
  <capture_id>/rgb.mp4
  <capture_id>/camera_matrix.csv       3x3, no header
  <capture_id>/odometry.csv            timestamp, frame, x..z, qx..qw, fx, fy, cx, cy, ...
  <capture_id>/imu.csv                 timestamp, a_x..a_z (g), alpha_x..alpha_z (rad/s)
"""
import zipfile
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
from PIL import Image

from ..geometry.transforms import scale_intrinsics

REQUIRED = ('depth', 'confidence', 'odometry.csv', 'imu.csv', 'rgb.mp4')


def _read_csv(path):
    df = pd.read_csv(path, skipinitialspace=True)
    df.columns = [c.strip() for c in df.columns]
    return df


def _check_finite(name, ids, df, columns):
    missing = [c for c in columns if c not in df.columns]
    if missing:
        raise ValueError(f'{name}: missing column(s) {missing}')
    values = df[list(columns)].apply(pd.to_numeric, errors='coerce').to_numpy(float)
    bad = ~np.isfinite(values)
    if bad.any():
        rows = np.flatnonzero(bad.any(axis=1))
        cols = [columns[j] for j in np.flatnonzero(bad[rows[0]])]
        raise ValueError(f'{name}: {len(rows)} row(s) with missing or non-numeric values, first at row '
                         f'{rows[0]} (frame/index {ids[rows[0]]}), columns {cols}')


def _index_pngs(folder):
    out = {}
    for p in folder.glob('*.png'):
        if p.stem.isdigit():
            out[int(p.stem)] = p
    return out


@dataclass
class RGBDCapture:
    capture_id: str
    root: Path
    timestamps: np.ndarray       # odometry clock, seconds (single time base, assumptions B-07a)
    frames: np.ndarray           # frame index per odometry row
    positions: np.ndarray        # (N, 3) as stored
    quaternions: np.ndarray      # (N, 4) columns as stored: qx, qy, qz, qw
    rgb_intrinsics: np.ndarray   # (N, 3, 3) per-frame (assumptions B-14)
    rgb_size: tuple              # (width, height)
    depth_size: tuple            # (width, height)
    depth_paths: dict
    confidence_paths: dict
    imu_timestamps: np.ndarray
    imu_accel: np.ndarray        # (M, 3), g
    imu_gyro: np.ndarray         # (M, 3), rad/s

    def __len__(self):
        return len(self.frames)

    @property
    def depth_to_rgb_scale(self):
        return self.depth_size[0] / self.rgb_size[0]

    def depth_intrinsics(self, i):
        # assumptions B-15: depth is a resampled, aligned view of the RGB image
        return scale_intrinsics(self.rgb_intrinsics[i], self.depth_to_rgb_scale)

    def load_depth_raw(self, i):
        return np.array(Image.open(self.depth_paths[int(self.frames[i])]))

    def load_confidence(self, i):
        return np.array(Image.open(self.confidence_paths[int(self.frames[i])]))


def depth_validity(cap, dcfg, samples=24):
    """Fraction of pixels with usable depth (confidence and range) over evenly spaced frames."""
    idx = np.linspace(0, len(cap) - 1, min(samples, len(cap))).astype(int)
    valid, raw = [], []
    for i in idx:
        d = cap.load_depth_raw(i).astype(float) * dcfg['unit_scale_m']
        ok = (cap.load_confidence(i) >= dcfg['confidence_min']) & (d > dcfg['min_m']) & (d < dcfg['max_m'])
        valid.append(ok.mean())
        nz = d[d > 0]
        raw.append(np.median(nz) if len(nz) else 0.0)
    return {'frames_sampled': len(idx), 'valid_fraction': float(np.mean(valid)),
            'median_nonzero_depth_m': float(np.median(raw))}


def resolve_capture_dir(path, raw_root):
    """Accept a capture directory, its parent, or a ZIP (extracted under raw_root/<zip stem>/)."""
    path = Path(path)
    if path.suffix.lower() == '.zip':
        with zipfile.ZipFile(path) as zf:
            tops = sorted({n.split('/')[0] for n in zf.namelist() if n.strip('/')})
            if len(tops) != 1:
                raise ValueError(f'{path.name}: expected one top-level folder, found {tops}')
            dest = Path(raw_root) / path.stem
            if not (dest / tops[0] / 'odometry.csv').exists():
                zf.extractall(dest)
        path = dest / tops[0]
    if path.is_dir() and not (path / 'odometry.csv').exists():
        subs = [d for d in path.iterdir() if d.is_dir() and (d / 'odometry.csv').exists()]
        if len(subs) == 1:
            path = subs[0]
    missing = [r for r in REQUIRED if not (path / r).exists()]
    if missing:
        raise FileNotFoundError(f'{path}: not an RGB-D capture, missing {missing}')
    return path


def load_capture(path, raw_root='data/raw'):
    root = resolve_capture_dir(path, raw_root)
    odo = _read_csv(root / 'odometry.csv')
    imu = _read_csv(root / 'imu.csv')

    _check_finite('odometry.csv', np.arange(len(odo)), odo, ('timestamp', 'frame', 'x', 'y', 'z', 'qx', 'qy', 'qz',
                                                             'qw', 'fx', 'fy', 'cx', 'cy'))
    frames = odo['frame'].to_numpy(int)
    ts = odo['timestamp'].to_numpy(float)
    if np.any(np.diff(ts) <= 0):
        raise ValueError('odometry timestamps are not strictly increasing')
    qn = np.linalg.norm(odo[['qx', 'qy', 'qz', 'qw']].to_numpy(float), axis=1)
    bad = np.flatnonzero(np.abs(qn - 1) > 0.1)
    if len(bad):
        raise ValueError(f'odometry.csv: {len(bad)} quaternion(s) far from unit length (norm {qn[bad[0]]:.3g} '
                         f'at frame {frames[bad[0]]}); poses are unusable')
    _check_finite('imu.csv', np.arange(len(imu)), imu, ('timestamp', 'a_x', 'a_y', 'a_z',
                                                        'alpha_x', 'alpha_y', 'alpha_z'))
    g = np.median(np.linalg.norm(imu[['a_x', 'a_y', 'a_z']].to_numpy(float), axis=1))
    if not 0.5 < g < 2.0:
        raise ValueError(f'imu.csv: median acceleration {g:.3g}, expected about 1 (units of g, '
                         'assumptions.md B-09); m/s2 would give about 9.8')

    depth_paths = _index_pngs(root / 'depth')
    conf_paths = _index_pngs(root / 'confidence')
    missing = [f for f in frames if f not in depth_paths or f not in conf_paths]
    if missing:
        raise ValueError(f'{len(missing)} odometry frames lack depth/confidence PNGs, e.g. {missing[:5]}')

    K = np.zeros((len(odo), 3, 3))
    K[:, 0, 0] = odo['fx']
    K[:, 1, 1] = odo['fy']
    K[:, 0, 2] = odo['cx']
    K[:, 1, 2] = odo['cy']
    K[:, 2, 2] = 1.0

    cap = cv2.VideoCapture(str(root / 'rgb.mp4'))
    rgb_size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    cap.release()
    if rgb_size[0] <= 0:
        raise ValueError('could not read RGB resolution from rgb.mp4')
    d0 = np.array(Image.open(depth_paths[int(frames[0])]))
    depth_size = (d0.shape[1], d0.shape[0])
    if not np.isclose(depth_size[0] / rgb_size[0], depth_size[1] / rgb_size[1]):
        raise ValueError(f'depth {depth_size} and RGB {rgb_size} aspect ratios differ; intrinsics scaling invalid')
    bad = np.flatnonzero((K[:, 0, 0] <= 0) | (K[:, 1, 1] <= 0) | (K[:, 0, 2] <= 0) | (K[:, 0, 2] >= rgb_size[0])
                         | (K[:, 1, 2] <= 0) | (K[:, 1, 2] >= rgb_size[1]))
    if len(bad):
        i = bad[0]
        raise ValueError(f'odometry.csv: {len(bad)} frame(s) with invalid intrinsics, e.g. frame {frames[i]}: '
                         f'fx {K[i, 0, 0]:.4g}, fy {K[i, 1, 1]:.4g}, cx {K[i, 0, 2]:.4g}, cy {K[i, 1, 2]:.4g} '
                         f'for a {rgb_size[0]}x{rgb_size[1]} image (focal lengths must be positive, principal '
                         'point inside the image)')

    return RGBDCapture(
        capture_id=root.name, root=root, timestamps=ts, frames=frames,
        positions=odo[['x', 'y', 'z']].to_numpy(float),
        quaternions=odo[['qx', 'qy', 'qz', 'qw']].to_numpy(float),
        rgb_intrinsics=K, rgb_size=rgb_size, depth_size=depth_size,
        depth_paths=depth_paths, confidence_paths=conf_paths,
        imu_timestamps=imu['timestamp'].to_numpy(float),
        imu_accel=imu[['a_x', 'a_y', 'a_z']].to_numpy(float),
        imu_gyro=imu[['alpha_x', 'alpha_y', 'alpha_z']].to_numpy(float),
    )
