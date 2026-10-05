"""Voxel-averaged point cloud from confidence-masked depth frames and poses (used as-is)."""
import numpy as np

from ..geometry.transforms import backproject, transform

_OFF = 1 << 20


def _pack(keys):
    k = keys + _OFF
    return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]


def fuse(cap, T_c2w, axes, frame_ids, dcfg, fcfg):
    """Back-project selected frames into world and average per voxel.

    Returns dict(points (M, 3), counts (M,), stats).
    """
    step = fcfg['pixel_step']
    H, W = cap.depth_size[1], cap.depth_size[0]
    v, u = np.mgrid[0:H:step, 0:W:step]
    u, v = u.ravel().astype(float), v.ravel().astype(float)
    ui, vi = u.astype(int), v.astype(int)

    chunks = []
    n_px = n_conf = n_range = 0
    for i in frame_ids:
        d = cap.load_depth_raw(i)[vi, ui] * dcfg['unit_scale_m']
        c = cap.load_confidence(i)[vi, ui]
        conf_ok = c >= dcfg['confidence_min']
        range_ok = (d > dcfg['min_m']) & (d < dcfg['max_m'])
        ok = conf_ok & range_ok
        n_px += d.size
        n_conf += int((~conf_ok).sum())
        n_range += int((conf_ok & ~range_ok).sum())
        P = backproject(u[ok], v[ok], d[ok], cap.depth_intrinsics(i), axes)
        chunks.append(transform(T_c2w[i], P).astype(np.float32))

    P = np.concatenate(chunks)
    keys = _pack(np.floor(P / fcfg['voxel_size_m']).astype(np.int64))
    uniq, inv, counts = np.unique(keys, return_inverse=True, return_counts=True)
    sums = np.zeros((len(uniq), 3))
    np.add.at(sums, inv, P)
    keep = counts >= fcfg['min_points_per_voxel']
    pts = sums[keep] / counts[keep, None]
    return {
        'points': pts,
        'counts': counts[keep],
        'stats': {
            'frames_used': len(frame_ids),
            'pixels_sampled': n_px,
            'rejected_low_confidence_fraction': n_conf / n_px,
            'rejected_out_of_range_fraction': n_range / n_px,
            'raw_points': int(len(P)),
            'voxels_total': int(len(uniq)),
            'voxels_kept': int(keep.sum()),
        },
    }
