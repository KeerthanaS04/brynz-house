"""Evaluation only: compare a video-tier trajectory with the device poses of the same capture.

This module reads device odometry, which the video tier itself must never see
(CLAUDE_updated.md rule 6). It is not imported by any production code path.

The video trajectory is in arbitrary SfM units and orientation, so it is aligned to the
device camera centres with the least-squares similarity (Umeyama). The residual after
alignment (absolute trajectory error, ATE) measures the video tier's camera motion
accuracy; the fitted scale is the factor a metric reference would have to supply.
"""
import json
from pathlib import Path

import numpy as np

from ..ingestion.rgbd import load_capture
from ..ingestion.video import align_video_to_odometry


def umeyama(src, dst, with_scale=True):
    """Similarity (s, R, t) minimizing ||dst - (s R src + t)||^2. src, dst: (N, 3)."""
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    xs, xd = src - mu_s, dst - mu_d
    U, S, Vt = np.linalg.svd(xd.T @ xs / len(src))
    D = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[2, 2] = -1
    R = U @ D @ Vt
    s = float(np.trace(np.diag(S) @ D) / (xs ** 2).sum(axis=1).mean()) if with_scale else 1.0
    return s, R, mu_d - s * R @ mu_s


def _align(frames, cap, k):
    keep = [f for f in frames if 0 <= f['video_index'] + k < len(cap)]
    if len(keep) < 3:
        return None
    dev = cap.positions[[f['video_index'] + k for f in keep]]
    sfm = np.array([f['centre'] for f in keep])
    s, R, t = umeyama(sfm, dev)
    err = np.linalg.norm((s * (R @ sfm.T)).T + t - dev, axis=1)
    path_len = float(np.linalg.norm(np.diff(dev, axis=0), axis=1).sum())
    return {'frames_compared': len(keep), 'first_video_index': keep[0]['video_index'],
            'last_video_index': keep[-1]['video_index'], 'device_path_length_m': path_len,
            'sfm_to_metric_scale': s, 'ate_rmse_m': float(np.sqrt(np.mean(err ** 2))),
            'ate_median_m': float(np.median(err)), 'ate_max_m': float(err.max()),
            'ate_rmse_percent_of_path': float(100 * np.sqrt(np.mean(err ** 2)) / max(path_len, 1e-9))}


def compare(video_run, capture_path, raw_root='data/raw'):
    video_run = Path(video_run)
    traj = json.loads((video_run / 'trajectory.json').read_text(encoding='utf-8'))
    cap = load_capture(capture_path, raw_root)
    pairing = align_video_to_odometry(traj['all_frame_pts_s'], cap.timestamps)
    if not pairing or not (pairing['best_is_exact'] and pairing['best_is_unique']):
        return {'status': 'not_evaluable', 'reason': 'video-to-device frame pairing not exact', 'pairing': pairing}
    k = pairing['best_offset']
    largest = _align(traj['frames'], cap, k)
    if largest is None:
        return {'status': 'not_evaluable', 'reason': 'fewer than 3 registered frames'}
    pieces = [p for p in (_align(f, cap, k) for f in traj.get('pieces', [])) if p is not None]
    scales = [p['sfm_to_metric_scale'] for p in pieces]
    return {
        'status': 'evaluated', 'pairing': pairing['mapping'], **largest,
        'pieces': pieces,
        'piece_scale_ratio_max_min': float(max(scales) / min(scales)) if scales else None,
        'note': 'device poses are a consistency reference (B-23: 1-3 cm), not ground truth; '
                'the similarity alignment uses the device poses and is evaluation-only. Each piece is aligned '
                'separately: pieces have independent scales that video alone cannot relate.',
    }
