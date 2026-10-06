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
from PIL import Image

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


def _plan_totals(run_dir):
    prop = json.loads((Path(run_dir) / 'property.json').read_text(encoding='utf-8'))
    ms = {m['id']: m for m in prop['measurements']}
    rooms = []
    for r in prop['rooms']:
        c = r['ceiling']
        rooms.append({'room_id': r['room_id'], 'area_m2': ms[r['floor_area_measurement_id']]['value'],
                      'walls': len(r['walls']),
                      'ceiling_height_m': ms[c['measurement_id']]['value'] if c['status'] == 'measured' else None})
    walls = sorted((m['value'] for m in prop['measurements'] if m['quantity'] == 'wall_length'), reverse=True)
    return {'rooms': len(rooms), 'total_area_m2': sum(r['area_m2'] for r in rooms),
            'room_areas_m2': sorted((round(r['area_m2'], 2) for r in rooms), reverse=True),
            'ceiling_heights_m': sorted(round(r['ceiling_height_m'], 3) for r in rooms if r['ceiling_height_m']),
            'longest_walls_m': [round(w, 3) for w in walls[:8]], 'rooms_detail': rooms}


def compare_plan(plan_run, capture_path, raw_root='data/raw', lidar_run=None, depth_frames=40):
    """Video plan run vs the same capture's device data: trajectory (device poses), network depth
    (LiDAR depth at the paired frames) and, optionally, the LiDAR-tier floor plan."""
    plan_run = Path(plan_run)
    vt = json.loads((plan_run / 'intermediates' / 'video_trajectory.json').read_text(encoding='utf-8'))
    step1 = Path(json.loads((plan_run / 'run_info.json').read_text(encoding='utf-8'))['input'])
    traj1 = json.loads((step1 / 'trajectory.json').read_text(encoding='utf-8'))
    cap = load_capture(capture_path, raw_root)
    pairing = align_video_to_odometry(traj1['all_frame_pts_s'], cap.timestamps)
    if not pairing or not (pairing['best_is_exact'] and pairing['best_is_unique']):
        return {'status': 'not_evaluable', 'reason': 'video-to-device frame pairing not exact', 'pairing': pairing}
    k = pairing['best_offset']
    result = {'status': 'evaluated', 'pairing': pairing['mapping'], 'components': {}}

    by_comp = {}
    for f in vt['frames']:
        by_comp.setdefault(f['component'], []).append(f)
    for comp, frames in sorted(by_comp.items()):
        keep = [f for f in frames if 0 <= f['video_index'] + k < len(cap)]
        if len(keep) < 3:
            continue
        vid = np.array([np.asarray(f['c2w'])[:3, 3] for f in keep])
        dev = cap.positions[[f['video_index'] + k for f in keep]]
        path_len = float(np.linalg.norm(np.diff(dev, axis=0), axis=1).sum())
        entry = {'frames': len(keep), 'main': comp == vt['main_component'], 'device_path_length_m': path_len,
                 'video_path_length_m': float(np.linalg.norm(np.diff(vid, axis=0), axis=1).sum())}
        for name, with_scale in (('rigid', False), ('similarity', True)):
            s, R, t = umeyama(vid, dev, with_scale=with_scale)
            err = np.linalg.norm((s * (R @ vid.T)).T + t - dev, axis=1)
            rmse = float(np.sqrt(np.mean(err ** 2)))
            entry[name] = {'ate_rmse_m': rmse, 'ate_median_m': float(np.median(err)), 'ate_max_m': float(err.max()),
                           'ate_rmse_percent_of_path': 100 * rmse / max(path_len, 1e-9)}
            if with_scale:
                entry[name]['video_to_device_scale'] = s
                entry[name]['scale_error_percent'] = 100 * (1 / s - 1)
        result['components'][str(comp)] = entry

    # network depth vs LiDAR depth (both 256 x 192 here) at evenly spaced paired frames
    for name, folder in (('network_depth_vs_lidar', 'depth'), ('corrected_depth_vs_lidar', 'depth_corrected')):
        files = sorted((plan_run / folder).glob('*.png'))
        ratios, absrel = [], []
        for p in files[::max(1, len(files) // depth_frames)]:
            j = int(p.stem) + k
            if not 0 <= j < len(cap):
                continue
            pred = np.array(Image.open(p)).astype(float) / 1000.0
            lidar = cap.load_depth_raw(j).astype(float) / 1000.0
            ok = (cap.load_confidence(j) >= 2) & (lidar > 0.2) & (lidar < 5.0) & (pred > 0)
            if pred.shape != lidar.shape or ok.sum() < 500:
                continue
            r = pred[ok] / lidar[ok]
            ratios.append(float(np.median(r)))
            absrel.append(float(np.median(np.abs(r - 1))))
        if ratios:
            result[name] = {
                'frames': len(ratios), 'median_ratio': float(np.median(ratios)),
                'ratio_p10_p90': [float(x) for x in np.percentile(ratios, [10, 90])],
                'median_abs_rel_error': float(np.median(absrel)),
                'note': 'ratio > 1: video depth longer than LiDAR depth; per frame, high-confidence LiDAR pixels'}
    if lidar_run:
        v, l = _plan_totals(plan_run), _plan_totals(lidar_run)
        result['plan_vs_lidar_plan'] = {
            'video': v, 'lidar': l,
            'total_area_diff_percent': 100 * (v['total_area_m2'] - l['total_area_m2']) / l['total_area_m2'],
            'note': 'LiDAR-tier plan is a consistency reference, not ground truth'}
    result['note'] = ('evaluation only: reads device poses and LiDAR depth, which the video tier never sees. '
                      'rigid = alignment without scale (tests the metric scale); similarity = with scale.')
    return result


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
