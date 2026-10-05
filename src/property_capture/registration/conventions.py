"""Resolve pose conventions and depth scale from multi-view depth consistency.

Each hypothesis (quaternion order x pose direction x camera axes) is scored
by back-projecting depth from frame i, moving it into frame j with the
hypothesised poses, and comparing against frame j's own depth. The correct
convention gives small residuals; wrong ones give large residuals or no
overlap. Addresses assumptions.md B-11 and B-12; the depth-scale sweep
addresses B-03 relative to the odometry's metric scale.
"""
import numpy as np

from ..geometry.transforms import (CAMERA_AXES, QUATERNION_ORDERS, backproject, invert,
                                   pose_matrices, project, transform)

POSE_DIRECTIONS = ('camera_to_world', 'world_to_camera')


def camera_to_world(cap, quaternion_order, pose_direction):
    T = pose_matrices(cap.positions, cap.quaternions, quaternion_order)
    return T if pose_direction == 'camera_to_world' else invert(T)


def _pair_residual(dA, cA, KA, TA, dB, KB, TB, axes, conf_min, step, dmin, dmax):
    H, W = dA.shape
    v, u = np.mgrid[0:H:step, 0:W:step]
    d = dA[v, u]
    ok = (cA[v, u] >= conf_min) & (d > dmin) & (d < dmax)
    if ok.sum() < 50:
        return None
    P = transform(invert(TB) @ TA, backproject(u[ok].astype(float), v[ok].astype(float), d[ok], KA, axes))
    ub, vb, db = project(P, KB, axes)
    ui, vi = np.rint(np.nan_to_num(ub, nan=-1)).astype(int), np.rint(np.nan_to_num(vb, nan=-1)).astype(int)
    inb = (db > dmin) & (ui >= 0) & (ui < W) & (vi >= 0) & (vi < H)
    if inb.sum() < 50:
        return {'overlap_fraction': float(inb.mean()), 'median_abs_residual_m': None, 'within_5cm': 0.0}
    r = np.abs(db[inb] - dB[vi[inb], ui[inb]])
    return {'overlap_fraction': float(inb.mean()), 'median_abs_residual_m': float(np.median(r)),
            'within_5cm': float(np.mean(r < 0.05))}


def _score(cap, T, axes, pairs, depth, conf, cfg, dcfg, scale=1.0):
    per_pair = []
    for i, j in pairs:
        per_pair.append(_pair_residual(depth[i] * scale, conf[i], cap.depth_intrinsics(i), T[i],
                                       depth[j] * scale, cap.depth_intrinsics(j), T[j], axes,
                                       dcfg['confidence_min'], cfg['pixel_step'], dcfg['min_m'], dcfg['max_m']))
    meds = [p['median_abs_residual_m'] for p in per_pair if p and p['median_abs_residual_m'] is not None]
    return {
        'pairs': len(pairs),
        'pairs_with_overlap': len(meds),
        'median_abs_residual_m': float(np.median(meds)) if meds else None,
        'mean_within_5cm': float(np.mean([p['within_5cm'] for p in per_pair if p])) if per_pair else 0.0,
        'mean_overlap_fraction': float(np.mean([p['overlap_fraction'] for p in per_pair if p])) if per_pair else 0.0,
    }


def resolve_conventions(cap, cfg, dcfg):
    n = len(cap)
    gap = cfg['pair_gap_frames']
    starts = np.linspace(0, n - 1 - gap, cfg['num_pairs']).astype(int)
    pairs = [(int(i), int(i + gap)) for i in starts]
    frames = sorted({i for p in pairs for i in p})
    depth = {i: cap.load_depth_raw(i).astype(float) * dcfg['unit_scale_m'] for i in frames}
    conf = {i: cap.load_confidence(i) for i in frames}

    table = []
    for q in QUATERNION_ORDERS:
        for d in POSE_DIRECTIONS:
            T = camera_to_world(cap, q, d)
            for a in CAMERA_AXES:
                s = _score(cap, T, a, pairs, depth, conf, cfg, dcfg)
                table.append({'quaternion_order': q, 'pose_direction': d, 'camera_axes': a, **s})

    def key(r):
        ok = r['median_abs_residual_m'] is not None and r['pairs_with_overlap'] >= len(pairs) // 2
        return r['median_abs_residual_m'] if ok else np.inf

    ranked = sorted(table, key=key)
    best, second = ranked[0], ranked[1]
    if not np.isfinite(key(best)):
        raise RuntimeError('no pose convention produced overlapping, consistent depth; cannot continue')
    T_best = camera_to_world(cap, best['quaternion_order'], best['pose_direction'])

    sweep = []
    for s in cfg['depth_scale_grid']:
        r = _score(cap, T_best, best['camera_axes'], pairs, depth, conf, cfg, dcfg, scale=s)
        sweep.append({'scale': s, 'median_abs_residual_m': r['median_abs_residual_m'],
                      'mean_within_5cm': r['mean_within_5cm']})
    valid = [r for r in sweep if r['median_abs_residual_m'] is not None]
    best_scale = min(valid, key=lambda r: r['median_abs_residual_m'])['scale'] if valid else None

    return {
        'method': 'multi-view depth reprojection residual over frame pairs',
        'pairs': pairs,
        'hypotheses': table,
        'selected': {k: best[k] for k in ('quaternion_order', 'pose_direction', 'camera_axes')},
        'selected_median_abs_residual_m': best['median_abs_residual_m'],
        'runner_up': {k: second[k] for k in ('quaternion_order', 'pose_direction', 'camera_axes')},
        'runner_up_median_abs_residual_m': second['median_abs_residual_m'],
        'depth_scale_sweep': sweep,
        'depth_scale_best': best_scale,
        'depth_unit_scale_m': dcfg['unit_scale_m'],
    }, T_best
