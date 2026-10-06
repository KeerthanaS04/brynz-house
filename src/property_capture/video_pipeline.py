"""Video tier: floor plan from a step-1 video run (selected frames + SfM pieces) and network depth.

Stages: depth per frame -> metric scale per SfM run -> runs checked against depth links ->
one trajectory -> up direction -> shared stages (plan_outputs) -> property.json, input_tier 'video'.
Reads only the step-1 run folder, the video it names (for its hash) and the model weights
(CLAUDE_updated.md rule 6; tests/unit/test_video_depth.py).
"""
import copy
import datetime
import json
import logging
import platform
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from .evaluation.gates import gate_results
from .geometry.planes import ransac_plane
from .geometry.transforms import invert, scale_intrinsics
from .modalities.video_depth import (DepthModel, FrameMatcher, VideoFrames, _rot_deg, _scaled, chain_trajectory,
                                     colmap_intrinsics, depth_confidence, estimate_up, extract_bridge_frames,
                                     load_pieces, scale_samples, split_runs)
from .plan_outputs import (Timer, apply_overrides, build_geometry, dump, setup_logging, split_half_consistency,
                           write_outputs)
from .reconstruction.fusion import fuse
from .reporting import provenance

REPO_ROOT = Path(__file__).resolve().parents[2]
log = logging.getLogger('property_capture')


def _provenance(cfg, video_run, video_path, weights_dir):
    import pycolmap
    import torch
    import transformers
    return {
        'command': ' '.join([sys.executable] + sys.argv),
        'python': sys.version,
        'platform': platform.platform(),
        'packages': {'numpy': np.__version__, 'opencv': cv2.__version__, 'pycolmap': pycolmap.__version__,
                     'torch': torch.__version__, 'transformers': transformers.__version__},
        'git': provenance.git_info(REPO_ROOT),
        'config_sha256': provenance.config_hash(cfg),
        'random_seed': cfg['seed'],
        'inputs': {'video_run': str(video_run), 'video': str(video_path),
                   'video_sha256': provenance.sha256_file(video_path) if video_path.exists() else None,
                   'weights_dir': str(weights_dir), 'weights_tree_sha256': provenance.sha256_tree(weights_dir)},
    }


def _refine_up(vf, T, up0, cfg, vcfg, rng):
    """Floor plane normal near the camera-axis estimate of up (two passes, the second tighter)."""
    q = fuse(vf, T, 'opencv', list(range(0, len(vf), vcfg['up_frame_stride'])), cfg['depth'],
             {'pixel_step': 4, 'voxel_size_m': 0.05, 'min_points_per_voxel': 1})
    P, up, info = q['points'], up0, []
    for max_angle in (vcfg['up_max_angle_deg'], vcfg['up_refine_angle_deg']):
        h, cam_h = P @ up, T[:, :3, 3] @ up
        below = P[h < np.percentile(cam_h, 10) - cfg['planes']['floor_below_camera_m']]
        pl = ransac_plane(below, vcfg['up_ransac_m'], 500, rng, normal_hint=up, max_angle_deg=max_angle)
        if pl is None:
            info.append({'max_angle_deg': max_angle, 'status': 'no floor plane'})
            break
        info.append({'max_angle_deg': max_angle, 'inliers': int(pl['inliers'].sum()), 'points_below': len(below),
                     'change_deg': float(np.degrees(np.arccos(np.clip(pl['normal'] @ up, -1, 1))))})
        up = pl['normal']
    return up, info


def run_video_tier(video_path, output_dir, cfg, gates_path):
    """One command for a video: step 1 (frames + SfM) into <output_dir>/video_sfm, then the plan
    into <output_dir>, laid out like a LiDAR run (property.json, floorplan.png, ...)."""
    from .modalities.video_sfm import run_video
    out = Path(output_dir)
    if out.exists() and any(out.iterdir()):
        raise FileExistsError(f'{out} exists and is not empty; runs never overwrite')
    run_video(video_path, out / 'video_sfm', cfg['video'])
    return run_video_plan(out / 'video_sfm', out, cfg, gates_path)


def run_video_plan(video_run, output_dir, cfg, gates_path):
    t_start = time.time()
    video_run, out = Path(video_run), Path(output_dir)
    # the step-1 run may live inside the output folder (run_video_tier); nothing else may
    if out.exists() and any(p.resolve() != video_run.resolve() for p in out.iterdir()):
        raise FileExistsError(f'{out} exists and is not empty; runs never overwrite')
    (out / 'intermediates').mkdir(parents=True, exist_ok=True)
    setup_logging(out)
    stage = Timer()
    vcfg = cfg['video_depth']
    cfg = apply_overrides(copy.deepcopy(cfg), [f'{k}={v}' for k, v in vcfg['shared_overrides'].items()])
    rng = np.random.default_rng(cfg['seed'])

    sfm_info = json.loads((video_run / 'video_sfm.json').read_text(encoding='utf-8'))
    traj = json.loads((video_run / 'trajectory.json').read_text(encoding='utf-8'))
    video_path = Path(sfm_info['inputs_read'][0])
    capture_id = video_path.parent.name if video_path.name == 'rgb.mp4' else video_path.stem
    selected = [k for k in traj['selected'] if k['kept']]
    seq = [k['video_index'] for k in selected]
    pts = {k['video_index']: k['pts_s'] for k in selected}
    order = {idx: n for n, idx in enumerate(seq)}
    frames_dir = video_run / 'frames'
    h0, w0 = cv2.imread(str(frames_dir / f'{seq[0]:06d}.jpg')).shape[:2]
    image_size = (w0, h0)

    pieces = load_pieces(video_run / 'colmap' / 'sparse')
    if not pieces:
        raise RuntimeError(f'{video_run}: no SfM models')
    largest = max(pieces, key=lambda p: len(p['poses']))
    K_img = colmap_intrinsics(next(iter(largest['rec'].cameras.values())))
    focals = [float(colmap_intrinsics(next(iter(p['rec'].cameras.values())))[0, 0]) for p in pieces]
    s_d = vcfg['depth_width'] / image_size[0]
    depth_size = (vcfg['depth_width'], int(round(image_size[1] * s_d)))
    K_depth = scale_intrinsics(K_img, s_d)
    log.info('video run %s: %d frames, %d SfM pieces; focal %.1f px at %dx%d (pieces %.1f-%.1f)', video_run,
             len(seq), len(pieces), K_img[0, 0], *image_size, min(focals), max(focals))

    t0 = time.time()
    paths = {i: frames_dir / f'{i:06d}.jpg' for i in seq}
    bridge = {}
    if vcfg['bridge_max_gap_frames'] is not None:
        bridge = extract_bridge_frames(video_path, seq, out / 'bridge_frames', image_size,
                                       vcfg['bridge_max_gap_frames'], vcfg['bridge_step_frames'])
    paths.update(bridge)
    for i in bridge:
        pts[i] = traj['all_frame_pts_s'][i]
    selected_seq, seq = seq, sorted(paths)
    log.info('bridge frames: %d added (gap threshold %s video frames)', len(bridge), vcfg['bridge_max_gap_frames'])
    stage('bridge_frames', t0)

    t0 = time.time()
    weights_dir = REPO_ROOT / 'weights' / vcfg['model_dir']
    model = DepthModel(weights_dir, vcfg['device'])
    (out / 'depth').mkdir()
    depth_of, conf_of = {}, {}
    for idx in seq:
        d = model.predict(cv2.imread(str(paths[idx])), depth_size)
        depth_of[idx], conf_of[idx] = d, depth_confidence(d, vcfg['edge_max_rel_jump'])
        cv2.imwrite(str(out / 'depth' / f'{idx:06d}.png'), np.round(d * 1000).clip(0, 65535).astype(np.uint16))
    del model
    log.info('depth: %d frames on %s, median depth %.2f m', len(seq), vcfg['device'],
             float(np.median([np.median(d) for d in depth_of.values()])))
    stage('depth', t0)

    t0 = time.time()
    matcher = FrameMatcher(paths, K_img, image_size, vcfg)
    links = {}

    def one_way(a, b):
        if (a, b) not in links:
            links[(a, b)] = matcher.relative(a, b, depth_of[a], conf_of[a], depth_of[b], conf_of[b])
        return links[(a, b)][0]

    def relative(a, b):
        """b_from_a in units of a's network depth, and b's depth scale relative to a."""
        rel = one_way(a, b)
        if rel is None and vcfg['link_reverse']:
            back = one_way(b, a)                      # a_from_b in units of b's depth; a's scale relative to b
            if back is not None:
                T_ab, s = back
                rel = (_scaled(invert(T_ab), 1.0 / s), 1.0 / s)
        return rel

    runs = []
    for p in pieces:
        ratios = scale_samples(p['rec'], depth_of, conf_of, K_depth, s_d, vcfg['scale_min_points'])
        for pos in split_runs(p['poses'], order, vcfg['run_max_gap_frames']):
            frames = [selected_seq[q] for q in pos]
            r = {'run_id': f"m{p['model']}r{len(runs)}", 'model': p['model'], 'frames': frames,
                 'first_video_index': frames[0], 'last_video_index': frames[-1],
                 'poses': {i: p['poses'][i] for i in frames}, 'ratio': {i: ratios[i] for i in frames if i in ratios}}
            runs.append(r)
            vals = list(r['ratio'].values())
            if len(frames) < vcfg['run_min_frames'] or len(vals) < vcfg['run_min_frames']:
                r.update(accepted=False, reason='too few frames with depth-scale samples')
                continue
            q25, q50, q75 = np.percentile(vals, [25, 50, 75])
            r.update(ratio_median=float(q50), ratio_iqr_rel=float((q75 - q25) / q50))
            dt, dr = [], []
            for a, b in zip(frames, frames[1:]):
                L = relative(a, b)
                if L is not None:
                    # both in units of frame a's network depth
                    s_a = r['ratio'].get(a, q50)
                    S = invert(r['poses'][b]) @ r['poses'][a]
                    dt.append(float(np.linalg.norm(s_a * S[:3, 3] - L[0][:3, 3])))
                    dr.append(_rot_deg(S[:3, :3] @ L[0][:3, :3].T))
            r.update(link_checks=len(dt), link_translation_diff_m=float(np.median(dt)) if dt else None,
                     link_rotation_diff_deg=float(np.median(dr)) if dr else None)
            if not dt:
                r.update(accepted=False, reason='no depth links inside the run to check it against')
            elif np.median(dt) > vcfg['run_check_max_m'] or np.median(dr) > vcfg['run_check_max_deg']:
                r.update(accepted=False, reason='SfM motion disagrees with depth links')
            else:
                r.update(accepted=True, reason='SfM motion agrees with depth links')
    log.info('runs: %d of %d accepted (%d of %d frames)', sum(r['accepted'] for r in runs), len(runs),
             sum(len(r['frames']) for r in runs if r['accepted']), len(selected_seq))

    poses, corr, rows, scale_solve = chain_trajectory(seq, runs, relative, vcfg)
    log.info('scale solve per component: %s', scale_solve)
    comps = {}
    for row in rows:
        comps[row['component']] = comps.get(row['component'], 0) + 1
    main = max(comps, key=comps.get)
    idxs = [row['video_index'] for row in rows if row['component'] == main]
    # One unit of length for the whole component; metres = the network's scale on the median frame.
    k_main = np.array([corr[i] for i in idxs])
    g = float(1.0 / np.median(k_main))
    for i in idxs:
        poses[i], corr[i] = _scaled(poses[i], g), corr[i] * g
    for row in rows:
        row['depth_correction'] = corr[row['video_index']]
    corrected = {i: corr[i] * depth_of[i] for i in idxs}
    T = np.stack([poses[i] for i in idxs])
    log.info('depth correction over main component: p10/50/90 %s (network scale varies frame to frame)',
             np.round(np.percentile(k_main * g, [10, 50, 90]), 3))
    sources = {}
    for row in rows:
        if row['component'] == main:
            sources[row['source']] = sources.get(row['source'], 0) + 1
    path_len = float(np.linalg.norm(np.diff(T[:, :3, 3], axis=0), axis=1).sum())
    log.info('trajectory: %d components %s; main has %d of %d frames (%s), path %.1f m', len(comps), comps,
             len(idxs), len(seq), sources, path_len)
    stage('trajectory', t0)

    t0 = time.time()
    # geometry from the sharp (selected) frames only; bridge frames serve the trajectory
    geo_idxs = [i for i in idxs if i not in bridge]
    T = np.stack([poses[i] for i in geo_idxs])
    vf = VideoFrames(capture_id, geo_idxs, [pts[i] for i in geo_idxs], corrected, conf_of, K_depth, depth_size)
    up0, up_axes = estimate_up([t[:3, :3] for t in T])
    up, up_refine = _refine_up(vf, T, up0, cfg, vcfg, rng)
    log.info('up: camera x axes (spread %.2f), floor refinement %s', up_axes['x_axis_spread'], up_refine)
    stage('up_direction', t0)

    warnings, unobservable = [], []
    warnings.append({'code': 'VIDEO_SCALE_FROM_NETWORK',
                     'message': f"metric scale comes from a pretrained depth network ({vcfg['model_dir']}), "
                                'not from a measurement; every length is uncalibrated (assumptions.md B-24)'})
    if len(idxs) < len(seq):
        warnings.append({'code': 'VIDEO_PARTIAL_COVERAGE',
                         'message': f'the plan covers only the largest connected part of the walk: '
                                    f'{len(seq) - len(idxs)} of {len(seq)} frames are in {len(comps) - 1} other '
                                    'part(s) that could not be joined (fast turns on the spot); areas seen only '
                                    'there are missing'})
    warnings.append({'code': 'VIDEO_NO_LOOP_CLOSURE',
                     'message': 'video trajectory is chained frame to frame and run to run; drift is not corrected'})

    geo = build_geometry(vf, T, 'opencv', up, cfg, out, rng, warnings, stage)
    consistency = split_half_consistency(vf, T, 'opencv', geo['frame_ids'], geo['plan'], cfg, rng, stage)

    gates = gate_results(gates_path, references_available=False, drift_correction=False)
    gates['gate_drift_accountability'] = {
        'status': 'incomplete',
        'reason': 'video tier: no device poses are used; the trajectory is chained from the video without loop '
                  'closure, so there is no corrected trajectory or on/off ablation yet'}
    prov = _provenance(cfg, video_run, video_path, weights_dir)
    runs_out = [{k: v for k, v in r.items() if k not in ('poses', 'ratio', 'anchor')} for r in runs]
    (out / 'depth_corrected').mkdir()
    for n in range(len(vf)):
        cv2.imwrite(str(out / 'depth_corrected' / f'{vf.frames[n]:06d}.png'), vf.load_depth_raw(n))
    video_metrics = {
        'step1_run': str(video_run), 'frames_selected': len(selected_seq),
        'bridge_frames': len(bridge), 'frames_for_geometry': len(geo_idxs), 'sfm_pieces': len(pieces),
        'intrinsics': {'focal_px': float(K_img[0, 0]), 'image_size': image_size,
                       'focal_px_per_piece': focals, 'source': 'COLMAP self-calibration, largest piece'},
        'depth_network': {'model_dir': vcfg['model_dir'], 'depth_size': depth_size},
        'runs': {'total': len(runs), 'accepted': sum(r['accepted'] for r in runs),
                 'frames_in_accepted': sum(len(r['frames']) for r in runs if r['accepted']),
                 'scales': [r.get('scale') for r in runs], 'details': runs_out},
        'links': {'attempted': len(links), 'succeeded': sum(v[0] is not None for v in links.values())},
        'depth_correction': {
            'method': 'network depth x per-frame correction, solved per component in log scale from link and '
                      'SfM-run constraints with the network scale as a loose prior; metres = network scale on '
                      'the median frame of the main component',
            'global_factor': g, 'p10_p50_p90': np.percentile(k_main * g, [10, 50, 90]).tolist(),
            'solve': {str(c): v for c, v in scale_solve.items()}},
        'trajectory': {'components': {str(k): v for k, v in comps.items()}, 'main_component_frames': len(idxs),
                       'placed_fraction': len(idxs) / len(seq), 'pose_sources': sources, 'path_length_m': path_len},
        'up': {'from_camera_axes': up0, **up_axes, 'floor_refinement': up_refine, 'final': up},
    }
    dump(out / 'intermediates' / 'video_trajectory.json', {
        'frames': [{**row, 'pts_s': pts[row['video_index']], 'c2w': poses[row['video_index']]} for row in rows],
        'main_component': main, 'runs': runs_out,
        'links': [{'a': a, 'b': b, **st, 'ok': T_ is not None} for (a, b), (T_, st) in links.items()]})
    write_outputs(
        out, vf, geo, consistency, cfg, prov, warnings, unobservable, gates, input_tier='video',
        frame_description='2D floor frame: origin/axes below are in the video trajectory frame '
                          '(metres from the depth network, uncalibrated)',
        pose_label='video only: SfM runs + depth links',
        metrics_head={'video': video_metrics},
        measurement_flags=['scale_from_depth_network'] + (['video_partial_coverage'] if len(idxs) < len(seq) else []),
        calibration_status='no reference measurements (assumptions.md B-21); scale from a pretrained depth '
                           'network (assumptions.md B-24)')

    stage.timings['total'] = round(time.time() - t_start, 2)
    dump(out / 'run_info.json', {
        'run_id': out.name, 'finished_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'input': str(video_run), 'config': cfg, 'gates_path': str(gates_path), 'timings_s': stage.timings, **prov})
    log.info('video plan complete in %.1f s -> %s', stage.timings['total'], out)
    return video_metrics
