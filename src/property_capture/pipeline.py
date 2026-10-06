"""LiDAR tier: one RGB-D capture in, one run directory out.

Stages: load -> resolve pose conventions -> gravity -> RGB alignment -> drift correction
(validated on held-out revisits, applied only if it helps) -> fusion, floor plan, openings
(plan_outputs) -> drift on/off ablation -> half-split consistency -> property.json, metrics,
gates, render (plan_outputs). Every measurement is uncalibrated because the supplied captures
have no reference measurements.
"""
import csv
import datetime
import logging
import time
from pathlib import Path

import numpy as np
import yaml

from .calibration.gravity import estimate_gravity
from .calibration.rgb_alignment import check_alignment, render_previews
from .ingestion.video import align_video_to_odometry
from .evaluation.gates import gate_results
from .ingestion.rgbd import depth_validity, load_capture
from .plan_outputs import (apply_overrides, build_geometry, dump, plan_summary, setup_logging,  # noqa: F401
                           split_half_consistency, write_outputs)
from .reconstruction.fusion import fuse
from .registration.conventions import resolve_conventions
from .registration.drift import correct_drift, revisit_consistency, revisit_pairs
from .registration.pose_graph import correct_drift_pose_graph
from .reporting import provenance
from .reporting.render import render_floorplan
from .rooms.floorplan import build_floorplan

REPO_ROOT = Path(__file__).resolve().parents[2]
log = logging.getLogger('property_capture')


def _write_trajectories(path, cap, T_raw, T_corr):
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['frame', 'timestamp', 'raw_x', 'raw_y', 'raw_z', 'corrected_x', 'corrected_y', 'corrected_z',
                    'correction_m'])
        for i in range(len(cap)):
            a, b = T_raw[i, :3, 3], T_corr[i, :3, 3]
            w.writerow([int(cap.frames[i]), f'{cap.timestamps[i]:.6f}', *[f'{v:.5f}' for v in a],
                        *[f'{v:.5f}' for v in b], f'{np.linalg.norm(b - a):.5f}'])


def run(input_path, config_path, output_dir, gates_path, overrides=None):
    t_start = time.time()
    with open(config_path, encoding='utf-8') as f:
        cfg = apply_overrides(yaml.safe_load(f), overrides)
    rng = np.random.default_rng(cfg['seed'])

    out = Path(output_dir)
    if out.exists() and any(out.iterdir()):
        raise FileExistsError(f'{out} exists and is not empty; runs never overwrite')
    (out / 'intermediates').mkdir(parents=True, exist_ok=True)
    setup_logging(out)
    timings = {}

    def stage(name, t0):
        timings[name] = round(time.time() - t0, 2)
        log.info('%s done in %.1f s', name, timings[name])

    t0 = time.time()
    cap = load_capture(input_path, REPO_ROOT / 'data' / 'raw')
    log.info('capture %s: %d frames, depth %s, rgb %s', cap.capture_id, len(cap), cap.depth_size, cap.rgb_size)
    prov = provenance.collect(REPO_ROOT, cap.root, cfg)
    stage('load_and_hash', t0)

    warnings, unobservable = [], []
    warnings.append({'code': 'DEPTH_UNIT_ASSUMED', 'message': f"depth unit {cfg['depth']['unit_scale_m']} m/count (assumptions.md B-03)"})
    validity = depth_validity(cap, cfg['depth'])
    log.info('depth validity: %.0f%% of sampled pixels usable, median non-zero depth %.2f m',
             100 * validity['valid_fraction'], validity['median_nonzero_depth_m'])
    if validity['valid_fraction'] < cfg['depth']['min_valid_fraction']:
        raise ValueError(
            f"only {validity['valid_fraction']:.1%} of sampled depth pixels are usable (confidence >= "
            f"{cfg['depth']['confidence_min']}, {cfg['depth']['min_m']}-{cfg['depth']['max_m']} m); median non-zero "
            f"depth {validity['median_nonzero_depth_m']:.2f} m with unit {cfg['depth']['unit_scale_m']} m/count. "
            'Depth is missing, all low confidence, or in other units (assumptions.md B-03)')
    if validity['valid_fraction'] < cfg['depth']['warn_valid_fraction']:
        warnings.append({'code': 'DEPTH_SPARSE',
                         'message': f"only {validity['valid_fraction']:.0%} of sampled depth pixels are usable"})

    t0 = time.time()
    conv, T = resolve_conventions(cap, cfg['pose_convention'], cfg['depth'])
    dump(out / 'intermediates' / 'pose_conventions.json', conv)
    log.info('pose convention %s, residual %.4f m (runner-up %s)', conv['selected'],
             conv['selected_median_abs_residual_m'], conv['runner_up_median_abs_residual_m'])
    log.info('depth scale sweep best = %s', conv['depth_scale_best'])
    if conv['depth_scale_best'] != 1.0:
        warnings.append({'code': 'DEPTH_SCALE_MISMATCH',
                         'message': f"depth scale {conv['depth_scale_best']} fits odometry better than 1.0"})
    stage('pose_conventions', t0)

    steps = np.linalg.norm(np.diff(T[:, :3, 3], axis=0), axis=1)
    jumps = np.flatnonzero(steps > cfg['checks']['jump_threshold_m']) + 1
    if len(jumps):
        # a relocalization or tracking failure moves the pose without the camera moving: geometry
        # from either side of it is duplicated or shifted (assumptions.md B-19)
        warnings.append({'code': 'TRACKING_JUMP',
                         'message': f'{len(jumps)} pose step(s) above {cfg["checks"]["jump_threshold_m"]} m between '
                                    f'consecutive frames, largest {steps.max():.2f} m, into frame(s) '
                                    f'{[int(cap.frames[j]) for j in jumps[:10]]}; geometry may be duplicated or '
                                    'shifted there',
                         'frames': [int(cap.frames[j]) for j in jumps]})
        log.info('tracking jumps into frames %s (largest %.2f m)', [int(cap.frames[j]) for j in jumps[:10]],
                 steps.max())

    t0 = time.time()
    grav = estimate_gravity(cap, T, cfg['gravity'])
    dump(out / 'intermediates' / 'gravity.json', grav)
    log.info('gravity spread %.2f deg (runner-up %.2f), up %s', grav['mean_angular_spread_deg'],
             grav['runner_up_spread_deg'], np.round(grav['up_world'], 3))
    if not grav['consistent']:
        warnings.append({'code': 'GRAVITY_INCONSISTENT',
                         'message': f"gravity direction spread {grav['mean_angular_spread_deg']:.1f} deg"})
    up = np.array(grav['up_world'])
    stage('gravity', t0)

    axes = conv['selected']['camera_axes']
    rgb = {'enabled': bool(cfg['rgb']['enabled'])}
    if rgb['enabled']:
        t0 = time.time()
        rcfg = cfg['rgb']
        summary, details, previews = check_alignment(
            cap, T, list(range(0, len(cap), rcfg['frame_stride'])), cap.root / 'rgb.mp4',
            align_video_to_odometry, rcfg, cfg['depth'])
        rgb.update(summary)
        dump(out / 'intermediates' / 'rgb_alignment.json', {'summary': summary, **details})
        render_previews(out / 'rgb_alignment.png', previews)
        pairing = summary.get('video_pairing')
        log.info('rgb: pairing %s (exact %s); %d frames, median shift %s depth px, quadrant deviation %s, '
                 'paired frame best in %s of frames', pairing and pairing['mapping'], pairing and pairing['best_is_exact'],
                 summary.get('frames', 0), summary.get('median_shift_depth_px'),
                 summary.get('quadrant_max_deviation_depth_px'), summary.get('temporal_paired_frame_best_fraction'))
        if not pairing or not (pairing['best_is_exact'] and pairing['best_is_unique']):
            warnings.append({'code': 'VIDEO_PAIRING_UNCERTAIN',
                             'message': 'video-to-depth frame pairing not pinned down by the gap pattern'})
        if summary.get('frames', 0) and summary['status'] == 'inconsistent':
            warnings.append({'code': 'RGB_DEPTH_MISALIGNED',
                             'message': f"depth edges sit {summary['median_shift_depth_px']} depth px from RGB edges "
                                        f"and image quadrants disagree by {summary['quadrant_max_deviation_depth_px']} px"})
        elif summary.get('frames', 0) and summary['status'] == 'constant_offset':
            warnings.append({'code': 'RGB_DEPTH_OFFSET',
                             'message': f"constant depth-to-RGB offset {np.round(summary['depth_to_rgb_offset_px'], 2)} "
                                        'depth px; apply when projecting between depth and RGB'})
        frac = summary.get('temporal_paired_frame_best_fraction')
        if frac is not None and frac < rcfg['temporal_min_fraction']:
            warnings.append({'code': 'VIDEO_PAIRING_NOT_CONFIRMED',
                             'message': f'paired video frame aligns best in only {frac:.0%} of checked frames'})
        stage('rgb_alignment', t0)

    T_raw = T
    dcfg_drift = cfg['drift']
    drift = {'enabled': bool(dcfg_drift['enabled'])}
    T_corr = None
    if drift['enabled']:
        t0 = time.time()
        if dcfg_drift['method'] == 'pose_graph':
            T_corr, drift_summary, drift_report = correct_drift_pose_graph(cap, T_raw, axes, up, dcfg_drift,
                                                                           cfg['depth'])
            held_out = drift_report['keyframes']
        elif dcfg_drift['method'] == 'keyframe_to_map':
            T_corr, drift_summary, drift_report = correct_drift(cap, T_raw, axes, up, dcfg_drift, cfg['depth'])
            held_out = [e['frame'] for e in drift_report]
        else:
            raise ValueError(f"unknown drift.method {dcfg_drift['method']!r}")
        dump(out / 'intermediates' / 'drift_report.json', drift_report)
        np.savez_compressed(out / 'intermediates' / 'trajectories.npz', raw=T_raw, corrected=T_corr,
                            timestamps=cap.timestamps, frames=cap.frames)
        _write_trajectories(out / 'intermediates' / 'trajectories.csv', cap, T_raw, T_corr)
        # validation pairs never use keyframes, so they are held out from the loop constraints
        pairs = revisit_pairs(cap, T_raw, axes, dcfg_drift, cfg['depth'], exclude=held_out)
        revisit = revisit_consistency(cap, {'raw': T_raw, 'corrected': T_corr}, pairs, axes, dcfg_drift,
                                      cfg['depth'])
        drift.update({'summary': drift_summary,
                      'revisit_consistency': {
                          'method': 'depth reprojection residual between frames >= '
                                    f"{dcfg_drift['revisit_min_gap_s']:.0f} s apart that see the same surfaces "
                                    f"(>= {dcfg_drift['revisit_min_overlap']:.0%} view overlap under raw poses, "
                                    'which favours raw)',
                          'pairs': len(pairs), **revisit}})
        log.info('drift (%s): %d keyframes, %d loop/keyframe constraints used, %d rejected, jumps at %s, '
                 'max correction %.3f m / %.2f deg', dcfg_drift['method'],
                 drift_summary['keyframes'], drift_summary['accepted'], drift_summary['rejected'],
                 drift_summary['jumps_detected_at_frames'], drift_summary['max_correction_translation_m'],
                 drift_summary['max_correction_yaw_deg'])

        # Validate before applying: corrected poses are used only if they make revisits agree
        # better on both measures; otherwise the device poses are kept and the reason recorded.
        raw_r, cor_r = revisit.get('raw'), revisit.get('corrected')
        n_inf = revisit['informative_pairs']
        evidence = n_inf >= dcfg_drift['validation_min_pairs'] and raw_r and cor_r
        if evidence:
            log.info('revisit residual over %d informative pairs (%d dropped): raw %.4f m (%.0f%% within 5 cm), '
                     'corrected %.4f m (%.0f%%)', n_inf, revisit['pairs_dropped_uninformative'],
                     raw_r['median_abs_residual_m'], 100 * raw_r['mean_within_5cm'],
                     cor_r['median_abs_residual_m'], 100 * cor_r['mean_within_5cm'])
            better = (cor_r['median_abs_residual_m'] < raw_r['median_abs_residual_m'] and
                      cor_r['mean_within_5cm'] >= raw_r['mean_within_5cm'])
            decision = ('applied: corrected poses agree better at revisits' if better else
                        'rejected: corrected poses agree worse at revisits than device poses')
        else:
            better = False
            decision = (f"rejected: too few informative revisit pairs to validate ({n_inf} of {len(pairs)} < "
                        f"{dcfg_drift['validation_min_pairs']})")
        drift.update({'applied': better, 'decision': decision})
        log.info('drift correction %s', decision)
        if better:
            T = T_corr
            warnings.append({'code': 'DRIFT_CORRECTED', 'message': decision})
        else:
            warnings.append({'code': 'DRIFT_CORRECTION_REJECTED',
                             'message': decision + '; device odometry kept (raw and corrected trajectories saved)'})
        stage('drift_correction', t0)
    else:
        drift['applied'] = False
        warnings.append({'code': 'POSES_USED_AS_IS', 'message': 'device odometry used without drift correction'})
    geo = build_geometry(cap, T, axes, up, cfg, out, rng, warnings, stage)
    fused, plan, frame_ids = geo['fused'], geo['plan'], geo['frame_ids']
    fcfg = cfg['fusion']

    if drift['enabled'] and dcfg_drift['ablation']:
        # PDF drift gate: the stitched footprint with correction on and off. The main plan uses
        # whichever trajectory was kept; the other one is built here.
        t0 = time.time()
        other_name, T_other = ('raw', T_raw) if drift['applied'] else ('corrected', T_corr)
        f_o = fuse(cap, T_other, axes, frame_ids, cfg['depth'], fcfg)
        p_o = build_floorplan(f_o['points'], plan['basis']['up'], T_other[:, :3, 3], cfg['planes'],
                              cfg['floorplan'], rng)
        main_name = 'corrected' if drift['applied'] else 'raw'
        drift['ablation'] = {main_name: plan_summary(plan), other_name: plan_summary(p_o),
                             'fused_voxels': {main_name: int(len(fused['points'])), other_name: int(len(f_o['points']))}}
        if p_o['status'] == 'ok':
            label = 'raw device poses' if other_name == 'raw' else 'drift-corrected poses (rejected)'
            render_floorplan(out / f'floorplan_{other_name}_poses.png', p_o,
                             [f'{cap.capture_id}  ABLATION: {label}'])
        stage('drift_ablation', t0)

    consistency = split_half_consistency(cap, T, axes, frame_ids, plan, cfg, rng, stage)

    gates = gate_results(gates_path, references_available=False, drift_correction=drift['applied'],
                         drift_ablation='ablation' in drift, drift_note=drift.get('decision'))
    write_outputs(
        out, cap, geo, consistency, cfg, prov, warnings, unobservable, gates, input_tier='lidar',
        frame_description='2D floor frame; axes and floor point below are in odometry world coordinates',
        pose_label='drift-corrected poses' if drift['applied'] else 'device poses',
        metrics_head={
            'pose_conventions': {k: conv[k] for k in ('selected', 'selected_median_abs_residual_m', 'runner_up',
                                                      'runner_up_median_abs_residual_m', 'depth_scale_best')},
            'gravity': {k: grav[k] for k in ('mean_angular_spread_deg', 'runner_up_spread_deg',
                                             'device_to_camera_rotation', 'up_world')}},
        metrics_tail={'drift': drift, 'rgb_alignment': rgb})

    timings['total'] = round(time.time() - t_start, 2)
    dump(out / 'run_info.json', {
        'run_id': out.name, 'finished_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'input': str(input_path), 'config_path': str(config_path), 'overrides': list(overrides or []),
        'config': cfg,
        'gates_path': str(gates_path), 'timings_s': timings, **prov,
    })
    log.info('run complete in %.1f s -> %s', timings['total'], out)
    return 0
