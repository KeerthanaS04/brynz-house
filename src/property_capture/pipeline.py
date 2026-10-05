"""Phase 0 RGB-D baseline: one capture in, one run directory out.

Stages: load -> resolve pose conventions -> gravity -> fuse -> floor plan
-> half-split consistency -> property.json, metrics, gates, render.
Poses are used as-is (no drift correction); every measurement is
uncalibrated because the supplied captures have no reference measurements.
"""
import csv
import datetime
import json
import logging
import time
from pathlib import Path

import numpy as np
import yaml

from .calibration.gravity import estimate_gravity
from .evaluation.gates import gate_results
from .ingestion.rgbd import load_capture
from .reconstruction.fusion import fuse
from .registration.conventions import resolve_conventions
from .reporting import provenance
from .reporting.render import render_floorplan
from .rooms.floorplan import build_floorplan

SCHEMA_VERSION = '0.1.0-baseline'
REPO_ROOT = Path(__file__).resolve().parents[2]
log = logging.getLogger('property_capture')


def _jsonable(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if hasattr(o, 'item'):
        return o.item()
    return str(o)


def _dump(path, obj):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(obj, f, indent=2, default=_jsonable)


def _setup_logging(out_dir):
    log.setLevel(logging.INFO)
    log.handlers.clear()
    fmt = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
    for h in (logging.StreamHandler(), logging.FileHandler(out_dir / 'run.log', encoding='utf-8')):
        h.setFormatter(fmt)
        log.addHandler(h)


def _plan_summary(plan):
    if plan['status'] != 'ok':
        return {'status': plan['status'], 'reason': plan.get('reason')}
    r = plan['room']
    return {'status': 'ok', 'area_m2': r['area_m2'], 'perimeter_m': r['perimeter_m'],
            'walls': len(r['walls']), 'ceiling_height_m': plan['ceiling_height_m'],
            'ceiling_coverage': plan['ceiling_coverage']}


def _floorplan_with_sign_check(points, up, cap, cfg, rng):
    """Build the plan with the gravity-derived up; flip once if no plausible floor is found."""
    lo, hi = cfg['checks']['camera_height_range_m']
    for sign, label in ((1, 'as_estimated'), (-1, 'flipped')):
        plan = build_floorplan(points, sign * up, cap.positions, cfg['planes'], cfg['floorplan'], rng)
        if plan['status'] == 'ok' and lo <= plan['camera_height_above_floor_m']['median'] <= hi:
            return plan, label
    return plan, 'unresolved'


def run(input_path, config_path, output_dir, gates_path):
    t_start = time.time()
    with open(config_path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    rng = np.random.default_rng(cfg['seed'])

    out = Path(output_dir)
    if out.exists() and any(out.iterdir()):
        raise FileExistsError(f'{out} exists and is not empty; runs never overwrite')
    (out / 'intermediates').mkdir(parents=True, exist_ok=True)
    _setup_logging(out)
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
    warnings.append({'code': 'POSES_USED_AS_IS', 'message': 'device odometry used without drift correction (baseline)'})
    warnings.append({'code': 'DEPTH_UNIT_ASSUMED', 'message': f"depth unit {cfg['depth']['unit_scale_m']} m/count (assumptions.md B-03)"})

    t0 = time.time()
    conv, T = resolve_conventions(cap, cfg['pose_convention'], cfg['depth'])
    _dump(out / 'intermediates' / 'pose_conventions.json', conv)
    log.info('pose convention %s, residual %.4f m (runner-up %s)', conv['selected'],
             conv['selected_median_abs_residual_m'], conv['runner_up_median_abs_residual_m'])
    log.info('depth scale sweep best = %s', conv['depth_scale_best'])
    if conv['depth_scale_best'] != 1.0:
        warnings.append({'code': 'DEPTH_SCALE_MISMATCH',
                         'message': f"depth scale {conv['depth_scale_best']} fits odometry better than 1.0"})
    stage('pose_conventions', t0)

    t0 = time.time()
    grav = estimate_gravity(cap, T, cfg['gravity'])
    _dump(out / 'intermediates' / 'gravity.json', grav)
    log.info('gravity spread %.2f deg (runner-up %.2f), up %s', grav['mean_angular_spread_deg'],
             grav['runner_up_spread_deg'], np.round(grav['up_world'], 3))
    if not grav['consistent']:
        warnings.append({'code': 'GRAVITY_INCONSISTENT',
                         'message': f"gravity direction spread {grav['mean_angular_spread_deg']:.1f} deg"})
    up = np.array(grav['up_world'])
    stage('gravity', t0)

    t0 = time.time()
    fcfg = cfg['fusion']
    frame_ids = list(range(0, len(cap), fcfg['frame_stride']))
    fused = fuse(cap, T, conv['selected']['camera_axes'], frame_ids, cfg['depth'], fcfg)
    np.savez_compressed(out / 'intermediates' / 'fused_points.npz', points=fused['points'], counts=fused['counts'])
    log.info('fused %d voxels from %d frames', len(fused['points']), len(frame_ids))
    stage('fusion', t0)

    t0 = time.time()
    plan, sign = _floorplan_with_sign_check(fused['points'], up, cap, cfg, rng)
    if plan['status'] != 'ok':
        raise RuntimeError(f"floor plan failed: {plan.get('reason')}")
    if sign != 'as_estimated':
        warnings.append({'code': 'GRAVITY_SIGN', 'message': f'up vector sign {sign} after floor plausibility check'})
    stage('floorplan', t0)

    consistency = None
    if cfg['consistency']['split_halves']:
        t0 = time.time()
        half = len(frame_ids) // 2
        halves = {}
        for name, ids in (('first_half', frame_ids[:half]), ('second_half', frame_ids[half:])):
            f_h = fuse(cap, T, conv['selected']['camera_axes'], ids, cfg['depth'], fcfg)
            p_h = build_floorplan(f_h['points'], plan['basis']['up'], cap.positions, cfg['planes'], cfg['floorplan'], rng)
            halves[name] = _plan_summary(p_h)
        a, b = halves['first_half'], halves['second_half']
        diffs = {}
        if a['status'] == b['status'] == 'ok':
            for k in ('area_m2', 'perimeter_m', 'ceiling_height_m'):
                if a[k] is not None and b[k] is not None:
                    diffs[f'{k}_abs_diff'] = abs(a[k] - b[k])
        consistency = {'method': 'independent floor plans from first vs second half of frames (same poses, same up)',
                       'note': 'internal consistency only; not the repeatability gate (needs two captures)',
                       **halves, 'differences': diffs}
        stage('half_split_consistency', t0)

    room = plan['room']
    measurements, walls_out = [], []
    base = {'unit': 'm', 'interval': None, 'uncertainty_status': 'uncalibrated',
            'calibration_status': 'no reference measurements (assumptions.md B-21)'}
    for i, w in enumerate(room['walls']):
        mid = f'M-wall-{i:02d}'
        flags = [] if w['evidence'] == 'observed' else ['inferred_low_support']
        measurements.append({'id': mid, 'quantity': 'wall_length', 'value': w['length_m'], **base,
                             'method': 'edge of simplified occupancy contour on floor plane',
                             'evidence_ids': [f'wall-{i:02d}'], 'quality_flags': flags,
                             'support_points': w['support_points'], 'observed_fraction': w['observed_fraction'],
                             'planarity_rms_m': w['planarity_rms_m']})
        walls_out.append({'wall_id': f'wall-{i:02d}', 'start': w['start'], 'end': w['end'],
                          'length_measurement_id': mid, 'evidence': w['evidence']})
    measurements.append({'id': 'M-floor-area', 'quantity': 'floor_area', 'value': room['area_m2'], **base, 'unit': 'm2',
                         'method': 'area of room polygon', 'evidence_ids': ['room-00'], 'quality_flags': []})

    min_cov = cfg['planes']['ceiling_min_coverage']
    if plan['ceiling_height_m'] is not None and plan['ceiling_coverage'] >= min_cov:
        ceiling = {'status': 'measured', 'measurement_id': 'M-ceiling-height'}
        measurements.append({'id': 'M-ceiling-height', 'quantity': 'ceiling_height', 'value': plan['ceiling_height_m'],
                             **base, 'method': 'median height of ceiling-plane inliers above floor plane',
                             'evidence_ids': ['ceiling-plane'], 'quality_flags': [],
                             'ceiling_coverage': plan['ceiling_coverage']})
    else:
        reason = ('no ceiling plane detected' if plan['ceiling_height_m'] is None
                  else f"ceiling observed over {plan['ceiling_coverage']:.0%} of room < {min_cov:.0%}")
        ceiling = {'status': 'not_measurable', 'reason': reason}
        unobservable.append({'quantity': 'ceiling_height', 'room_id': 'room-00', 'reason': reason})
    unobservable.append({'quantity': 'openings', 'room_id': 'room-00', 'reason': 'opening detection not in baseline'})
    unobservable.append({'quantity': 'damage', 'room_id': 'room-00', 'reason': 'damage detection not in baseline'})

    run_id = out.name
    prop = {
        'schema_version': SCHEMA_VERSION,
        'schema_note': 'internal schema; published evaluator schema not yet obtained (assumptions.md A-08)',
        'run_id': run_id, 'capture_id': cap.capture_id, 'units': 'm', 'input_tier': 'lidar',
        'software_commit': prov['git']['commit'], 'software_dirty': prov['git']['dirty'],
        'config_hash': prov['config_sha256'],
        'coordinate_frame': {'description': '2D floor frame: origin/axes below are in odometry world coordinates',
                             'origin_world': plan['basis']['origin'], 'x_axis_world': plan['basis']['e1'],
                             'y_axis_world': plan['basis']['e2'], 'up_world': plan['basis']['up']},
        'rooms': [{
            'room_id': 'room-00', 'polygon': room['polygon'], 'floor_elevation_m': 0.0,
            'floor_area_measurement_id': 'M-floor-area', 'ceiling': ceiling,
            'walls': walls_out, 'openings': [], 'damage': [],
            'quality': {'floor_plane_rms_m': plan['floor']['rms_m'], 'floor_tilt_deg': plan['floor']['tilt_from_up_deg'],
                        'inferred_walls': sum(w['evidence'] != 'observed' for w in walls_out)},
        }],
        'connections': [], 'measurements': measurements,
        'warnings': warnings, 'unobservable': unobservable,
        'evaluation': [{'status': 'not_evaluable', 'reason': 'no reference measurements'}],
    }
    _dump(out / 'property.json', prop)

    with open(out / 'per_measurement.csv', 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['id', 'quantity', 'value', 'unit', 'uncertainty_status', 'interval', 'quality_flags',
                    'reference', 'error'])
        for m in measurements:
            w.writerow([m['id'], m['quantity'], f"{m['value']:.4f}", m['unit'], m['uncertainty_status'], '',
                        ';'.join(m['quality_flags']), '', ''])

    gates = gate_results(gates_path, references_available=False, drift_correction=False)
    _dump(out / 'gate_results.json', gates)

    metrics = {
        'pose_conventions': {k: conv[k] for k in ('selected', 'selected_median_abs_residual_m', 'runner_up',
                                                  'runner_up_median_abs_residual_m', 'depth_scale_best')},
        'gravity': {k: grav[k] for k in ('mean_angular_spread_deg', 'runner_up_spread_deg',
                                         'device_to_camera_rotation', 'up_world')},
        'gravity_sign_check': sign,
        'fusion': fused['stats'],
        'floor_plane': {k: plan['floor'][k] for k in ('rms_m', 'tilt_from_up_deg', 'support_fraction')},
        'ceiling_plane': ({k: plan['ceiling'][k] for k in ('rms_m', 'tilt_from_up_deg', 'support_fraction')}
                          if plan['ceiling'] else None),
        'ceiling_coverage': plan['ceiling_coverage'],
        'camera_height_above_floor_m': plan['camera_height_above_floor_m'],
        'room': _plan_summary(plan),
        'walls': [{k: w[k] for k in ('length_m', 'support_points', 'observed_fraction', 'planarity_rms_m', 'evidence')}
                  for w in room['walls']],
        'half_split_consistency': consistency,
        'accuracy': 'not_evaluable: no reference measurements',
    }
    _dump(out / 'metrics.json', metrics)

    title = [f'{cap.capture_id}  run {run_id}  (baseline, poses as-is)',
             f"floor area {room['area_m2']:.2f} m2   perimeter {room['perimeter_m']:.2f} m   "
             f"ceiling: {ceiling.get('reason') or format(plan['ceiling_height_m'], '.3f') + ' m'}"]
    render_floorplan(out / 'floorplan.png', plan, title)

    timings['total'] = round(time.time() - t_start, 2)
    _dump(out / 'run_info.json', {
        'run_id': run_id, 'finished_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'input': str(input_path), 'config_path': str(config_path), 'config': cfg,
        'gates_path': str(gates_path), 'timings_s': timings, **prov,
    })
    log.info('run complete in %.1f s -> %s', timings['total'], out)
    return 0
