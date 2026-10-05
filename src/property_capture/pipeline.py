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
    return {'status': 'ok', 'polygon_method': plan['wall_detection']['polygon_method'],
            'area_m2': r['area_m2'], 'perimeter_m': r['perimeter_m'],
            'walls': len(r['walls']), 'ceiling_height_m': plan['ceiling_height_m'],
            'ceiling_coverage': plan['ceiling_coverage'],
            'rooms': len(plan['rooms']),
            'room_areas_m2': sorted((round(x['area_m2'], 3) for x in plan['rooms']), reverse=True),
            'rooms_total_area_m2': float(sum(x['area_m2'] for x in plan['rooms'])),
            'openings': sum(c['type'] == 'opening' for c in plan['connections'])}


def _floorplan_with_sign_check(points, up, cap, cfg, rng):
    """Build the plan with the gravity-derived up; flip once if no plausible floor is found."""
    lo, hi = cfg['checks']['camera_height_range_m']
    for sign, label in ((1, 'as_estimated'), (-1, 'flipped')):
        plan = build_floorplan(points, sign * up, cap.positions, cfg['planes'], cfg['floorplan'], rng)
        if plan['status'] == 'ok' and lo <= plan['camera_height_above_floor_m']['median'] <= hi:
            return plan, label
    return plan, 'unresolved'


def apply_overrides(cfg, overrides):
    """Apply 'a.b.c=value' overrides (value parsed as YAML) to a nested config dict."""
    for item in overrides or []:
        key, sep, raw = item.partition('=')
        if not sep:
            raise ValueError(f'override {item!r} is not KEY=VALUE')
        node = cfg
        *parents, leaf = key.split('.')
        for p in parents:
            if p not in node or not isinstance(node[p], dict):
                raise KeyError(f'override {key!r}: no config section {p!r}')
            node = node[p]
        if leaf not in node:
            raise KeyError(f'override {key!r}: unknown config key {leaf!r}')
        node[leaf] = yaml.safe_load(raw)
    return cfg


def run(input_path, config_path, output_dir, gates_path, overrides=None):
    t_start = time.time()
    with open(config_path, encoding='utf-8') as f:
        cfg = apply_overrides(yaml.safe_load(f), overrides)
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
    wd = plan['wall_detection']
    if wd['polygon_method'] == 'occupancy_fallback':
        warnings.append({'code': 'WALL_SNAP_FALLBACK', 'message': wd['fallback_reason']})
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
            for k in ('area_m2', 'perimeter_m', 'ceiling_height_m', 'rooms', 'rooms_total_area_m2'):
                if a[k] is not None and b[k] is not None:
                    diffs[f'{k}_abs_diff'] = abs(a[k] - b[k])
        consistency = {'method': 'independent floor plans from first vs second half of frames (same poses, same up)',
                       'note': 'internal consistency only; not the repeatability gate (needs two captures)',
                       **halves, 'differences': diffs}
        stage('half_split_consistency', t0)

    measurements, rooms_out = [], []
    base = {'unit': 'm', 'interval': None, 'uncertainty_status': 'uncalibrated',
            'calibration_status': 'no reference measurements (assumptions.md B-21)'}
    min_cov = cfg['planes']['ceiling_min_coverage']
    for r in plan['rooms']:
        rid = r['room_id']
        snapped = r['outline_method'] == 'wall_snap'
        walls_out = []
        for i, w in enumerate(r['walls']):
            wid, mid = f'{rid}-wall-{i:02d}', f'M-{rid}-wall-{i:02d}'
            flags = [] if w['evidence'] == 'observed' else ['inferred_low_support']
            if not r['visited']:
                flags.append('room_not_entered')
            measurements.append({'id': mid, 'quantity': 'wall_length', 'value': w['length_m'], **base,
                                 'method': ('edge of wall-snapped room polygon on floor plane' if snapped
                                            else 'edge of simplified occupancy contour on floor plane'),
                                 'evidence_ids': [wid], 'quality_flags': flags,
                                 'support_points': w['support_points'], 'observed_fraction': w['observed_fraction'],
                                 'planarity_rms_m': w['planarity_rms_m']})
            walls_out.append({'wall_id': wid, 'start': w['start'], 'end': w['end'],
                              'length_measurement_id': mid, 'evidence': w['evidence']})
        measurements.append({'id': f'M-{rid}-floor-area', 'quantity': 'floor_area', 'value': r['area_m2'], **base,
                             'unit': 'm2', 'method': 'area of room polygon', 'evidence_ids': [rid],
                             'quality_flags': [] if r['visited'] else ['room_not_entered']})

        c = r['ceiling']
        if c['height_m'] is not None and c['coverage'] >= min_cov:
            ceiling = {'status': 'measured', 'measurement_id': f'M-{rid}-ceiling-height'}
            measurements.append({'id': f'M-{rid}-ceiling-height', 'quantity': 'ceiling_height', 'value': c['height_m'],
                                 **base, 'method': 'median height above floor plane of ceiling-plane inliers in the room',
                                 'evidence_ids': ['ceiling-plane', rid], 'quality_flags': [],
                                 'ceiling_coverage': c['coverage'], 'ceiling_inliers': c['inliers']})
        else:
            reason = ('no ceiling plane detected' if plan['ceiling'] is None
                      else f"ceiling observed over {c['coverage']:.0%} of room < {min_cov:.0%}")
            ceiling = {'status': 'not_measurable', 'reason': reason}
            unobservable.append({'quantity': 'ceiling_height', 'room_id': rid, 'reason': reason})
        unobservable.append({'quantity': 'damage', 'room_id': rid, 'reason': 'damage detection not implemented'})
        rooms_out.append({
            'room_id': rid, 'polygon': r['polygon'], 'floor_elevation_m': 0.0,
            'floor_area_measurement_id': f'M-{rid}-floor-area', 'ceiling': ceiling,
            'walls': walls_out, 'openings': [], 'damage': [],
            'quality': {'floor_plane_rms_m': plan['floor']['rms_m'], 'floor_tilt_deg': plan['floor']['tilt_from_up_deg'],
                        'outline_method': r['outline_method'], 'visited': r['visited'],
                        'camera_fraction': r['camera_fraction'],
                        'inferred_walls': sum(w['evidence'] != 'observed' for w in walls_out)},
        })

    connections_out = []
    for i, c in enumerate(plan['connections']):
        cid = f'conn-{i:02d}'
        entry = {'connection_id': cid, 'rooms': c['room_ids'], 'type': c['type'],
                 'shared_boundary_m': c['shared_boundary_m'], 'camera_transitions': c['camera_transitions'],
                 'location': c['location'], 'evidence': c['evidence']}
        if c['type'] == 'opening' and c['opening_width_m'] is None:
            entry['opening_width_status'] = 'not_measurable: passage inferred from the camera path only'
        elif c['type'] == 'opening':
            mid = f'M-{cid}-opening-width'
            entry['opening_width_measurement_id'] = mid
            measurements.append({'id': mid, 'quantity': 'opening_width', 'value': c['opening_width_m'], **base,
                                 'method': 'extent of contact between room regions through free space',
                                 'evidence_ids': [cid], 'quality_flags': ['inferred_from_segmentation']})
        connections_out.append(entry)
    unobservable.append({'quantity': 'openings', 'room_id': None,
                         'reason': 'door/window classification and windows within a single room not implemented; '
                                   'room-to-room openings are listed under connections'})
    not_entered = [r['room_id'] for r in plan['rooms'] if not r['visited']]
    if not_entered:
        warnings.append({'code': 'ROOM_NOT_ENTERED',
                         'message': f'{not_entered}: geometry seen only from outside, partial coverage likely'})
    total_area = sum(r['area_m2'] for r in plan['rooms'])

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
        'rooms': rooms_out,
        'connections': connections_out, 'measurements': measurements,
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
        'wall_detection': {k: v for k, v in plan['wall_detection'].items() if k != 'wall_lines_geometry'},
        'segmentation': {k: v for k, v in plan['segmentation'].items() if k != 'labels'},
        'rooms': [{'room_id': r['room_id'], 'area_m2': r['area_m2'], 'perimeter_m': r['perimeter_m'],
                   'walls': len(r['walls']), 'observed_walls': sum(w['evidence'] == 'observed' for w in r['walls']),
                   'visited': r['visited'], 'camera_fraction': r['camera_fraction'],
                   'outline_method': r['outline_method'], 'ceiling': r['ceiling'],
                   'clipped_area_m2': r['clipped_area_m2'],
                   'outline_info': {k: v for k, v in r['outline_info'].items()
                                    if k not in ('gap_candidates', 'corner_fills')}}
                  for r in plan['rooms']],
        'connections': plan['connections'],
        'walls': [{'room_id': r['room_id'],
                   **{k: w[k] for k in ('length_m', 'support_points', 'observed_fraction', 'planarity_rms_m', 'evidence')}}
                  for r in plan['rooms'] for w in r['walls']],
        'half_split_consistency': consistency,
        'accuracy': 'not_evaluable: no reference measurements',
    }
    _dump(out / 'metrics.json', metrics)
    _dump(out / 'intermediates' / 'wall_lines.json', plan['wall_detection'].get('wall_lines_geometry', []))

    measured = sum(r['ceiling']['status'] == 'measured' for r in rooms_out)
    title = [f'{cap.capture_id}  run {run_id}  (poses as-is)',
             f"{len(rooms_out)} room(s), total floor area {total_area:.2f} m2, "
             f"{sum(c['type'] == 'opening' for c in connections_out)} opening(s); "
             f"ceiling height measured in {measured}/{len(rooms_out)} room(s)"]
    render_floorplan(out / 'floorplan.png', plan, title)

    timings['total'] = round(time.time() - t_start, 2)
    _dump(out / 'run_info.json', {
        'run_id': run_id, 'finished_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'input': str(input_path), 'config_path': str(config_path), 'overrides': list(overrides or []),
        'config': cfg,
        'gates_path': str(gates_path), 'timings_s': timings, **prov,
    })
    log.info('run complete in %.1f s -> %s', timings['total'], out)
    return 0
