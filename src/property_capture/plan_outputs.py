"""Stages shared by every input tier once frames have poses and depth.

A tier supplies a capture-like object (frames with depth, confidence and intrinsics; see
ingestion.rgbd.RGBDCapture), camera-to-world poses and an up direction. From there:
fusion -> floor plan -> openings -> half-split consistency -> property.json, metrics,
per-measurement CSV, gates and the rendered plan. Field meanings are the same for all tiers.
"""
import csv
import json
import logging
import time

import numpy as np
import yaml

from .reconstruction.fusion import fuse
from .reporting.render import render_floorplan
from .rooms.floorplan import build_floorplan
from .rooms.openings import detect_openings

SCHEMA_VERSION = '0.1.0-baseline'
log = logging.getLogger('property_capture')


def _jsonable(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if hasattr(o, 'item'):
        return o.item()
    return str(o)


def dump(path, obj):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(obj, f, indent=2, default=_jsonable)


def setup_logging(out_dir):
    log.setLevel(logging.INFO)
    log.handlers.clear()
    fmt = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
    for h in (logging.StreamHandler(), logging.FileHandler(out_dir / 'run.log', encoding='utf-8')):
        h.setFormatter(fmt)
        log.addHandler(h)


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


class Timer:
    """Per-stage wall-clock timings, logged as each stage ends."""

    def __init__(self):
        self.timings = {}

    def __call__(self, name, t0):
        self.timings[name] = round(time.time() - t0, 2)
        log.info('%s done in %.1f s', name, self.timings[name])


def plan_summary(plan):
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


def floorplan_with_sign_check(points, up, camera_positions, cfg, rng):
    """Build the plan with the given up; flip once if no plausible floor is found."""
    lo, hi = cfg['checks']['camera_height_range_m']
    for sign, label in ((1, 'as_estimated'), (-1, 'flipped')):
        plan = build_floorplan(points, sign * up, camera_positions, cfg['planes'], cfg['floorplan'], rng)
        if plan['status'] == 'ok' and lo <= plan['camera_height_above_floor_m']['median'] <= hi:
            return plan, label
    return plan, 'unresolved'


def build_geometry(cap, T, axes, up, cfg, out, rng, warnings, stage):
    """Fusion, floor plan and openings. Returns fused points, plan, sign check, openings and frame ids."""
    cam_pos = T[:, :3, 3]
    t0 = time.time()
    fcfg = cfg['fusion']
    frame_ids = list(range(0, len(cap), fcfg['frame_stride']))
    fused = fuse(cap, T, axes, frame_ids, cfg['depth'], fcfg)
    np.savez_compressed(out / 'intermediates' / 'fused_points.npz', points=fused['points'], counts=fused['counts'])
    log.info('fused %d voxels from %d frames', len(fused['points']), len(frame_ids))
    stage('fusion', t0)

    t0 = time.time()
    plan, sign = floorplan_with_sign_check(fused['points'], up, cam_pos, cfg, rng)
    if plan['status'] != 'ok':
        raise RuntimeError(f"floor plan failed: {plan.get('reason')}")
    if sign != 'as_estimated':
        warnings.append({'code': 'GRAVITY_SIGN', 'message': f'up vector sign {sign} after floor plausibility check'})
    wd = plan['wall_detection']
    if wd['polygon_method'] == 'occupancy_fallback':
        warnings.append({'code': 'WALL_SNAP_FALLBACK', 'message': wd['fallback_reason']})
    stage('floorplan', t0)

    openings, open_gaps, open_stats = [], [], {'enabled': bool(cfg['openings']['enabled'])}
    if cfg['openings']['enabled']:
        t0 = time.time()
        openings, open_gaps, stats_ = detect_openings(cap, T, axes, plan, cfg['openings'], cfg['depth'])
        open_stats.update(stats_)
        widths = {p['physical_id']: p['width_m'] for p in stats_['physical']}
        for o in openings:
            o['physical_width_m'] = widths[o['physical_id']]
        plan['openings'] = openings
        log.info('openings: %d found (%d physical; %s), %d unbounded gaps excluded', len(openings),
                 stats_['physical_openings'], ', '.join(f"{o['type']} {o['width_m']:.2f} m" for o in openings),
                 len(open_gaps))
        stage('openings', t0)
    return {'fused': fused, 'plan': plan, 'sign': sign, 'openings': openings, 'open_gaps': open_gaps,
            'open_stats': open_stats, 'frame_ids': frame_ids}


def split_half_consistency(cap, T, axes, frame_ids, plan, cfg, rng, stage):
    if not cfg['consistency']['split_halves']:
        return None
    t0 = time.time()
    cam_pos = T[:, :3, 3]
    half = len(frame_ids) // 2
    halves = {}
    for name, ids in (('first_half', frame_ids[:half]), ('second_half', frame_ids[half:])):
        f_h = fuse(cap, T, axes, ids, cfg['depth'], cfg['fusion'])
        p_h = build_floorplan(f_h['points'], plan['basis']['up'], cam_pos, cfg['planes'], cfg['floorplan'], rng)
        halves[name] = plan_summary(p_h)
    a, b = halves['first_half'], halves['second_half']
    diffs = {}
    if a['status'] == b['status'] == 'ok':
        for k in ('area_m2', 'perimeter_m', 'ceiling_height_m', 'rooms', 'rooms_total_area_m2'):
            if a[k] is not None and b[k] is not None:
                diffs[f'{k}_abs_diff'] = abs(a[k] - b[k])
    stage('half_split_consistency', t0)
    return {'method': 'independent floor plans from first vs second half of frames (same poses, same up)',
            'note': 'internal consistency only; not the repeatability gate (needs two captures)',
            **halves, 'differences': diffs}


def write_outputs(out, cap, geo, consistency, cfg, prov, warnings, unobservable, gates, *, input_tier,
                  frame_description, pose_label, metrics_head=None, metrics_tail=None, measurement_flags=(),
                  calibration_status='no reference measurements (assumptions.md B-21)'):
    """property.json, per_measurement.csv, gate_results.json, metrics.json, wall_lines.json, floorplan.png."""
    plan, openings, open_stats, open_gaps = geo['plan'], geo['openings'], geo['open_stats'], geo['open_gaps']
    measurement_flags = list(measurement_flags)
    measurements, rooms_out = [], []
    base = {'unit': 'm', 'interval': None, 'uncertainty_status': 'uncalibrated',
            'calibration_status': calibration_status}

    def room_openings(rid):
        """This room's detections; widths are measured once per physical opening (below)."""
        return [{'opening_id': o['opening_id'], 'physical_opening_id': o['physical_id'], 'type': o['type'],
                 'wall_id': f"{rid}-wall-{o['edge']:02d}", 'detected_width_m': o['width_m'],
                 'jamb_wall_fraction': o['jamb_wall_fraction'], 'height_m': o['height_m'], 'sill_m': o['sill_m'],
                 'head_m': o['head_m'], 'start': o['start'], 'end': o['end'], 'same_as': o['same_as'],
                 'matches_room_connection': o['matches_connection'], 'accepted_by': o['accepted_by'],
                 'evidence': 'observed: camera rays passed through the wall plane here'}
                for o in openings if o['room_id'] == rid]

    physical_out = []
    for p in open_stats.get('physical', []):
        entry = {k: p[k] for k in ('physical_id', 'type', 'members', 'width_status', 'candidate_width_m',
                                   'end_spread_m', 'jamb_estimates', 'centre', 'sill_m', 'head_m')}
        if p['width_m'] is not None:
            mid = f"M-{p['physical_id']}-width"
            measurements.append({
                'id': mid, 'quantity': 'opening_width', 'value': p['width_m'], **base,
                'method': 'distance between jambs seen by camera rays (mean over rooms that saw each jamb), '
                          'edges refined within the 2 cm cell by through fraction',
                'evidence_ids': [p['physical_id'], *p['members']],
                'quality_flags': ([] if len(p['members']) > 1 or p['end_spread_m'] == 0 else ['seen_from_one_room'])
                                 + measurement_flags,
                'end_spread_m': p['end_spread_m']})
            entry['width_measurement_id'] = mid
        else:
            unobservable.append({'quantity': 'opening_width', 'opening_id': p['physical_id'],
                                 'reason': p['width_status']})
        physical_out.append(entry)

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
                                 'evidence_ids': [wid], 'quality_flags': flags + measurement_flags,
                                 'support_points': w['support_points'], 'observed_fraction': w['observed_fraction'],
                                 'planarity_rms_m': w['planarity_rms_m']})
            walls_out.append({'wall_id': wid, 'start': w['start'], 'end': w['end'],
                              'length_measurement_id': mid, 'evidence': w['evidence']})
        measurements.append({'id': f'M-{rid}-floor-area', 'quantity': 'floor_area', 'value': r['area_m2'], **base,
                             'unit': 'm2', 'method': 'area of room polygon', 'evidence_ids': [rid],
                             'quality_flags': ([] if r['visited'] else ['room_not_entered']) + measurement_flags})

        c = r['ceiling']
        if c['status'] == 'measured':
            ceiling = {'status': 'measured', 'measurement_id': f'M-{rid}-ceiling-height'}
            measurements.append({
                'id': f'M-{rid}-ceiling-height', 'quantity': 'ceiling_height', 'value': c['height_m'], **base,
                'method': 'median of per-cell ceiling levels minus median of per-cell floor levels in the room',
                'evidence_ids': ['ceiling-plane', rid], 'quality_flags': list(measurement_flags),
                'standard_error_m': c['standard_error_m'],
                'standard_error_note': 'precision indicator from cell-level spread; not a calibrated interval',
                'ceiling_cells': c['ceiling_cells'], 'flatness_iqr_m': c['flatness_iqr_m'],
                'floor_source': c['floor_source'], 'ceiling_coverage': c['coverage']})
        else:
            ceiling = {'status': 'not_measurable', 'reason': c['reason'],
                       'candidate_height_m': c.get('candidate_height_m')}
            unobservable.append({'quantity': 'ceiling_height', 'room_id': rid, 'reason': c['reason']})
        unobservable.append({'quantity': 'damage', 'room_id': rid, 'reason': 'damage detection not implemented'})
        rooms_out.append({
            'room_id': rid, 'polygon': r['polygon'], 'floor_elevation_m': 0.0,
            'floor_area_measurement_id': f'M-{rid}-floor-area', 'ceiling': ceiling,
            'walls': walls_out, 'openings': room_openings(rid), 'damage': [],
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
                                 'evidence_ids': [cid],
                                 'quality_flags': ['inferred_from_segmentation'] + measurement_flags})
        connections_out.append(entry)
    shared_out = []
    for sw in plan['shared_walls']:
        faces = [[f'{rid}-wall-{e:02d}' for rid, e in zip(sw['rooms'], pair)] for pair in sw['faces']]
        entry = {**{k: sw[k] for k in ('shared_wall_id', 'rooms', 'overlap_m', 'angle_deg', 'centrelines',
                                       'thickness_status', 'excluded_outline_jogs', 'evidence')},
                 'face_pairs': faces}
        if sw['thickness_m'] is not None:
            mid = f"M-{sw['shared_wall_id']}-thickness"
            measurements.append({'id': mid, 'quantity': 'wall_thickness', 'value': sw['thickness_m'], **base,
                                 'method': 'overlap-weighted median distance between paired facing edges of '
                                           'neighbouring rooms',
                                 'evidence_ids': [sw['shared_wall_id'], *[f for pair in faces for f in pair]],
                                 'quality_flags': ['inferred_from_room_outlines'] + measurement_flags,
                                 'thickness_variation_m': sw['thickness_variation_m'], 'overlap_m': sw['overlap_m']})
            entry['thickness_measurement_id'] = mid
        else:
            entry['candidate_thickness_m'] = sw['candidate_thickness_m']
        shared_out.append(entry)
    unobservable.append({'quantity': 'openings', 'room_id': None,
                         'reason': 'openings are detected only where the camera saw through them; closed doors '
                                   'and covered windows are not detected' if cfg['openings']['enabled'] else
                                   'opening detection disabled; room-to-room openings are listed under connections'})
    not_entered = [r['room_id'] for r in plan['rooms'] if not r['visited']]
    if not_entered:
        warnings.append({'code': 'ROOM_NOT_ENTERED',
                         'message': f'{not_entered}: geometry seen only from outside, partial coverage likely'})
    total_area = sum(r['area_m2'] for r in plan['rooms'])

    run_id = out.name
    prop = {
        'schema_version': SCHEMA_VERSION,
        'schema_note': 'internal schema; published evaluator schema not yet obtained (assumptions.md A-08)',
        'run_id': run_id, 'capture_id': cap.capture_id, 'units': 'm', 'input_tier': input_tier,
        'software_commit': prov['git']['commit'], 'software_dirty': prov['git']['dirty'],
        'config_hash': prov['config_sha256'],
        'coordinate_frame': {'description': frame_description,
                             'origin_world': plan['basis']['origin'], 'x_axis_world': plan['basis']['e1'],
                             'y_axis_world': plan['basis']['e2'], 'up_world': plan['basis']['up']},
        'rooms': rooms_out,
        'connections': connections_out, 'shared_walls': shared_out, 'openings': physical_out,
        'measurements': measurements,
        'warnings': warnings, 'unobservable': unobservable,
        'evaluation': [{'status': 'not_evaluable', 'reason': 'no reference measurements'}],
    }
    dump(out / 'property.json', prop)

    with open(out / 'per_measurement.csv', 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['id', 'quantity', 'value', 'unit', 'uncertainty_status', 'interval', 'quality_flags',
                    'reference', 'error'])
        for m in measurements:
            w.writerow([m['id'], m['quantity'], f"{m['value']:.4f}", m['unit'], m['uncertainty_status'], '',
                        ';'.join(m['quality_flags']), '', ''])

    dump(out / 'gate_results.json', gates)

    metrics = {
        **(metrics_head or {}),
        'gravity_sign_check': geo['sign'],
        'fusion': geo['fused']['stats'],
        'floor_plane': {k: plan['floor'][k] for k in ('rms_m', 'tilt_from_up_deg', 'support_fraction')},
        'ceiling_plane': ({k: plan['ceiling'][k] for k in ('rms_m', 'tilt_from_up_deg', 'support_fraction')}
                          if plan['ceiling'] else None),
        'ceiling_coverage': plan['ceiling_coverage'],
        'camera_height_above_floor_m': plan['camera_height_above_floor_m'],
        'room': plan_summary(plan),
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
        'openings': {
            **{k: v for k, v in open_stats.items() if k != 'physical'},
            'physical': [{k: p[k] for k in ('physical_id', 'type', 'members', 'width_m', 'width_status',
                                            'candidate_width_m', 'end_spread_m', 'jamb_estimates')}
                         for p in open_stats.get('physical', [])],
            'found': [{k: o[k] for k in ('opening_id', 'type', 'width_m', 'height_m', 'sill_m', 'head_m', 'same_as',
                                         'matches_connection', 'accepted_by', 'through_rays', 'jamb_wall_fraction')}
                      for o in openings],
            'unbounded_gaps_excluded': [{k: o[k] for k in ('room_id', 'edge', 'type', 'width_m', 'sill_m', 'head_m',
                                                          'jamb_wall_fraction')} for o in open_gaps],
        },
        'wall_alignment': {
            'aligned': sum(a['status'] == 'aligned' for a in plan['wall_alignment']),
            'skipped': [{k: a[k] for k in ('shared_wall_id', 'rooms', 'reason')}
                        for a in plan['wall_alignment'] if a['status'] == 'skipped'],
            'rotation_deg': [round(a['max_rotation_deg'], 3) for a in plan['wall_alignment']
                             if a['status'] == 'aligned'],
            'area_change_m2': [round(v, 4) for a in plan['wall_alignment'] if a['status'] == 'aligned'
                               for v in a['area_change_m2'].values()],
            'fallbacks': [{k: a[k] for k in ('shared_wall_id', 'scope', 'notes')}
                          for a in plan['wall_alignment'] if a['status'] == 'aligned'
                          and (a['scope'] != 'all faces' or a['notes'])],
        },
        'shared_walls': {
            'count': len(plan['shared_walls']),
            'thickness_m': [round(s['thickness_m'], 4) if s['thickness_m'] is not None else None
                            for s in plan['shared_walls']],
            'candidate_thickness_m': [round(s['candidate_thickness_m'], 4) for s in plan['shared_walls']],
            'thickness_status': [s['thickness_status'] for s in plan['shared_walls']],
            'overlap_m': [round(s['overlap_m'], 3) for s in plan['shared_walls']],
            'excluded_outline_jogs': sum(len(s['excluded_outline_jogs']) for s in plan['shared_walls']),
            'gap_area_m2': float(sum(s['gap_area_m2'] for s in plan['shared_walls'])),
            'room_pairs': sorted({tuple(s['rooms']) for s in plan['shared_walls']}),
        },
        'walls': [{'room_id': r['room_id'],
                   **{k: w[k] for k in ('length_m', 'support_points', 'observed_fraction', 'planarity_rms_m', 'evidence')}}
                  for r in plan['rooms'] for w in r['walls']],
        'half_split_consistency': consistency,
        **(metrics_tail or {}),
        'accuracy': 'not_evaluable: no reference measurements',
    }
    dump(out / 'metrics.json', metrics)
    dump(out / 'intermediates' / 'wall_lines.json', plan['wall_detection'].get('wall_lines_geometry', []))

    measured = sum(r['ceiling']['status'] == 'measured' for r in rooms_out)
    title = [f"{cap.capture_id}  run {run_id}  ({pose_label})",
             f"{len(rooms_out)} room(s), total floor area {total_area:.2f} m2, "
             f"{sum(c['type'] == 'opening' for c in connections_out)} opening(s); "
             f"ceiling height measured in {measured}/{len(rooms_out)} room(s)"]
    render_floorplan(out / 'floorplan.png', plan, title)
    return prop
