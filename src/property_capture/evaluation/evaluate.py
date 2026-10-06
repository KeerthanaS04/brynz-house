"""Score pipeline runs against laser/tape reference measurements (docs/benchmark_protocol.md).

References: CSV in the format of docs/templates/references_template.csv. Each row is one
reading; an item's reference value is the mean of its readings. `pipeline_id` links the
item to what the pipeline produced:
  wall_length     -> wall id            (room-02-wall-10)    measurement M-<id>
  ceiling_height  -> room id            (room-02)            measurement M-<id>-ceiling-height
  opening_width   -> physical opening   (opening-04)         measurement M-<id>-width
An empty pipeline_id means the pipeline did not produce the item (a miss).

Gates use thresholds from configs/evaluation/gates.yaml only. Where the PDF wording is
ambiguous (assumptions.md A-03 opening denominator, A-04 repeatability 'or'), every
reading is scored and the gate passes or fails only if they agree; otherwise it is
'ambiguous'.
"""
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np
import yaml

QUANTITIES = {
    'wall_length': lambda pid: f'M-{pid}',
    'ceiling_height': lambda pid: f'M-{pid}-ceiling-height',
    'opening_width': lambda pid: f'M-{pid}-width',
}


def load_references(path):
    """Items keyed by (capture_id, room_label, item_label, quantity), readings averaged."""
    items = defaultdict(lambda: {'readings': [], 'pipeline_ids': set(), 'notes': []})
    with open(path, encoding='utf-8') as f:
        for row in csv.DictReader(f):
            if row['capture_id'].startswith('EXAMPLE'):
                continue
            key = (row['capture_id'], row['room_label'], row['item_label'], row['quantity'])
            items[key]['readings'].append(float(row['value_m']))
            if row.get('pipeline_id', '').strip():
                items[key]['pipeline_ids'].add(row['pipeline_id'].strip())
    out = []
    for (cap, room, item, qty), v in items.items():
        if len(v['pipeline_ids']) > 1:
            raise ValueError(f'{cap}/{room}/{item}/{qty}: conflicting pipeline_id {sorted(v["pipeline_ids"])}')
        r = np.asarray(v['readings'])
        out.append({'capture_id': cap, 'room_label': room, 'item_label': item, 'quantity': qty,
                    'reference_m': float(r.mean()), 'readings': len(r),
                    'reference_spread_m': float(np.ptp(r)) if len(r) > 1 else 0.0,
                    'pipeline_id': next(iter(v['pipeline_ids']), None)})
    return out


def load_run(run_dir):
    prop = json.loads((Path(run_dir) / 'property.json').read_text(encoding='utf-8'))
    return {'property': prop, 'measurements': {m['id']: m for m in prop.get('measurements', [])}}


def match(refs, runs):
    """One row per reference item: pipeline value, error and status."""
    rows = []
    for ref in refs:
        row = {**ref, 'measured_m': None, 'error_m': None, 'abs_error_m': None, 'uncertainty_status': None}
        run = runs.get(ref['capture_id'])
        if run is None:
            row['status'] = 'no_run_for_capture'
        elif ref['quantity'] not in QUANTITIES:
            row['status'] = 'quantity_not_scored'
        elif not ref['pipeline_id']:
            row['status'] = 'missed'
        else:
            m = run['measurements'].get(QUANTITIES[ref['quantity']](ref['pipeline_id']))
            if m is None:
                row['status'] = 'not_measurable'
            else:
                err = m['value'] - ref['reference_m']
                row.update({'measured_m': m['value'], 'error_m': err, 'abs_error_m': abs(err),
                            'uncertainty_status': m.get('uncertainty_status'), 'status': 'measured'})
        rows.append(row)
    return rows


def phantoms(refs, runs):
    """Physical openings in a run that no opening_width reference points at."""
    linked = {(r['capture_id'], r['pipeline_id']) for r in refs if r['quantity'] == 'opening_width'}
    out = []
    for cap, run in runs.items():
        if not any(r['capture_id'] == cap and r['quantity'] == 'opening_width' for r in refs):
            continue                                   # openings not referenced at all for this capture
        for o in run['property'].get('openings', []):
            if (cap, o['physical_id']) not in linked:
                out.append({'capture_id': cap, 'physical_id': o['physical_id'], 'type': o['type']})
    return out


def gate_opening_width(rows, phantom_list, spec):
    tol, cov = spec['threshold']['tolerance_cm'] / 100, spec['threshold']['coverage_pct']
    ops = [r for r in rows if r['quantity'] == 'opening_width' and r['status'] != 'no_run_for_capture']
    if not ops:
        return {'status': 'not_evaluable', 'reason': 'no opening_width references'}
    hits = sum(r['status'] == 'measured' and r['abs_error_m'] <= tol for r in ops)
    frac_without = hits / len(ops)
    frac_with = hits / (len(ops) + len(phantom_list))
    readings = {'phantoms_in_denominator': frac_with >= cov, 'phantoms_not_in_denominator': frac_without >= cov}
    status = ('pass' if all(readings.values()) else 'fail' if not any(readings.values()) else 'ambiguous')
    return {'status': status, 'within_tolerance': hits, 'referenced_openings': len(ops),
            'missed': sum(r['status'] == 'missed' for r in ops),
            'not_measurable': sum(r['status'] == 'not_measurable' for r in ops),
            'phantoms': len(phantom_list), 'fraction_with_phantoms': frac_with,
            'fraction_without_phantoms': frac_without, 'required_fraction': cov, 'tolerance_m': tol,
            'readings': readings,
            'note': 'denominator unresolved (assumptions.md A-03): scored with and without phantoms'}


def gate_ceiling_height(rows, spec):
    tol = spec['threshold']['per_room_tolerance_cm'] / 100
    spread_max = spec['threshold']['repeat_spread_cm'] / 100
    ceil = [r for r in rows if r['quantity'] == 'ceiling_height' and r['status'] != 'no_run_for_capture']
    if not ceil:
        return {'status': 'not_evaluable', 'reason': 'no ceiling_height references'}
    per_room = [{'capture_id': r['capture_id'], 'room_label': r['room_label'], 'status': r['status'],
                 'error_m': r['error_m'], 'pass': r['status'] == 'measured' and r['abs_error_m'] <= tol} for r in ceil]
    # rooms referenced in more than one capture: spread of the pipeline's heights
    by_room = defaultdict(list)
    for r in ceil:
        if r['status'] == 'measured':
            by_room[r['room_label']].append(r['measured_m'])
    spreads = {room: float(np.ptp(v)) for room, v in by_room.items() if len(v) > 1}
    spread_ok = all(s <= spread_max for s in spreads.values())
    errors = [p['error_m'] for p in per_room if p['error_m'] is not None]
    bias = float(np.mean(errors)) if errors else None
    ok = all(p['pass'] for p in per_room) and spread_ok
    return {'status': 'pass' if ok else 'fail', 'rooms': per_room, 'tolerance_m': tol,
            'repeat_spreads_m': spreads, 'repeat_spread_limit_m': spread_max, 'mean_error_m': bias,
            'diagnosis': ('unrepeatable' if not spread_ok else
                          'repeatable but biased' if spreads and not ok else None)}


def gate_repeatability(rows, spec):
    """Same item referenced in two captures: compare the pipeline's two values."""
    abs_tol = spec['threshold']['absolute_cm'] / 100
    rel_tol = spec['threshold']['relative_pct'] / 100
    groups = defaultdict(list)
    for r in rows:
        if r['quantity'] == 'wall_length' and r['status'] == 'measured':
            groups[(r['room_label'], r['item_label'])].append(r)
    pairs = []
    for (room, item), rs in groups.items():
        if len({r['capture_id'] for r in rs}) < 2:
            continue
        vals = [r['measured_m'] for r in rs]
        diff = float(np.ptp(vals))
        ref = float(np.mean([r['reference_m'] for r in rs]))
        pairs.append({'room_label': room, 'item_label': item, 'captures': [r['capture_id'] for r in rs],
                      'difference_m': diff, 'pass_either': diff <= abs_tol or diff <= rel_tol * ref,
                      'pass_both': diff <= abs_tol and diff <= rel_tol * ref})
    if not pairs:
        return {'status': 'not_evaluable', 'reason': 'no wall referenced and measured in two captures'}
    either, both = all(p['pass_either'] for p in pairs), all(p['pass_both'] for p in pairs)
    status = 'pass' if both else 'fail' if not either else 'ambiguous'
    return {'status': status, 'walls': pairs, 'readings': {'either_tolerance': either, 'both_tolerances': both},
            'note': "'1 cm or 0.5%' semantics unresolved (assumptions.md A-04): scored both ways"}


def wall_length_errors(rows):
    w = [r for r in rows if r['quantity'] == 'wall_length' and r['status'] == 'measured']
    if not w:
        return None
    e = np.array([r['abs_error_m'] for r in w])
    rel = np.array([r['abs_error_m'] / r['reference_m'] for r in w])
    return {'walls': len(w), 'median_abs_error_m': float(np.median(e)), 'max_abs_error_m': float(e.max()),
            'median_relative_error': float(np.median(rel)), 'max_relative_error': float(rel.max()),
            'missed_or_not_measurable': sum(r['quantity'] == 'wall_length' and r['status'] != 'measured'
                                            for r in rows)}


def evaluate(references_path, runs_by_capture, gates_path):
    refs = load_references(references_path)
    runs = {cap: load_run(p) for cap, p in runs_by_capture.items()}
    rows = match(refs, runs)
    ph = phantoms(refs, runs)
    with open(gates_path, encoding='utf-8') as f:
        gates = yaml.safe_load(f)['gates']
    results = {}
    for name, spec in gates.items():
        if spec.get('status') == 'BLOCKED':
            results[name] = {'status': 'blocked', 'reason': spec.get('description', '').strip()}
        elif name == 'gate_opening_width':
            results[name] = gate_opening_width(rows, ph, spec)
        elif name == 'gate_ceiling_height':
            results[name] = gate_ceiling_height(rows, spec)
        elif name == 'gate_repeatability':
            results[name] = gate_repeatability(rows, spec)
        elif name == 'gate_drift_accountability':
            per_run = {}
            for cap, p in runs_by_capture.items():
                gr = Path(p) / 'gate_results.json'
                per_run[cap] = json.loads(gr.read_text(encoding='utf-8')).get(name) if gr.exists() else None
            results[name] = {'status': 'see_runs', 'per_run': per_run}
        else:
            results[name] = {'status': 'not_evaluable',
                             'reason': 'needs inputs not scored by this command (photo tier, consumer app, '
                                       'walk-in, fix loop or process evidence)'}
    return rows, ph, results, {'wall_length': wall_length_errors(rows)}


def write_outputs(out_dir, rows, phantom_list, results, summary):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=False)
    cols = ['capture_id', 'room_label', 'item_label', 'quantity', 'reference_m', 'readings', 'reference_spread_m',
            'pipeline_id', 'measured_m', 'error_m', 'abs_error_m', 'status', 'uncertainty_status']
    with open(out_dir / 'per_measurement.csv', 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)
    payload = {'gates': results, 'phantom_openings': phantom_list, 'summary': summary}
    (out_dir / 'gate_results.json').write_text(json.dumps(payload, indent=2, default=str), encoding='utf-8')
    lines = ['# Evaluation', '', '| gate | status | detail |', '|---|---|---|']
    for name, g in results.items():
        detail = g.get('reason') or g.get('note') or ''
        if name == 'gate_opening_width' and 'within_tolerance' in g:
            detail = (f"{g['within_tolerance']}/{g['referenced_openings']} within {100 * g['tolerance_m']:.0f} cm; "
                      f"missed {g['missed']}, not measurable {g['not_measurable']}, phantoms {g['phantoms']}")
        lines.append(f"| {name} | {g['status']} | {detail} |")
    if summary.get('wall_length'):
        s = summary['wall_length']
        lines += ['', f"Wall lengths: {s['walls']} measured, median error {100 * s['median_abs_error_m']:.1f} cm "
                      f"({100 * s['median_relative_error']:.2f}%), max {100 * s['max_abs_error_m']:.1f} cm; "
                      f"{s['missed_or_not_measurable']} missed or not measurable."]
    (out_dir / 'evaluation.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return out_dir
