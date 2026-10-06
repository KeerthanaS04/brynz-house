import csv
import json
from pathlib import Path

import pytest

from property_capture.evaluation.evaluate import evaluate, write_outputs

GATES = Path(__file__).resolve().parents[2] / 'configs' / 'evaluation' / 'gates.yaml'


def _run(tmp, name, measurements, openings=()):
    d = tmp / name
    d.mkdir()
    prop = {'measurements': [{'id': k, 'value': v, 'uncertainty_status': 'uncalibrated'} for k, v in measurements.items()],
            'openings': [{'physical_id': o, 'type': 'door'} for o in openings]}
    (d / 'property.json').write_text(json.dumps(prop), encoding='utf-8')
    return d


def _refs(tmp, rows):
    p = tmp / 'refs.csv'
    with open(p, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['capture_id', 'room_label', 'item_label', 'quantity', 'value_m', 'reading', 'instrument',
                    'pipeline_id', 'notes'])
        for r in rows:
            w.writerow([*r[:5], 1, 'laser', r[5], ''])
    return p


def test_opening_gate_counts_misses_and_phantoms(tmp_path):
    run = _run(tmp_path, 'A', {'M-opening-00-width': 0.815, 'M-opening-01-width': 0.95},
               openings=['opening-00', 'opening-01', 'opening-02'])
    refs = _refs(tmp_path, [
        ('A', 'r1', 'door-1', 'opening_width', 0.820, 'opening-00'),   # 5 mm: within
        ('A', 'r1', 'door-2', 'opening_width', 0.900, 'opening-01'),   # 5 cm: outside
        ('A', 'r2', 'door-3', 'opening_width', 0.800, ''),             # missed
    ])
    rows, ph, res, _ = evaluate(refs, {'A': run}, GATES)
    g = res['gate_opening_width']
    assert g['within_tolerance'] == 1 and g['missed'] == 1 and g['phantoms'] == 1   # opening-02 unreferenced
    assert g['status'] == 'fail'
    assert abs(g['fraction_with_phantoms'] - 0.25) < 1e-9 and abs(g['fraction_without_phantoms'] - 1 / 3) < 1e-9


def test_ceiling_gate_passes_within_tolerance_and_fails_unmeasured(tmp_path):
    run = _run(tmp_path, 'A', {'M-room-00-ceiling-height': 2.508})
    refs_ok = _refs(tmp_path, [('A', 'r1', 'c1', 'ceiling_height', 2.50, 'room-00'),
                               ('A', 'r1', 'c2', 'ceiling_height', 2.51, 'room-00')])
    rows, _, res, _ = evaluate(refs_ok, {'A': run}, GATES)
    assert res['gate_ceiling_height']['status'] == 'pass'


def test_ceiling_not_measurable_fails(tmp_path):
    run = _run(tmp_path, 'A', {})
    refs = _refs(tmp_path, [('A', 'r1', 'c1', 'ceiling_height', 2.50, 'room-00')])
    _, _, res, _ = evaluate(refs, {'A': run}, GATES)
    assert res['gate_ceiling_height']['status'] == 'fail'
    assert res['gate_ceiling_height']['rooms'][0]['status'] == 'not_measurable'


@pytest.mark.parametrize('b_value, expected', [(4.005, 'pass'), (4.015, 'ambiguous'), (4.05, 'fail')])
def test_repeatability_scores_both_readings_of_or(tmp_path, b_value, expected):
    # wall 4.00 m: 1 cm absolute vs 0.5% = 2 cm relative
    a = _run(tmp_path, 'A', {'M-room-00-wall-01': 4.000})
    b = _run(tmp_path, 'B', {'M-room-00-wall-03': b_value})
    refs = _refs(tmp_path, [('A', 'r1', 'wall-n', 'wall_length', 4.0, 'room-00-wall-01'),
                            ('B', 'r1', 'wall-n', 'wall_length', 4.0, 'room-00-wall-03')])
    _, _, res, _ = evaluate(refs, {'A': a, 'B': b}, GATES)
    assert res['gate_repeatability']['status'] == expected


def test_outputs_are_written_to_a_fresh_folder(tmp_path):
    run = _run(tmp_path, 'A', {'M-room-00-wall-01': 3.99})
    refs = _refs(tmp_path, [('A', 'r1', 'wall-n', 'wall_length', 4.0, 'room-00-wall-01')])
    out = write_outputs(tmp_path / 'eval', *evaluate(refs, {'A': run}, GATES))
    assert {p.name for p in out.iterdir()} == {'per_measurement.csv', 'gate_results.json', 'evaluation.md'}
    with pytest.raises(FileExistsError):
        write_outputs(tmp_path / 'eval', *evaluate(refs, {'A': run}, GATES))


def test_cli_evaluate_and_strict_exit_code(tmp_path):
    from property_capture.cli import main
    run = _run(tmp_path, 'A', {'M-room-00-ceiling-height': 2.60})
    refs = _refs(tmp_path, [('A', 'r1', 'c1', 'ceiling_height', 2.50, 'room-00')])   # 10 cm off: fails
    args = ['evaluate', '--references', str(refs), '--run', f'A={run}', '--output', str(tmp_path / 'e1')]
    assert main(args) == 0
    assert main(args[:-1] + [str(tmp_path / 'e2'), '--strict']) == 1
    assert (tmp_path / 'e1' / 'gate_results.json').exists()


def test_template_example_rows_are_ignored(tmp_path):
    template = Path(__file__).resolve().parents[2] / 'docs' / 'templates' / 'references_template.csv'
    run = _run(tmp_path, 'A', {})
    rows, _, res, _ = evaluate(template, {'A': run}, GATES)
    assert rows == [] and res['gate_opening_width']['status'] == 'not_evaluable'
