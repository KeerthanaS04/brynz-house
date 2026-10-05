from pathlib import Path

from property_capture.evaluation.gates import gate_results

GATES = Path(__file__).resolve().parents[2] / 'configs' / 'evaluation' / 'gates.yaml'


def test_drift_gate_fails_when_poses_are_device_odometry():
    g = gate_results(GATES, references_available=False, drift_correction=False)['gate_drift_accountability']
    assert g['status'] == 'would_fail' and 'no drift correction run' in g['reason']


def test_drift_gate_reports_why_correction_was_rejected():
    note = 'rejected: corrected poses agree worse at revisits than device poses'
    g = gate_results(GATES, references_available=False, drift_correction=False, drift_ablation=True,
                     drift_note=note)['gate_drift_accountability']
    assert g['status'] == 'would_fail' and note in g['reason']


def test_drift_gate_with_applied_correction_and_ablation():
    g = gate_results(GATES, references_available=False, drift_correction=True,
                     drift_ablation=True)['gate_drift_accountability']
    assert g['status'] == 'evidence_produced'


def test_blocked_and_unevaluable_gates():
    gates = gate_results(GATES, references_available=False, drift_correction=False)
    assert gates['gate_round1_unspecified']['status'] == 'blocked'
    assert gates['gate_opening_width']['status'] == 'not_evaluable'
