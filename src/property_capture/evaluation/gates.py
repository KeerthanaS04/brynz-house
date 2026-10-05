"""Gate reporting. Every gate in configs/evaluation/gates.yaml gets its own result."""
import yaml

NO_REFERENCE = 'no reference measurements for this capture (assumptions.md A-01, B-21)'


def gate_results(gates_path, references_available, drift_correction):
    with open(gates_path, encoding='utf-8') as f:
        gates = yaml.safe_load(f)['gates']
    out = {}
    for name, spec in gates.items():
        if spec.get('status') == 'BLOCKED':
            out[name] = {'status': 'blocked', 'reason': spec.get('description', '').strip()}
        elif name == 'gate_drift_accountability' and not drift_correction:
            out[name] = {'status': 'would_fail',
                         'reason': "poses used as-is (no drift correction in baseline); PDF: automatic fail"}
        elif name in ('gate_process_evidence', 'gate_fix_loop', 'gate_walk_in_test'):
            out[name] = {'status': 'not_applicable_to_run', 'reason': 'assessed at submission/defense level'}
        elif not references_available:
            out[name] = {'status': 'not_evaluable', 'reason': NO_REFERENCE}
        else:
            out[name] = {'status': 'not_implemented', 'reason': 'evaluator not implemented yet'}
    return out
