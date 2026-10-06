"""Check that two runs of the same capture produced the same measurements (regression check).

Usage:
  python scripts/diff_runs.py outputs/run_a outputs/run_b [--tol 1e-9]
Exit code 0 if every measurement id, value and warning code matches; 1 otherwise.
"""
import argparse
import json
import sys
from pathlib import Path


def load(run):
    return json.loads((Path(run) / 'property.json').read_text(encoding='utf-8'))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('a')
    ap.add_argument('b')
    ap.add_argument('--tol', type=float, default=1e-9)
    args = ap.parse_args(argv)
    a, b = load(args.a), load(args.b)
    ma = {m['id']: m['value'] for m in a['measurements']}
    mb = {m['id']: m['value'] for m in b['measurements']}
    problems = [f'only in {args.a}: {k}' for k in sorted(ma.keys() - mb.keys())]
    problems += [f'only in {args.b}: {k}' for k in sorted(mb.keys() - ma.keys())]
    problems += [f'{k}: {ma[k]} vs {mb[k]}' for k in sorted(ma.keys() & mb.keys()) if abs(ma[k] - mb[k]) > args.tol]
    wa, wb = sorted(w['code'] for w in a['warnings']), sorted(w['code'] for w in b['warnings'])
    if wa != wb:
        problems.append(f'warnings differ: {wa} vs {wb}')
    for key in ('input_tier', 'rooms', 'connections', 'openings'):
        if json.dumps(a[key], sort_keys=True) != json.dumps(b[key], sort_keys=True):
            problems.append(f'{key} differs')
    print(f'{len(ma)} vs {len(mb)} measurements; {len(problems)} difference(s)')
    for p in problems:
        print('  ' + p)
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
