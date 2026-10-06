"""Command-line entry point: python -m property_capture <command> ..."""
import argparse
import datetime
import runpy
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def main(argv=None):
    ap = argparse.ArgumentParser(prog='property_capture')
    sub = ap.add_subparsers(dest='command', required=True)

    a = sub.add_parser('audit', help='audit supplied capture ZIPs (scripts/audit_datasets.py)')
    a.add_argument('rest', nargs=argparse.REMAINDER, help='arguments passed to the audit script')

    r = sub.add_parser('run', help='run the RGB-D baseline on one capture')
    r.add_argument('--input', required=True, help='capture ZIP or extracted capture directory')
    r.add_argument('--config', default=str(REPO_ROOT / 'configs' / 'default.yaml'))
    r.add_argument('--gates', default=str(REPO_ROOT / 'configs' / 'evaluation' / 'gates.yaml'))
    r.add_argument('--output', help='fresh run directory (default outputs/run_<UTC time>)')
    r.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                   help='override a config value, e.g. --set floorplan.polygon_method=occupancy (repeatable)')

    e = sub.add_parser('evaluate', help='score runs against laser/tape references (docs/benchmark_protocol.md)')
    e.add_argument('--references', required=True, help='reference CSV (docs/templates/references_template.csv)')
    e.add_argument('--run', action='append', required=True, metavar='CAPTURE_ID=RUN_DIR',
                   help='run folder for a capture_id in the references (repeatable)')
    e.add_argument('--gates', default=str(REPO_ROOT / 'configs' / 'evaluation' / 'gates.yaml'))
    e.add_argument('--output', help='fresh folder (default <first run>/evaluation_<UTC time>)')
    e.add_argument('--strict', action='store_true', help='exit 1 if any gate fails')

    args = ap.parse_args(argv)
    if args.command == 'audit':
        sys.argv = ['audit_datasets.py', *args.rest]
        runpy.run_path(str(REPO_ROOT / 'scripts' / 'audit_datasets.py'), run_name='__main__')
        return 0
    if args.command == 'evaluate':
        from .evaluation.evaluate import evaluate, write_outputs
        runs = {}
        for item in args.run:
            cap, sep, path = item.partition('=')
            if not sep:
                ap.error(f'--run {item!r} is not CAPTURE_ID=RUN_DIR')
            runs[cap] = path
        rows, ph, results, summary = evaluate(args.references, runs, args.gates)
        out = args.output or str(Path(next(iter(runs.values()))) /
                                 f"evaluation_{datetime.datetime.now(datetime.timezone.utc):%Y%m%dT%H%M%SZ}")
        write_outputs(out, rows, ph, results, summary)
        for name, g in results.items():
            print(f"{name:40s} {g['status']}")
        print(f'evaluation written to {out}')
        return 1 if args.strict and any(g['status'] == 'fail' for g in results.values()) else 0

    from .pipeline import run
    output = args.output or str(REPO_ROOT / 'outputs' /
                                f"run_{datetime.datetime.now(datetime.timezone.utc):%Y%m%dT%H%M%SZ}")
    try:
        return run(args.input, args.config, output, args.gates, overrides=args.set)
    except Exception as e:
        print(f'pipeline failed: {e}', file=sys.stderr)
        raise


if __name__ == '__main__':
    sys.exit(main())
