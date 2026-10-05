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

    args = ap.parse_args(argv)
    if args.command == 'audit':
        sys.argv = ['audit_datasets.py', *args.rest]
        runpy.run_path(str(REPO_ROOT / 'scripts' / 'audit_datasets.py'), run_name='__main__')
        return 0

    from .pipeline import run
    output = args.output or str(REPO_ROOT / 'outputs' /
                                f"run_{datetime.datetime.now(datetime.timezone.utc):%Y%m%dT%H%M%SZ}")
    try:
        return run(args.input, args.config, output, args.gates)
    except Exception as e:
        print(f'pipeline failed: {e}', file=sys.stderr)
        raise


if __name__ == '__main__':
    sys.exit(main())
