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

    r = sub.add_parser('run', help='floor plan from one capture (tier detected from the input)')
    r.add_argument('--input', required=True,
                   help='LiDAR capture ZIP or folder, video file, or folder of room photos')
    r.add_argument('--tier', default='auto', choices=('auto', 'lidar', 'video', 'photo'),
                   help='auto: from the input layout; video on a LiDAR capture uses only its rgb.mp4')
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

    v = sub.add_parser('video', help='video tier: camera motion and sparse structure from an RGB video alone')
    v.add_argument('--input', required=True, help='video file, or a capture folder (only its rgb.mp4 is read)')
    v.add_argument('--config', default=str(REPO_ROOT / 'configs' / 'default.yaml'))
    v.add_argument('--output', help='fresh folder (default outputs/video_<UTC time>)')
    v.add_argument('--set', action='append', default=[], metavar='KEY=VALUE')

    vp = sub.add_parser('video-plan', help='video tier step 2: floor plan from a video run and network depth')
    vp.add_argument('--video-run', required=True, help='folder written by the video command')
    vp.add_argument('--config', default=str(REPO_ROOT / 'configs' / 'default.yaml'))
    vp.add_argument('--gates', default=str(REPO_ROOT / 'configs' / 'evaluation' / 'gates.yaml'))
    vp.add_argument('--output', help='fresh folder (default outputs/video_plan_<UTC time>)')
    vp.add_argument('--set', action='append', default=[], metavar='KEY=VALUE')

    vo = sub.add_parser('video-oracle', help='EVALUATION ONLY: compare a video run with the device poses')
    vo.add_argument('--video-run', required=True, help='folder written by the video or video-plan command')
    vo.add_argument('--capture', required=True, help='capture folder or ZIP the video came from')
    vo.add_argument('--lidar-run', help='video-plan runs only: LiDAR-tier run of the same capture to compare plans')

    args = ap.parse_args(argv)
    if args.command == 'video':
        import json

        import yaml

        from .modalities.video_sfm import run_video
        from .pipeline import apply_overrides
        with open(args.config, encoding='utf-8') as f:
            cfg = apply_overrides(yaml.safe_load(f), args.set)
        src = Path(args.input)
        video_path = src / 'rgb.mp4' if src.is_dir() else src
        out = args.output or str(REPO_ROOT / 'outputs' /
                                 f"video_{datetime.datetime.now(datetime.timezone.utc):%Y%m%dT%H%M%SZ}")
        summary = run_video(video_path, out, cfg['video'])
        print(json.dumps({k: summary[k] for k in ('frame_selection', 'sfm', 'registered_fraction',
                                                  'fraction_in_any_model', 'scale_status')},
                         indent=2, default=str))
        print(f'video run written to {out}')
        return 0
    if args.command == 'video-plan':
        import json

        import yaml

        from .plan_outputs import apply_overrides
        from .video_pipeline import run_video_plan
        with open(args.config, encoding='utf-8') as f:
            cfg = apply_overrides(yaml.safe_load(f), args.set)
        out = args.output or str(REPO_ROOT / 'outputs' /
                                 f"video_plan_{datetime.datetime.now(datetime.timezone.utc):%Y%m%dT%H%M%SZ}")
        vm = run_video_plan(args.video_run, out, cfg, args.gates)
        print(json.dumps({k: vm[k] for k in ('runs', 'links', 'trajectory')} | {'runs': {
            k: v for k, v in vm['runs'].items() if k != 'details'}}, indent=2, default=str))
        print(f'video plan written to {out}')
        return 0
    if args.command == 'video-oracle':
        import json

        from .evaluation.video_oracle import compare, compare_plan
        if (Path(args.video_run) / 'intermediates' / 'video_trajectory.json').exists():
            result = compare_plan(args.video_run, args.capture, REPO_ROOT / 'data' / 'raw', args.lidar_run)
        else:
            result = compare(args.video_run, args.capture, REPO_ROOT / 'data' / 'raw')
        (Path(args.video_run) / 'oracle_comparison.json').write_text(json.dumps(result, indent=2, default=str),
                                                                      encoding='utf-8')
        print(json.dumps(result, indent=2, default=str))
        return 0
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

    from .tiers import UnknownInput, detect_tier, find_video
    try:
        tier = detect_tier(args.input) if args.tier == 'auto' else args.tier
    except (FileNotFoundError, UnknownInput) as e:
        print(f'cannot run: {e}', file=sys.stderr)
        return 2
    output = args.output or str(REPO_ROOT / 'outputs' /
                                f"run_{datetime.datetime.now(datetime.timezone.utc):%Y%m%dT%H%M%SZ}")
    print(f'input tier: {tier}' + (' (detected)' if args.tier == 'auto' else ''))
    if tier == 'photo':
        print('the photo tier is not implemented yet (docs/requirements/traceability.csv REQ-01: no photo data '
              'supplied); no output written', file=sys.stderr)
        return 2
    try:
        if tier == 'video':
            import yaml

            from .plan_outputs import apply_overrides
            from .video_pipeline import run_video_tier
            src = Path(args.input)
            if src.suffix.lower() == '.zip':                  # a LiDAR capture ZIP: use only its video
                from .ingestion.rgbd import resolve_capture_dir
                src = resolve_capture_dir(src, REPO_ROOT / 'data' / 'raw')
            with open(args.config, encoding='utf-8') as f:
                cfg = apply_overrides(yaml.safe_load(f), args.set)
            run_video_tier(find_video(src), output, cfg, args.gates)
        else:
            from .pipeline import run
            run(args.input, args.config, output, args.gates, overrides=args.set)
    except Exception as e:
        print(f'pipeline failed: {e}', file=sys.stderr)
        raise
    print(f'{tier} run written to {output} (floorplan.png, property.json)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
