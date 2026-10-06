"""Regenerate the current evidence from the raw capture ZIPs into one fresh folder (DEL-04).

    python scripts/reproduce.py                          # audit + every capture, both tiers
    python scripts/reproduce.py --tiers lidar            # LiDAR tier only (minutes, no GPU)
    python scripts/reproduce.py --captures single_room   # one capture

Steps come from configs/reproduce.yaml. Every step runs as its own process; its log is saved.
<out>/REPRODUCTION.md records the environment, input hashes, each command with exit code and
time, and the headline numbers read back from the outputs. Older report folders were made at
earlier commits (each run_info.json names its commit); this regenerates today's numbers.
"""
import argparse
import datetime
import json
import platform
import subprocess
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from property_capture.ingestion.rgbd import resolve_capture_dir  # noqa: E402
from property_capture.reporting.provenance import git_info, sha256_file, sha256_tree  # noqa: E402


def _versions():
    out = {'python': sys.version.split()[0], 'platform': platform.platform()}
    for name in ('numpy', 'cv2', 'shapely', 'pycolmap', 'torch', 'transformers'):
        try:
            out[name] = __import__(name).__version__
        except Exception:
            out[name] = 'not installed'
    try:
        import torch
        out['gpu'] = torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'none (CPU)'
    except Exception:
        out['gpu'] = 'unknown'
    return out


def _load(path):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return None


def _lidar_summary(run):
    prop, metrics = _load(run / 'property.json'), _load(run / 'metrics.json')
    if not prop:
        return None
    ms = {m['id']: m for m in prop['measurements']}
    return {
        'rooms': len(prop['rooms']),
        'total_area_m2': round(sum(ms[r['floor_area_measurement_id']]['value'] for r in prop['rooms']), 2),
        'walls': sum(len(r['walls']) for r in prop['rooms']),
        'ceilings_measured': sum(r['ceiling']['status'] == 'measured' for r in prop['rooms']),
        'opening_widths_measured': sum(m['quantity'] == 'opening_width' for m in prop['measurements']),
        'drift_correction': (metrics or {}).get('drift', {}).get('decision'),
    }


def _video_summary(run):
    metrics, oracle = _load(run / 'metrics.json'), _load(run / 'oracle_comparison.json')
    lid = _lidar_summary(run)
    if not metrics or not lid:
        return None
    v = metrics['video']
    out = {'frames': v['frames_selected'], 'parts': len(v['trajectory']['components']),
           'main_part_frames': v['trajectory']['main_component_frames'], 'plan_area_m2': lid['total_area_m2']}
    if oracle and oracle.get('status') == 'evaluated':
        main = next((c for c in oracle['components'].values() if c['main']), None)
        if main:
            out['main_part_scale_error_percent'] = round(main['similarity']['scale_error_percent'], 1)
            out['main_part_rigid_error_m'] = round(main['rigid']['ate_rmse_m'], 3)
        cd = oracle.get('corrected_depth_vs_lidar')
        if cd:
            out['corrected_depth_over_lidar'] = round(cd['median_ratio'], 3)
        pl = oracle.get('plan_vs_lidar_plan')
        if pl:
            out['plan_area_over_lidar'] = round(pl['video']['total_area_m2'] / pl['lidar']['total_area_m2'], 3)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--manifest', default=str(ROOT / 'configs' / 'reproduce.yaml'))
    ap.add_argument('--output', help='fresh folder (default outputs/repro_<UTC time>)')
    ap.add_argument('--tiers', nargs='+', default=['lidar', 'video'], choices=['lidar', 'video'])
    ap.add_argument('--captures', nargs='+', help='subset of the manifest capture names')
    args = ap.parse_args(argv)
    manifest = yaml.safe_load(Path(args.manifest).read_text(encoding='utf-8'))
    out = Path(args.output or ROOT / 'outputs' /
               f"repro_{datetime.datetime.now(datetime.timezone.utc):%Y%m%dT%H%M%SZ}")
    if out.exists() and any(out.iterdir()):
        sys.exit(f'{out} exists and is not empty; runs never overwrite')
    (out / 'logs').mkdir(parents=True)
    captures = {k: v for k, v in manifest['captures'].items() if not args.captures or k in args.captures}

    steps = []
    if manifest.get('audit'):
        steps.append(('audit', ['scripts/audit_datasets.py', '--zip', *[c['zip'] for c in captures.values()],
                                '--output', str(out / 'audit'), '--data-dictionary',
                                str(out / 'audit' / 'data_dictionary.md')]))
    for name, c in captures.items():
        lidar, video = out / name / 'lidar', out / name / 'video'
        cap = resolve_capture_dir(ROOT / c['zip'], ROOT / 'data' / 'raw')
        sets = lambda key: [a for s in c.get(key) or [] for a in ('--set', s)]  # noqa: E731
        if 'lidar' in args.tiers:
            steps.append((f'{name}_lidar', ['-m', 'property_capture', 'run', '--tier', 'lidar', '--input',
                                            c['zip'], '--output', str(lidar), *sets('lidar_set')]))
        if 'video' in args.tiers:
            steps.append((f'{name}_video', ['-m', 'property_capture', 'run', '--tier', 'video', '--input',
                                            c['zip'], '--output', str(video), *sets('video_set')]))
            steps.append((f'{name}_video_sfm_oracle', ['-m', 'property_capture', 'video-oracle', '--video-run',
                                                       str(video / 'video_sfm'), '--capture', str(cap)]))
            steps.append((f'{name}_video_oracle', ['-m', 'property_capture', 'video-oracle', '--video-run',
                                                   str(video), '--capture', str(cap)]
                          + (['--lidar-run', str(lidar)] if 'lidar' in args.tiers else [])))

    results = []
    for name, cmd in steps:
        t0 = time.time()
        with open(out / 'logs' / f'{name}.log', 'w', encoding='utf-8') as log:
            rc = subprocess.run([sys.executable, *cmd], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT).returncode
        results.append({'step': name, 'command': ' '.join(['python', *cmd]), 'exit_code': rc,
                        'seconds': round(time.time() - t0, 1)})
        print(f"{name:32s} exit {rc}  {results[-1]['seconds']:7.1f} s")

    summary = {
        'finished_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'git': git_info(ROOT), 'environment': _versions(),
        'inputs': {c['zip']: sha256_file(ROOT / c['zip']) for c in captures.values()},
        'weights_tree_sha256': {p.name: sha256_tree(p) for p in sorted((ROOT / 'weights').glob('*')) if p.is_dir()},
        'steps': results,
        'captures': {name: {'lidar': _lidar_summary(out / name / 'lidar'),
                            'video': _video_summary(out / name / 'video')} for name in captures},
    }
    (out / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')

    L = ['# Reproduction run', '',
         f"Commit `{summary['git']['commit']}`" + (' (uncommitted changes present)' if summary['git']['dirty'] else '')
         + f", finished {summary['finished_utc']}.", '',
         '## Environment', '', *[f'- {k}: {v}' for k, v in summary['environment'].items()], '',
         '## Inputs (SHA-256)', '', *[f'- `{k}`: `{v}`' for k, v in summary['inputs'].items()],
         *[f'- weights `{k}`: `{v}`' for k, v in summary['weights_tree_sha256'].items()], '',
         '## Steps', '', '| step | exit | seconds | command |', '|---|---|---|---|',
         *[f"| {r['step']} | {r['exit_code']} | {r['seconds']} | `{r['command']}` |" for r in results], '',
         '## Headline numbers', '']
    for name, s in summary['captures'].items():
        L.append(f'### {name}')
        for tier in ('lidar', 'video'):
            if s[tier]:
                L.append(f"- {tier}: " + ', '.join(f'{k} {v}' for k, v in s[tier].items()))
        L.append('')
    L.append('All measurements are uncalibrated (no reference measurements, assumptions.md B-21); the video '
             'oracle reads device data and is evaluation only.')
    (out / 'REPRODUCTION.md').write_text('\n'.join(L) + '\n', encoding='utf-8')
    print(f'reproduction written to {out / "REPRODUCTION.md"}')
    return 1 if any(r['exit_code'] for r in results) else 0


if __name__ == '__main__':
    sys.exit(main())
