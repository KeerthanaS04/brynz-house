#!/usr/bin/env python3
"""Markdown table comparing run directories (metrics.json) side by side.

Usage:
  python scripts/compare_runs.py outputs/run_a outputs/run_b [...] [--out report.md]
"""
import argparse
import json
from pathlib import Path


def _get(d, path):
    for k in path.split('.'):
        if not isinstance(d, dict) or k not in d or d[k] is None:
            return None
        d = d[k]
    return d


def _fmt(v):
    if v is None:
        return '—'
    if isinstance(v, float):
        return f'{v:.3f}'
    return str(v)


def summarize(run_dir):
    m = json.loads((Path(run_dir) / 'metrics.json').read_text(encoding='utf-8'))
    walls = m.get('walls') or []
    hs = m.get('half_split_consistency') or {}
    observed = [w for w in walls if w['evidence'] == 'observed']
    return {
        'polygon method': _get(m, 'room.polygon_method') or 'occupancy',
        'outline edges': _get(m, 'room.walls'),
        'observed edges': len(observed),
        'edges >= 0.5 m': sum(w['length_m'] >= 0.5 for w in walls),
        'outline area (m2)': _get(m, 'room.area_m2'),
        'perimeter (m)': _get(m, 'room.perimeter_m'),
        'wall lines detected': _get(m, 'wall_detection.wall_lines'),
        'contour explained by walls': _get(m, 'wall_detection.contour_explained_fraction'),
        'gap candidates': len(_get(m, 'wall_detection.gap_candidates') or []),
        'corner fills': len(_get(m, 'wall_detection.corner_fills') or []),
        'half-split edges (1st / 2nd)': f"{_fmt(_get(hs, 'first_half.walls'))} / {_fmt(_get(hs, 'second_half.walls'))}",
        'half-split area diff (m2)': _get(hs, 'differences.area_m2_abs_diff'),
        'half-split perimeter diff (m)': _get(hs, 'differences.perimeter_m_abs_diff'),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--out')
    args = ap.parse_args()
    rows = [summarize(r) for r in args.runs]
    names = [Path(r).name for r in args.runs]
    lines = ['| metric | ' + ' | '.join(names) + ' |', '|---|' + '---|' * len(names)]
    for key in rows[0]:
        lines.append(f'| {key} | ' + ' | '.join(_fmt(r[key]) for r in rows) + ' |')
    text = '\n'.join(lines) + '\n'
    if args.out:
        Path(args.out).write_text(text, encoding='utf-8')
    print(text)


if __name__ == '__main__':
    main()
