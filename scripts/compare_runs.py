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
        'sliver removal': _get(m, 'wall_detection.sliver_opening'),
        'outline repaired': _get(m, 'wall_detection.repaired'),
        'gap candidates': len(_get(m, 'wall_detection.gap_candidates') or []),
        'corner fills': len(_get(m, 'wall_detection.corner_fills') or []),
        'rooms': len(m.get('rooms') or []) or None,
        'room areas (m2)': ', '.join(f"{r['area_m2']:.1f}" for r in m.get('rooms') or []) or None,
        'rooms not entered': sum(not r['visited'] for r in m.get('rooms') or []) if m.get('rooms') else None,
        'room outlines falling back': sum(r['outline_method'] != 'wall_snap' for r in m.get('rooms') or [])
        if m.get('rooms') else None,
        'openings (widths m; ? = camera path only)': ', '.join(
            f"{c['opening_width_m']:.2f}" if c['opening_width_m'] is not None else '?'
            for c in m.get('connections') or [] if c['type'] == 'opening') or None,
        'shared walls': sum(c['type'] == 'shared_wall' for c in m.get('connections') or []) if m.get('rooms') else None,
        'ceiling per room (candidate m: status)': '; '.join(
            f"{r['ceiling']['candidate_height_m']:.3f}: " + (
                f"measured, SE {100 * r['ceiling']['standard_error_m']:.2f} cm"
                if r['ceiling']['status'] == 'measured' else r['ceiling']['reason'])
            if r['ceiling'].get('candidate_height_m') is not None else (r['ceiling'].get('reason') or '—')
            for r in m.get('rooms') or []) or None,
        'shared walls (thickness cm; ? = faces not parallel)': (
            f"{_get(m, 'shared_walls.count')} ("
            + ', '.join(f'{100 * t:.0f}' if t is not None else f'~{100 * c:.0f}?'
                        for t, c in zip(_get(m, 'shared_walls.thickness_m') or [],
                                        _get(m, 'shared_walls.candidate_thickness_m') or [])) + ')'
            if _get(m, 'shared_walls') else None),
        'shared-wall gap area (m2) / outline jogs excluded': (
            f"{_fmt(_get(m, 'shared_walls.gap_area_m2'))} / {_get(m, 'shared_walls.excluded_outline_jogs')}"
            if _get(m, 'shared_walls') else None),
        'room overlap before / after clipping (m2)':
            f"{_fmt(_get(m, 'segmentation.overlap_before_m2'))} / {_fmt(_get(m, 'segmentation.overlap_after_m2'))}",
        'outline area not in any room (m2)': _get(m, 'segmentation.outline_area_not_in_rooms_m2'),
        'ceiling heights (m, coverage)': ', '.join(
            f"{r['ceiling']['height_m']:.2f} ({r['ceiling']['coverage']:.0%})" for r in m.get('rooms') or []
            if r['ceiling'].get('height_m') is not None) or None,
        'rgb: video pairing (exact)': (f"{_get(m, 'rgb_alignment.video_pairing.mapping')} "
                                       f"({_get(m, 'rgb_alignment.video_pairing.best_is_exact')})"
                                       if _get(m, 'rgb_alignment.video_pairing') else None),
        'rgb: frames checked / median shift (depth px)': (
            f"{_get(m, 'rgb_alignment.frames')} / {_get(m, 'rgb_alignment.median_shift_depth_px')}"
            if _get(m, 'rgb_alignment.frames') else None),
        'rgb: quadrant deviation (depth px) / status': (
            f"{_fmt(_get(m, 'rgb_alignment.quadrant_max_deviation_depth_px'))} / "
            f"{_get(m, 'rgb_alignment.status') or _get(m, 'rgb_alignment.spatially_aligned')}"
            if _get(m, 'rgb_alignment.frames') else None),
        'rgb: paired frame best (fraction of frames)': _get(m, 'rgb_alignment.temporal_paired_frame_best_fraction'),
        'drift correction decision': _get(m, 'drift.decision'),
        'fused voxels raw / corrected (fewer = sharper)': (
            f"{_get(m, 'drift.ablation.fused_voxels.raw')} / {_get(m, 'drift.ablation.fused_voxels.corrected')}"
            if _get(m, 'drift.ablation.fused_voxels') else None),
        'loops candidates / used / removed after solve': (
            f"{_get(m, 'drift.summary.loop_candidates')} / {_get(m, 'drift.summary.loops_used')} / "
            f"{_get(m, 'drift.summary.loops_removed_after_solve')}"
            if _get(m, 'drift.summary.loop_candidates') is not None else None),
        'drift keyframes accepted / rejected': (f"{_get(m, 'drift.summary.accepted')} / "
                                                f"{_get(m, 'drift.summary.rejected')}"
                                                if _get(m, 'drift.summary') else None),
        'tracking jumps at frames': _get(m, 'drift.summary.jumps_detected_at_frames'),
        'max correction (m / deg)': (f"{_fmt(_get(m, 'drift.summary.max_correction_translation_m'))} / "
                                     f"{_fmt(_get(m, 'drift.summary.max_correction_yaw_deg'))}"
                                     if _get(m, 'drift.summary') else None),
        'revisit residual raw / corrected (m), informative / all pairs': (
            f"{_fmt(_get(m, 'drift.revisit_consistency.raw.median_abs_residual_m'))} / "
            f"{_fmt(_get(m, 'drift.revisit_consistency.corrected.median_abs_residual_m'))}, "
            f"{_get(m, 'drift.revisit_consistency.informative_pairs')} / "
            f"{_get(m, 'drift.revisit_consistency.pairs')}" if _get(m, 'drift.revisit_consistency') else None),
        'revisit within 5 cm raw / corrected': (
            f"{_fmt(_get(m, 'drift.revisit_consistency.raw.mean_within_5cm'))} / "
            f"{_fmt(_get(m, 'drift.revisit_consistency.corrected.mean_within_5cm'))}"
            if _get(m, 'drift.revisit_consistency.raw') else None),
        'ablation rooms total area raw / corrected (m2)': (
            f"{_fmt(_get(m, 'drift.ablation.raw.rooms_total_area_m2'))} / "
            f"{_fmt(_get(m, 'drift.ablation.corrected.rooms_total_area_m2'))}" if _get(m, 'drift.ablation') else None),
        'ablation rooms raw / corrected': (
            f"{_get(m, 'drift.ablation.raw.rooms')} / {_get(m, 'drift.ablation.corrected.rooms')}"
            if _get(m, 'drift.ablation') else None),
        'half-split rooms (1st / 2nd)': f"{_fmt(_get(hs, 'first_half.rooms'))} / {_fmt(_get(hs, 'second_half.rooms'))}",
        'half-split methods (1st / 2nd)': f"{_fmt(_get(hs, 'first_half.polygon_method'))} / "
                                          f"{_fmt(_get(hs, 'second_half.polygon_method'))}",
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
