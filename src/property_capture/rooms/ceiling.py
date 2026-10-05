"""Per-room ceiling height and whether it is measurable.

A room's ceiling height is reported only when the evidence can support the PDF
tolerance (ceiling height within 1.5 cm per room):
  1. enough independent evidence: ceiling points spread over >= min_cells cells
     (neighbouring points are not independent, cells are),
  2. a single flat ceiling: interquartile range of per-cell levels <= max_flatness_iqr_m
     (stepped ceilings, soffits and beams have no single height),
  3. a precise estimate: standard error of (ceiling level - floor level) <=
     max_standard_error_m, so three standard errors fit inside the tolerance,
  4. plausible for a ceiling: min_height_m <= height <= max_height_m (rejects the
     tops of tall furniture just above the camera).
Levels are the median of per-cell medians; the standard error is a robust
spread of per-cell levels (MAD) over sqrt(cells). With min_cells >= 8, a ceiling
alone cannot exceed the standard-error limit without first failing the flatness
check, so check 3 mainly catches an uneven or poorly seen floor. The floor level is measured
inside the room when it has enough floor cells, otherwise from all floor points.
The standard error is a precision indicator, not a calibrated interval.
"""
import numpy as np


def level_stats(xy, h, cell_m, min_sigma_m=0.0):
    """Robust level of points h at 2D positions xy, using one median per cell."""
    if len(h) == 0:
        return None
    cells = np.floor(xy / cell_m).astype(np.int64)
    _, inv = np.unique(cells, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    order = np.argsort(inv, kind='stable')
    bounds = np.flatnonzero(np.diff(inv[order])) + 1
    meds = np.array([np.median(g) for g in np.split(h[order], bounds)])
    level = float(np.median(meds))
    sigma = max(1.4826 * float(np.median(np.abs(meds - level))), min_sigma_m)
    q1, q3 = np.percentile(meds, [25, 75])
    return {'level_m': level, 'cells': int(len(meds)), 'points': int(len(h)), 'mad_m': sigma,
            'standard_error_m': sigma / np.sqrt(len(meds)), 'iqr_m': float(q3 - q1)}


def assess_ceiling(ceil, floor_room, floor_all, cfg, coverage=None):
    """Combine ceiling and floor level stats into a reported height or a reason it is not measurable."""
    out = {'coverage': coverage, 'height_m': None, 'candidate_height_m': None, 'standard_error_m': None,
           'ceiling_cells': ceil['cells'] if ceil else 0, 'inliers': ceil['points'] if ceil else 0,
           'flatness_iqr_m': ceil['iqr_m'] if ceil else None}
    if ceil is None:
        return {**out, 'status': 'not_measurable', 'reason': 'no ceiling points in this room'}
    if floor_room and floor_room['cells'] >= cfg['min_floor_cells']:
        floor, source = floor_room, 'room'
    else:
        floor, source = floor_all, 'all floor points'
    height = ceil['level_m'] - floor['level_m']
    se = float(np.hypot(ceil['standard_error_m'], floor['standard_error_m']))
    out.update({'candidate_height_m': height, 'standard_error_m': se, 'floor_level_m': floor['level_m'],
                'floor_source': source, 'floor_cells': floor['cells']})
    if ceil['cells'] < cfg['min_cells']:
        reason = f"ceiling seen in {ceil['cells']} cells < {cfg['min_cells']}"
    elif not cfg['min_height_m'] <= height <= cfg['max_height_m']:
        reason = (f'implausible ceiling height {height:.2f} m (outside {cfg["min_height_m"]}-'
                  f'{cfg["max_height_m"]} m): likely a furniture top or another level')
    elif ceil['iqr_m'] > cfg['max_flatness_iqr_m']:
        reason = f"ceiling not flat: cell levels spread {100 * ceil['iqr_m']:.1f} cm (IQR)"
    elif se > cfg['max_standard_error_m']:
        reason = f'height standard error {100 * se:.2f} cm > {100 * cfg["max_standard_error_m"]:.2f} cm'
    else:
        return {**out, 'height_m': height, 'status': 'measured', 'reason': None}
    return {**out, 'status': 'not_measurable', 'reason': reason}
