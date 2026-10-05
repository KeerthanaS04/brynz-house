"""Pair the two faces of walls shared by neighbouring rooms.

Each room's outline snaps to its own face of a wall it shares with a neighbour,
so a shared wall appears as two parallel edges with a gap between them. Two
edges from different rooms are paired as the faces of one wall when they are
nearly parallel, face each other (each room's outward normal points at the
other room), are a plausible wall thickness apart, and overlap along the wall.

Room polygons are counter-clockwise, so the outward normal of edge a->b is
(dy, -dx) / length.
"""
import numpy as np


def _edges(poly):
    for i in range(len(poly)):
        a, b = np.asarray(poly[i], float), np.asarray(poly[(i + 1) % len(poly)], float)
        d = b - a
        L = float(np.linalg.norm(d))
        if L > 1e-9:
            yield i, a, b, d / L, np.array([d[1], -d[0]]) / L, L


def pair_faces(room_a, room_b, cfg):
    """Shared-wall face pairs between two rooms (dicts with room_id and CCW polygon)."""
    cos_par = np.cos(np.radians(cfg['max_angle_deg']))
    out = []
    for i, a0, a1, ta, na, La in _edges(room_a['polygon']):
        for j, b0, b1, tb, nb, Lb in _edges(room_b['polygon']):
            if na @ nb > -cos_par:                         # must face each other
                continue
            # B's endpoints in A's edge frame: along-wall position s, outward distance from A's face g
            sb = np.array([(b0 - a0) @ ta, (b1 - a0) @ ta])
            gb = np.array([(b0 - a0) @ na, (b1 - a0) @ na])
            lo, hi = max(0.0, sb.min()), min(La, sb.max())
            overlap = hi - lo
            if overlap < cfg['min_overlap_m']:
                continue
            # gap at both ends of the overlap, interpolated along B's edge
            gap = np.interp([lo, hi], np.sort(sb), gb[np.argsort(sb)])
            thickness = float(gap.mean())
            if not cfg['min_thickness_m'] <= thickness <= cfg['max_thickness_m']:
                continue
            mid = [a0 + s * ta + (g / 2) * na for s, g in zip((lo, hi), gap)]
            out.append({
                'rooms': [room_a['room_id'], room_b['room_id']], 'edges': [i, j],
                'thickness_m': thickness,
                'thickness_variation_m': float(abs(gap[1] - gap[0])),
                'angle_deg': float(np.degrees(np.arccos(np.clip(-na @ nb, -1, 1)))),
                'overlap_m': float(overlap),
                'gap_area_m2': float(thickness * overlap),
                'centreline': [mid[0].tolist(), mid[1].tolist()],
                'evidence': 'inferred: parallel facing room edges a wall-thickness apart',
            })
    return out


def _weighted_median(values, weights):
    order = np.argsort(values)
    v, w = np.asarray(values)[order], np.asarray(weights)[order]
    return float(v[np.searchsorted(np.cumsum(w), 0.5 * w.sum())])


def group_into_walls(pairs, cfg):
    """Face pairs between the same two rooms with collinear centrelines are one wall.

    A wall has one thickness: the overlap-weighted median of its pairs. Pairs disagreeing with it
    by more than group_tolerance_m are outline jogs (one room snapped to a different feature) and
    are excluded. Thickness is measurable only if the kept pairs' faces are parallel
    (thickness_variation_m <= max_thickness_variation_m).
    """
    cos_par = np.cos(np.radians(cfg['max_angle_deg']))
    walls = []
    for p in pairs:
        c = np.asarray(p['centreline'])
        d = (c[1] - c[0]) / max(np.linalg.norm(c[1] - c[0]), 1e-9)
        for w in walls:
            if w['rooms'] != p['rooms']:
                continue
            n = np.array([-w['dir'][1], w['dir'][0]])
            if abs(d @ w['dir']) >= cos_par and abs((c.mean(axis=0) - w['origin']) @ n) <= cfg['max_thickness_m']:
                w['pairs'].append(p)
                break
        else:
            walls.append({'rooms': p['rooms'], 'dir': d, 'origin': c.mean(axis=0), 'pairs': [p]})

    out = []
    for k, w in enumerate(walls):
        ps = w['pairs']
        t = _weighted_median([p['thickness_m'] for p in ps], [p['overlap_m'] for p in ps])
        kept = [p for p in ps if abs(p['thickness_m'] - t) <= cfg['group_tolerance_m']]
        jogs = [p for p in ps if p not in kept]
        variation = max(p['thickness_variation_m'] for p in kept)
        parallel = variation <= cfg['max_thickness_variation_m']
        out.append({
            'shared_wall_id': f'sw-{k:02d}', 'rooms': w['rooms'],
            'faces': [[p['edges'][0], p['edges'][1]] for p in kept],
            'thickness_m': t if parallel else None,
            'candidate_thickness_m': t,
            'thickness_status': 'measured' if parallel else
                                f'not_measurable: faces not parallel (thickness varies {100 * variation:.1f} cm)',
            'thickness_variation_m': variation,
            'overlap_m': float(sum(p['overlap_m'] for p in kept)),
            'gap_area_m2': float(sum(p['gap_area_m2'] for p in kept)),
            'angle_deg': float(max(p['angle_deg'] for p in kept)),
            'centrelines': [p['centreline'] for p in kept],
            'excluded_outline_jogs': [{'thickness_m': p['thickness_m'], 'overlap_m': p['overlap_m'],
                                       'edges': p['edges']} for p in jogs],
            'evidence': 'inferred: parallel facing room edges a wall-thickness apart',
        })
    return out


def pair_shared_walls(rooms, cfg):
    """Shared walls between every pair of rooms (face pairs grouped into walls)."""
    pairs = []
    for x in range(len(rooms)):
        for y in range(x + 1, len(rooms)):
            pairs += pair_faces(rooms[x], rooms[y], cfg)
    return group_into_walls(pairs, cfg)
