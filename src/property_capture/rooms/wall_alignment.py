"""Make the two faces of each shared wall parallel.

Each room snaps to its own detected wall line, so the two faces of a shared wall
can differ in direction by a degree or two. Here every face of a shared wall is
rotated about its own midpoint to the wall's common direction (length-weighted
mean of all its faces), which keeps each face's position and the wall thickness
on average. The corners at each end of a rotated face are recomputed as the
intersection with the neighbouring edge, or projected onto the face when the
neighbouring edge is nearly parallel.

A wall's change is kept only if both outlines stay simple, neither room's area
changes by more than max_area_change, and no new overlap with any room appears;
otherwise the wall is skipped with the reason. Two fallbacks are tried first:
a small corner overlap between the wall's own two rooms is trimmed from the
smaller room by exact subtraction, and if aligning all faces of a wall fails,
only its longest face pair is aligned. Both are recorded in the report.
"""
import numpy as np
from shapely.geometry import Polygon
from shapely.geometry.polygon import orient

from .walls import is_simple_polygon


def _area(poly):
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def _edge(poly, i):
    a, b = poly[i], poly[(i + 1) % len(poly)]
    L = np.linalg.norm(b - a)
    return a, (b - a) / L, L


def _intersect(p1, d1, p2, d2):
    A = np.column_stack([d1, -d2])
    if abs(np.linalg.det(A)) < 1e-12:
        return None
    s, _ = np.linalg.solve(A, p2 - p1)
    return p1 + s * d1


def realign_polygon(poly, new_lines, min_corner_deg):
    """Polygon with edges in new_lines {edge index: (point, unit direction)} moved onto those lines."""
    n = len(poly)
    out = poly.copy()
    cos_min = np.cos(np.radians(min_corner_deg))
    for k in range(n):
        prev, cur = (k - 1) % n, k
        if prev not in new_lines and cur not in new_lines:
            continue
        p1, d1 = new_lines.get(prev, _edge(poly, prev)[:2])
        p2, d2 = new_lines.get(cur, _edge(poly, cur)[:2])
        x = _intersect(p1, d1, p2, d2) if abs(d1 @ d2) < cos_min else None
        if x is None:
            # nearly parallel neighbours: project the old corner onto the moved line
            p, d = new_lines[cur] if cur in new_lines else new_lines[prev]
            x = p + ((poly[k] - p) @ d) * d
        out[k] = x
    return out


def _wall_direction(rooms_by_id, wall):
    """Length-weighted mean direction of all faces of a wall, oriented like room A's faces."""
    ra, rb = (rooms_by_id[r] for r in wall['rooms'])
    acc = np.zeros(2)
    for ia, ib in wall['faces']:
        _, da, La = _edge(ra['polygon'], ia)
        _, db, Lb = _edge(rb['polygon'], ib)
        acc += La * da - Lb * db            # B's face runs the opposite way (faces each other)
    return acc / np.linalg.norm(acc)


def _try_align(rooms, by_id, wall, faces, u, cfg):
    """Candidate outlines for aligning `faces` of a wall, or the reasons it is not acceptable.

    A small new overlap between the wall's own two rooms (a corner effect) is trimmed from the
    smaller room by exact subtraction, as in room segmentation; overlap with any other room is
    a reason to skip.
    """
    candidates, rotations = {}, []
    for side, rid in enumerate(wall['rooms']):
        poly = by_id[rid]['polygon']
        new_lines = {}
        for pair in faces:
            i = pair[side]
            a, d, L = _edge(poly, i)
            rotations.append(float(np.degrees(np.arccos(np.clip(abs(d @ u), -1, 1)))))
            new_lines[i] = (a + d * L / 2, u if d @ u > 0 else -u)
        candidates[rid] = realign_polygon(poly, new_lines, cfg['align_min_corner_deg'])
    notes = []

    ra, rb = wall['rooms']
    pa, pb = Polygon(candidates[ra]).buffer(0), Polygon(candidates[rb]).buffer(0)
    mutual = pa.intersection(pb).area - Polygon(by_id[ra]['polygon']).buffer(0).intersection(
        Polygon(by_id[rb]['polygon']).buffer(0)).area
    if cfg['align_max_new_overlap_m2'] < mutual <= cfg['align_trim_overlap_m2']:
        small, big = (ra, pb) if abs(_area(candidates[ra])) < abs(_area(candidates[rb])) else (rb, pa)
        trimmed = Polygon(candidates[small]).buffer(0).difference(big)
        if isinstance(trimmed, Polygon) and not trimmed.is_empty:
            candidates[small] = np.asarray(orient(trimmed, sign=1.0).exterior.coords)[:-1]
            notes.append(f'trimmed {mutual:.3f} m2 corner overlap from {small}')

    reasons = []
    for rid, poly in candidates.items():
        old = by_id[rid]['polygon']
        if not is_simple_polygon(poly):
            reasons.append(f'{rid} outline would self-intersect')
        elif abs(_area(poly) - _area(old)) > cfg['align_max_area_change'] * abs(_area(old)):
            reasons.append(f'{rid} area would change by {abs(_area(poly) / _area(old) - 1):.1%}')
    if not reasons:
        shapes = {r['room_id']: Polygon(candidates.get(r['room_id'], r['polygon'])).buffer(0) for r in rooms}
        before = {r['room_id']: Polygon(r['polygon']).buffer(0) for r in rooms}
        for rid in candidates:
            for other in by_id:
                if other == rid:
                    continue
                grew = shapes[rid].intersection(shapes[other]).area - before[rid].intersection(before[other]).area
                if grew > cfg['align_max_new_overlap_m2']:
                    reasons.append(f'{rid} would overlap {other} by {grew:.3f} m2')
    return candidates, max(rotations), reasons, notes


def align_shared_walls(rooms, walls, cfg):
    """Rotate shared-wall faces to a common direction in place; returns a per-wall report.

    If aligning all faces of a wall is not acceptable, only its longest face pair is aligned.
    """
    by_id = {r['room_id']: r for r in rooms}
    report = []
    for w in walls:
        if not w['faces']:
            continue
        u = _wall_direction(by_id, w)
        candidates, rot, reasons, notes = _try_align(rooms, by_id, w, w['faces'], u, cfg)
        scope = 'all faces'
        if reasons and len(w['faces']) > 1:
            longest = max(w['faces'], key=lambda p: _edge(by_id[w['rooms'][0]]['polygon'], p[0])[2])
            c2, rot2, reasons2, notes2 = _try_align(rooms, by_id, w, [longest], u, cfg)
            if not reasons2:
                candidates, rot, notes = c2, rot2, notes2 + [f'all faces: {"; ".join(sorted(set(reasons)))}']
                reasons, scope = [], 'longest face pair'
        entry = {'shared_wall_id': w['shared_wall_id'], 'rooms': w['rooms'], 'max_rotation_deg': rot,
                 'faces': len(w['faces']), 'scope': scope, 'notes': notes}
        if reasons:
            entry.update({'status': 'skipped', 'reason': '; '.join(sorted(set(reasons)))})
        else:
            for rid, poly in candidates.items():
                entry.setdefault('area_change_m2', {})[rid] = _area(poly) - _area(by_id[rid]['polygon'])
                by_id[rid]['polygon'] = poly
            entry['status'] = 'aligned'
        report.append(entry)
    return report
