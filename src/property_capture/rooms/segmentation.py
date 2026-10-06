"""Split an occupied floor region into rooms at narrow passages (doorways).

Detected wall segments are drawn into the occupancy grid as barriers; gaps
between segments (doorways) stay open. Free space is shrunk by half the
maximum door width, so doorways and other passages narrower than
door_max_width_m close and each room becomes its own seed. A watershed on the
distance-to-obstacle map then grows the seeds back over all free space, which
puts room boundaries at the narrowest point (the doorway). Open-plan openings
wider than door_max_width_m do not split rooms.

Grid convention (shared with floorplan.extract_room): pixel (row, col) has its
centre at origin + (col + 0.5, row + 0.5) * cell_m in the 2D floor frame.
"""
import cv2
import numpy as np
from shapely.geometry import GeometryCollection, MultiPolygon, Polygon
from shapely.geometry.polygon import orient
from shapely.ops import unary_union


def to_px(p, grid):
    return (np.asarray(p, float) - grid['origin']) / grid['cell_m'] - 0.5


def to_m(px, grid):
    return (np.asarray(px, float) + 0.5) * grid['cell_m'] + grid['origin']


def barrier_image(shape, segments, grid, thickness_m):
    img = np.zeros(shape, np.uint8)
    t = max(1, int(round(thickness_m / grid['cell_m'])))
    for s in segments:
        ends = [s['centre'] + s['s0'] * s['direction'], s['centre'] + s['s1'] * s['direction']]
        a, b = np.rint(to_px(ends, grid)).astype(int)
        cv2.line(img, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), 255, t)
    return img > 0


def _grow_by_level(seeds, dist, free, start_px):
    """Grow seed labels into free space in order of decreasing distance to obstacles.

    At each level t (pixels), rooms expand only into free pixels with dist >= t that touch
    them, so rooms meet where free space is narrowest (doorways) and never through walls or
    outside space. When two rooms reach a pixel in the same step the higher label takes it.
    """
    labels = seeds.astype(np.int32).copy()
    kern = np.ones((3, 3), np.uint8)
    for t in np.arange(np.floor(start_px), -1, -1):
        allowed = free & (dist >= t)
        while True:
            grown = cv2.dilate(labels.astype(np.float32), kern).astype(np.int32)
            update = allowed & (labels == 0) & (grown > 0)
            if not update.any():
                break
            labels[update] = grown[update]
    labels[~free] = 0
    return labels


def _neighbours(labels, k, radius_px):
    """Label -> number of its pixels within radius_px of room k."""
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius_px + 1, 2 * radius_px + 1))
    ring = cv2.dilate((labels == k).astype(np.uint8), kern) > 0
    vals, counts = np.unique(labels[ring & (labels > 0) & (labels != k)], return_counts=True)
    return dict(zip(vals.tolist(), counts.tolist()))


def _merge_small(labels, min_px, radius_px):
    """Rooms smaller than min_px join the neighbour they touch most; isolated ones are dropped."""
    dropped = 0
    while True:
        ids, areas = np.unique(labels[labels > 0], return_counts=True)
        small = [(a, i) for i, a in zip(ids, areas) if a < min_px]
        if not small or len(ids) == 1:
            return labels, dropped
        _, k = min(small)
        nb = _neighbours(labels, k, radius_px)
        if nb:
            labels[labels == k] = max(nb, key=nb.get)
        else:
            dropped += int((labels == k).sum())
            labels[labels == k] = 0


def _extent(mask):
    """Length of a pixel set along its principal axis (pixels)."""
    rr, cc = np.nonzero(mask)
    if len(rr) < 2:
        return float(len(rr))
    P = np.column_stack([cc, rr]).astype(float)
    P -= P.mean(axis=0)
    _, _, vt = np.linalg.svd(P, full_matrices=False)
    s = P @ vt[0]
    return float(s.max() - s.min() + 1)


def segment_rooms(mask, segments, grid, camera2d, cfg):
    """Room labels (0 = not a room) plus per-room stats and room-to-room connections."""
    g = grid['cell_m']
    barrier = barrier_image(mask.shape, segments, grid, cfg['barrier_thickness_m'])
    free = (mask > 0) & ~barrier
    dist = cv2.distanceTransform(free.astype(np.uint8) * 255, cv2.DIST_L2, 5)

    seeds = (dist > cfg['door_max_width_m'] / 2 / g).astype(np.uint8)
    n, comp, stats, _ = cv2.connectedComponentsWithStats(seeds, connectivity=8)
    markers = np.zeros(mask.shape, np.int32)
    k = 0
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] * g * g >= cfg['min_seed_area_m2']:
            k += 1
            markers[comp == i] = k
    if k == 0:
        markers[free] = k = 1
    labels = _grow_by_level(markers, dist, free, cfg['door_max_width_m'] / 2 / g)

    wall_px = max(1, int(round(cfg['barrier_thickness_m'] / g)))
    labels, dropped = _merge_small(labels, cfg['min_room_area_m2'] / (g * g), wall_px + 2)
    ids = sorted(int(i) for i in np.unique(labels[labels > 0]))
    remap = np.zeros(labels.max() + 1, np.int32)
    remap[ids] = np.arange(1, len(ids) + 1)
    labels = remap[labels]

    cam = np.rint(to_px(camera2d, grid)).astype(int)
    inside = (cam[:, 0] >= 0) & (cam[:, 0] < mask.shape[1]) & (cam[:, 1] >= 0) & (cam[:, 1] < mask.shape[0])
    cam_labels = np.zeros(len(cam), np.int32)
    cam_labels[inside] = labels[cam[inside, 1], cam[inside, 0]]

    rooms = []
    for r in range(1, len(ids) + 1):
        frac = float(np.mean(cam_labels == r)) if len(cam_labels) else 0.0
        rooms.append({'label': r, 'area_px': int((labels == r).sum()),
                      'camera_fraction': frac, 'visited': frac >= cfg['visited_min_camera_fraction']})

    # The camera moving from one room to another proves a passage between them.
    transitions = {}
    last, last_i = 0, None
    for i, lab in enumerate(cam_labels):
        if lab == 0:
            continue
        if last and lab != last:
            key = (min(last, lab), max(last, lab))
            transitions.setdefault(key, []).append((camera2d[last_i] + camera2d[i]) / 2)
        last, last_i = lab, i

    connections = []
    across_px = int(round(cfg['max_wall_thickness_m'] / g)) + 2
    for a in range(1, len(ids) + 1):
        A = (labels == a).astype(np.uint8)
        near = cv2.dilate(A, np.ones((5, 5), np.uint8)) > 0
        across = cv2.dilate(A, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * across_px + 1,) * 2)) > 0
        for b in range(a + 1, len(ids) + 1):
            B = labels == b
            contact = across & B
            walked = transitions.get((a, b), [])
            if not contact.any() and not walked:
                continue
            direct = near & B
            opening_w = _extent(direct) * g if direct.any() else 0.0
            boundary = _extent(contact) * g if contact.any() else 0.0
            evidence = []
            if opening_w >= cfg['min_opening_width_m']:
                evidence.append('room regions meet through free space')
            if walked:
                evidence.append(f'camera passed between the rooms {len(walked)} time(s)')
            if evidence:
                kind = 'opening'
            elif boundary >= cfg['min_shared_wall_m']:
                kind, evidence = 'shared_wall', ['room regions meet across a wall']
            else:
                continue
            if kind == 'opening' and opening_w >= cfg['min_opening_width_m']:
                rr, cc = np.nonzero(direct)
                centre = to_m([cc.mean(), rr.mean()], grid)
            elif walked:
                centre = np.mean(walked, axis=0)
            else:
                rr, cc = np.nonzero(contact)
                centre = to_m([cc.mean(), rr.mean()], grid)
            connections.append({
                'rooms': [a, b], 'type': kind,
                'opening_width_m': opening_w if kind == 'opening' and opening_w >= cfg['min_opening_width_m'] else None,
                'shared_boundary_m': boundary, 'camera_transitions': len(walked),
                'location': np.asarray(centre).tolist(), 'evidence': 'inferred: ' + '; '.join(evidence)})
    return labels, rooms, connections, {
        'seeds': k, 'rooms': len(ids), 'dropped_area_m2': dropped * g * g,
        'barrier_px': int(barrier.sum()), 'free_area_m2': float(free.sum() * g * g),
        'camera_transitions': int(sum(len(v) for v in transitions.values())),
    }


def _shape(poly):
    return Polygon(poly).buffer(0) if len(poly) >= 3 else Polygon()


def overlap_area(polys):
    """Total pairwise intersection area (m2) of room polygons."""
    shapes = [_shape(p) for p in polys]
    return float(sum(shapes[i].intersection(shapes[j]).area
                     for i in range(len(shapes)) for j in range(i + 1, len(shapes))))


def resolve_overlaps(rooms, sliver_m=0.01):
    """Make room polygons disjoint by exact polygon subtraction.

    Rooms are processed largest first; each room loses only the part covered by rooms already
    accepted, so every other edge keeps its wall-snapped geometry. Slivers thinner than
    2 * sliver_m left by the subtraction are removed. Polygons with no overlap are untouched.
    Modifies rooms in place (polygon, clipped_area_m2).
    """
    accepted = []
    for r in sorted(rooms, key=lambda r: -r['area_m2']):
        shape = _shape(r['polygon'])
        r['clipped_area_m2'] = 0.0
        if accepted:
            others = unary_union(accepted)
            if shape.intersection(others).area > 1e-9:
                clipped = shape.difference(others)
                clipped = clipped.buffer(-sliver_m, join_style='mitre').buffer(sliver_m, join_style='mitre')
                clipped = clipped.difference(others)
                if isinstance(clipped, (MultiPolygon, GeometryCollection)):
                    parts = [g for g in clipped.geoms if isinstance(g, Polygon)]
                    clipped = max(parts, key=lambda g: g.area) if parts else Polygon()
                if not clipped.is_empty:
                    r['clipped_area_m2'] = float(shape.area - clipped.area)
                    shape = orient(clipped, sign=1.0)
                    r['polygon'] = np.asarray(shape.exterior.coords)[:-1]
        accepted.append(shape)
    return rooms
