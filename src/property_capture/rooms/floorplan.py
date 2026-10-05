"""Single-room floor plan from a fused point cloud.

Floor and ceiling come from horizontal plane detection along the gravity
up vector. The room outline is the outer contour of a top-down occupancy
grid of wall-height points plus floor points, morphologically closed and
simplified. Each polygon edge is a wall candidate whose support (wall
points near it) decides observed vs inferred. No Manhattan prior is used.
"""
import cv2
import numpy as np

from ..geometry.planes import find_horizontal_plane
from .walls import (detect_wall_segments, group_collinear, is_simple_polygon, repair_polygon, snap_polygon,
                    tall_cells)


def horizontal_basis(up):
    ref = np.array([1.0, 0.0, 0.0]) if abs(up[0]) < 0.9 else np.array([0.0, 0.0, 1.0])
    e1 = ref - (ref @ up) * up
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(up, e1)


def detect_floor_ceiling(points, up, camera_positions, cfg, rng):
    cam_h = camera_positions @ up
    floor = find_horizontal_plane(points, up, lambda h: h < cam_h.min() - cfg['floor_below_camera_m'],
                                  cfg, rng, cfg['floor_min_fraction'])
    ceiling = find_horizontal_plane(points, up, lambda h: h > cam_h.max() + cfg['ceiling_above_camera_m'],
                                    cfg, rng, cfg['ceiling_min_fraction'])
    return floor, ceiling


def polygon_area(poly):
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def _raster(p, lo, g, shape):
    ij = np.floor((p - lo) / g).astype(int)
    img = np.zeros(shape, np.int32)
    ok = (ij[:, 0] >= 0) & (ij[:, 0] < shape[1]) & (ij[:, 1] >= 0) & (ij[:, 1] < shape[0])
    np.add.at(img, (ij[ok, 1], ij[ok, 0]), 1)
    return img


def extract_room(wall2d, floor2d, cfg, wall_min_points=None, open_m=None):
    """Occupancy contour of wall + floor points. wall_min_points overrides the config count
    (1 for tall-cell centres, which are already one filtered point per cell). open_m removes
    protrusions narrower than open_m (strips seen through gaps) before tracing the contour."""
    g = cfg['grid_m']
    wall_min = cfg['wall_min_points'] if wall_min_points is None else wall_min_points
    allp = np.vstack([wall2d, floor2d])
    lo = allp.min(axis=0) - 0.5
    hi = allp.max(axis=0) + 0.5
    shape = (int(np.ceil((hi[1] - lo[1]) / g)), int(np.ceil((hi[0] - lo[0]) / g)))
    W = _raster(wall2d, lo, g, shape)
    F = _raster(floor2d, lo, g, shape)

    occ = (((W >= wall_min) | (F >= 1)) * 255).astype(np.uint8)
    k = max(1, int(round(cfg['close_m'] / g)))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1))
    occ = cv2.morphologyEx(occ, cv2.MORPH_CLOSE, kernel)

    def largest(img):
        cs, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        return max(cs, key=cv2.contourArea) if cs else None

    c = largest(occ)
    if c is None:
        return None
    opening = 'not_requested'
    if open_m:
        # Opening removes slivers; it is rejected when it would cut off real area
        # (the largest region must keep >= open_min_area_kept of its area).
        r = max(1, int(round(open_m / (2 * g))))
        opened = cv2.morphologyEx(occ, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1)))
        c_open = largest(opened)
        if c_open is not None and cv2.contourArea(c_open) >= cfg['open_min_area_kept'] * cv2.contourArea(c):
            c, opening = c_open, 'applied'
        else:
            opening = 'rejected: would cut off area'
    mask = np.zeros(shape, np.uint8)
    cv2.drawContours(mask, [c], -1, 255, -1)
    approx = cv2.approxPolyDP(c, cfg['simplify_m'] / g, True)[:, 0, :].astype(float)
    poly = (approx + 0.5) * g + lo
    if polygon_area(poly) < 0:
        poly = poly[::-1]
    return {
        **describe_polygon(poly, wall2d, cfg),
        'contour_m': (c[:, 0, :].astype(float) + 0.5) * g + lo,
        'opening': opening,
        'grid': {'origin': lo, 'cell_m': g, 'shape': shape},
        'mask': mask, 'wall_counts': W, 'floor_counts': F,
    }


def describe_polygon(poly, wall2d, cfg):
    walls = [_wall_support(poly[i], poly[(i + 1) % len(poly)], wall2d, cfg) for i in range(len(poly))]
    return {'polygon': poly, 'area_m2': polygon_area(poly),
            'perimeter_m': float(sum(w['length_m'] for w in walls)), 'walls': walls}


def _wall_support(a, b, wall2d, cfg):
    d = b - a
    L = float(np.linalg.norm(d))
    t_hat = d / L
    n_hat = np.array([t_hat[1], -t_hat[0]])
    rel = wall2d - a
    t = rel @ t_hat
    s = rel @ n_hat
    near = (np.abs(s) < cfg['wall_support_m']) & (t >= 0) & (t <= L)
    nb = max(1, int(round(L / cfg['observed_bin_m'])))
    hit = np.zeros(nb, bool)
    hit[np.clip((t[near] / L * nb).astype(int), 0, nb - 1)] = True
    frac = float(hit.mean())
    return {
        'start': a.tolist(), 'end': b.tolist(), 'length_m': L,
        'support_points': int(near.sum()),
        'planarity_rms_m': float(np.sqrt(np.mean(s[near] ** 2))) if near.any() else None,
        'observed_fraction': frac,
        'evidence': 'observed' if frac >= cfg['min_observed_fraction'] else 'inferred',
    }


def build_floorplan(points, up, camera_positions, pcfg, fcfg, rng):
    """Floor/ceiling planes + room polygon in a 2D frame on the floor plane."""
    floor, ceiling = detect_floor_ceiling(points, up, camera_positions, pcfg, rng)
    if floor is None:
        return {'status': 'failed', 'reason': 'no floor plane found below the camera trajectory'}
    n_f = floor['normal']
    h = (points - floor['point']) @ n_f
    cam_h = (camera_positions - floor['point']) @ n_f

    ceiling_height = None
    if ceiling is not None:
        ceiling_height = float(np.median(h[ceiling['inlier_index']]))
    top = (ceiling_height - fcfg['wall_band_below_ceiling_m'] if ceiling_height is not None
           else fcfg['wall_band_max_above_floor_m'])
    band = (h > fcfg['wall_band_above_floor_m']) & (h < top)

    e1, e2 = horizontal_basis(n_f)
    to2d = lambda P: np.stack([P @ e1, P @ e2], axis=1)
    floor_pts = np.abs(h) < fcfg['floor_inlier_m']
    band2d = to2d(points[band])
    method = fcfg['polygon_method']
    wcfg = fcfg['walls']
    # Tall cells are the wall evidence for every method, so edge support is comparable.
    tall2d, tall_stats = tall_cells(band2d, h[band], fcfg['wall_band_above_floor_m'], top, wcfg)
    wall_detection = {'polygon_method': method, **tall_stats}

    if method == 'occupancy':
        room = extract_room(band2d, to2d(points[floor_pts]), fcfg)
        if room is not None:
            room.update(describe_polygon(room['polygon'], tall2d, fcfg))
    elif method == 'wall_snap':
        # Interior extent comes from everything observed (walls, furniture, floor) plus the
        # camera path, which is always inside the room; tall cells only define the walls.
        interior = np.vstack([to2d(points[floor_pts]), to2d(camera_positions)])
        room = extract_room(band2d, interior, fcfg, open_m=wcfg['min_feature_width_m'])
        if room is not None:
            segments = detect_wall_segments(tall2d, wcfg, rng)
            walls = group_collinear(segments, wcfg)
            # An invalid outline (coarse simplification crossing a nearby edge, or both sides of
            # a thin notch snapped onto one wall) is first repaired by re-tracing its outer
            # boundary, then retried with finer simplification, before falling back.
            eps = wcfg['free_simplify_m']
            for _ in range(3):
                snapped = snap_polygon(room['contour_m'], walls, {**wcfg, 'free_simplify_m': eps})
                if snapped is not None and not snapped['simple']:
                    fixed = repair_polygon(snapped['polygon'])
                    if fixed is not None and len(fixed) >= 3 and is_simple_polygon(fixed):
                        snapped = {**snapped, 'polygon': fixed, 'simple': True}
                        wall_detection['repaired'] = True
                if snapped is not None and snapped['simple']:
                    break
                eps /= 2
            wall_detection['free_simplify_used_m'] = eps
            wall_detection['sliver_opening'] = room['opening']
            wall_detection.update({
                'segments': len(segments), 'wall_lines': len(walls),
                'wall_lines_geometry': [{'start': (w['centre'] + w['extent'][0] * w['direction']).tolist(),
                                         'end': (w['centre'] + w['extent'][1] * w['direction']).tolist(),
                                         'observed_length_m': w['observed_length_m'],
                                         'segments': len(w['segments'])} for w in walls],
            })
            if snapped is not None and snapped['simple']:
                wall_detection.update({k: snapped[k] for k in ('contour_explained_fraction', 'runs', 'wall_runs',
                                                               'gap_candidates', 'corner_fills')})
                room.update(describe_polygon(snapped['polygon'], tall2d, fcfg))
            else:
                wall_detection['polygon_method'] = 'occupancy_fallback'
                wall_detection['fallback_reason'] = ('snapped polygon self-intersects' if snapped is not None
                                                     else 'snapped polygon degenerate')
    else:
        raise ValueError(f'unknown floorplan.polygon_method {method!r}')
    if room is None:
        return {'status': 'failed', 'reason': 'no room contour found'}

    ceiling_coverage = None
    if ceiling is not None:
        C = _raster(to2d(points[ceiling['inlier_index']]), room['grid']['origin'], room['grid']['cell_m'],
                    room['grid']['shape'])
        k = max(1, int(round(fcfg['close_m'] / fcfg['grid_m'])))
        C = cv2.morphologyEx(((C > 0) * 255).astype(np.uint8), cv2.MORPH_CLOSE,
                             cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1)))
        ceiling_coverage = float(((C > 0) & (room['mask'] > 0)).sum() / max((room['mask'] > 0).sum(), 1))

    return {
        'status': 'ok',
        'floor': floor, 'ceiling': ceiling,
        'ceiling_height_m': ceiling_height,
        'ceiling_coverage': ceiling_coverage,
        'camera_height_above_floor_m': {'min': float(cam_h.min()), 'median': float(np.median(cam_h)),
                                        'max': float(cam_h.max())},
        'basis': {'e1': e1, 'e2': e2, 'up': n_f, 'origin': floor['point']},
        'camera_2d': to2d(camera_positions),
        'room': room,
        'wall_detection': wall_detection,
        'wall_band_points': int(band.sum()),
        'floor_inlier_points': int(floor_pts.sum()),
    }
