"""Vertical wall detection and wall-snapped room polygons, in the 2D floor frame.

1. tall_cells: keep 2D cells whose points cover a large fraction of the
   wall-height band (walls, tall cabinets); low furniture is dropped.
2. detect_wall_segments: sequential RANSAC lines on tall-cell centres,
   split into segments at gaps (doors, windows).
3. group_collinear: segments on the same line form one wall.
4. snap_polygon: each point of the room's occupancy contour is assigned to
   the nearest wall it lies on; runs of the same wall become one edge and
   adjacent walls meet at their intersection. Contour stretches that no
   wall explains are kept as simplified free edges.
No Manhattan or parallelism prior is applied.
"""
import cv2
import numpy as np


def tall_cells(p2, h, band_lo, band_hi, cfg):
    """Centres of cells whose points cover >= tall_fraction of the height band."""
    c, hb = cfg['cell_m'], cfg['height_bin_m']
    nb = max(1, int(np.ceil((band_hi - band_lo) / hb)))
    ij = np.floor(p2 / c).astype(np.int64)
    hk = np.clip(((h - band_lo) / hb).astype(int), 0, nb - 1)
    cells, inv = np.unique(ij, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    occ = np.zeros((len(cells), nb), bool)
    occ[inv, hk] = True
    frac = occ.mean(axis=1)
    keep = frac >= cfg['tall_fraction']
    return (cells[keep] + 0.5) * c, {'cells': int(len(cells)), 'tall_cells': int(keep.sum()), 'height_bins': nb}


def _fit_line(P):
    c = P.mean(axis=0)
    _, _, vt = np.linalg.svd(P - c, full_matrices=False)
    return c, vt[0], vt[1]


def detect_wall_segments(P, cfg, rng):
    """Sequential RANSAC lines on 2D points, each split into gap-free segments."""
    remaining = np.arange(len(P))
    r0, r1 = cfg['sample_radius_m']
    thr = cfg['ransac_threshold_m']
    segments = []
    for _ in range(cfg['max_lines']):
        if len(remaining) < cfg['min_inliers']:
            break
        Q = P[remaining]
        best_cnt, best_inl = 0, None
        for _ in range(cfg['ransac_iterations']):
            a = Q[rng.integers(len(Q))]
            dist = np.linalg.norm(Q - a, axis=1)
            cand = np.flatnonzero((dist > r0) & (dist < r1))
            if len(cand) == 0:
                continue
            d = Q[cand[rng.integers(len(cand))]] - a
            n = np.array([-d[1], d[0]]) / np.linalg.norm(d)
            inl = np.abs((Q - a) @ n) < thr
            cnt = int(inl.sum())
            if cnt > best_cnt:
                best_cnt, best_inl = cnt, inl
        if best_inl is None or best_cnt < cfg['min_inliers']:
            break
        c, t, n = _fit_line(Q[best_inl])
        inl = np.abs((Q - c) @ n) < thr
        members_all = remaining[inl]
        s = (Q[inl] - c) @ t
        order = np.argsort(s)
        s_sorted = s[order]
        breaks = np.flatnonzero(np.diff(s_sorted) > cfg['max_gap_m'])
        for a_, b_ in zip(np.r_[0, breaks + 1], np.r_[breaks, len(s_sorted) - 1]):
            members = members_all[order[a_:b_ + 1]]
            if s_sorted[b_] - s_sorted[a_] < cfg['min_segment_m'] or len(members) < cfg['min_segment_points']:
                continue
            cs, ts, ns = _fit_line(P[members])
            ss = (P[members] - cs) @ ts
            segments.append({'centre': cs, 'direction': ts, 'normal': ns,
                             's0': float(ss.min()), 's1': float(ss.max()),
                             'length_m': float(ss.max() - ss.min()), 'points': int(len(members)),
                             'rms_m': float(np.sqrt(np.mean(((P[members] - cs) @ ns) ** 2)))})
        remaining = remaining[~inl]
    return segments


def group_collinear(segments, cfg):
    """Group segments lying on one line; the longest segment defines the wall line."""
    max_cos = np.cos(np.radians(cfg['merge_angle_deg']))
    walls = []
    for seg in sorted(segments, key=lambda s: -s['length_m']):
        for w in walls:
            if abs(seg['direction'] @ w['direction']) >= max_cos and \
                    abs((seg['centre'] - w['centre']) @ w['normal']) < cfg['merge_offset_m']:
                w['segments'].append(seg)
                break
        else:
            walls.append({'centre': seg['centre'], 'direction': seg['direction'],
                          'normal': seg['normal'], 'segments': [seg]})
    for w in walls:
        ends = []
        for seg in w['segments']:
            for s in (seg['s0'], seg['s1']):
                ends.append((seg['centre'] + s * seg['direction'] - w['centre']) @ w['direction'])
        w['extent'] = [float(min(ends)), float(max(ends))]
        w['observed_length_m'] = float(sum(seg['length_m'] for seg in w['segments']))
    return walls


def _runs(label):
    n = len(label)
    b = np.flatnonzero(label != np.roll(label, 1))
    if len(b) == 0:
        return [(int(label[0]), np.arange(n))]
    runs = []
    for k in range(len(b)):
        s, e = b[k], b[(k + 1) % len(b)]
        runs.append((int(label[s]), np.arange(s, e if e > s else e + n) % n))
    return runs


def _project(p, w):
    return w['centre'] + ((p - w['centre']) @ w['direction']) * w['direction']


def _intersect(w1, w2):
    A = np.column_stack([w1['direction'], -w2['direction']])
    if abs(np.linalg.det(A)) < 1e-9:
        return None
    a, _ = np.linalg.solve(A, w2['centre'] - w1['centre'])
    return w1['centre'] + a * w1['direction']


def _segments_intersect(p1, p2, q1, q2):
    def orient(a, b, c):
        return np.sign((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
    return (orient(p1, p2, q1) * orient(p1, p2, q2) < 0) and (orient(q1, q2, p1) * orient(q1, q2, p2) < 0)


def is_simple_polygon(poly):
    """True when no two non-adjacent edges cross."""
    n = len(poly)
    if n < 3:
        return False
    for i in range(n):
        for j in range(i + 1, n):
            if j == i + 1 or (i == 0 and j == n - 1):
                continue
            if _segments_intersect(poly[i], poly[(i + 1) % n], poly[j], poly[(j + 1) % n]):
                return False
    return True


def _clean(poly, min_edge, collinear_m):
    pts = [p for p in poly]
    changed = True
    while changed and len(pts) > 3:
        changed = False
        for i in range(len(pts)):
            a, b, c = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
            ac = c - a
            L = np.linalg.norm(ac)
            short = np.linalg.norm(b - a) < min_edge
            straight = L > 0 and abs(ac[0] * (b - a)[1] - ac[1] * (b - a)[0]) / L < collinear_m
            if short or straight:
                pts.pop(i)
                changed = True
                break
    return np.array(pts)


def _merge_adjacent(label, walls, step, cfg):
    """Consecutive runs on near-parallel, nearby wall lines are one wall seen as several
    RANSAC lines (finish, small drift): the shorter run takes the longer run's line.
    Modifies label in place and returns the new runs."""
    max_cos = np.cos(np.radians(cfg['adjacent_merge_angle_deg']))
    runs = _runs(label)
    merged = True
    while merged and len(runs) >= 2:
        merged = False
        for j in range(len(runs)):
            (la, ia), (lb, ib) = runs[j], runs[(j + 1) % len(runs)]
            if la < 0 or lb < 0 or la == lb:
                continue
            wa, wb = walls[la], walls[lb]
            if abs(wa['direction'] @ wb['direction']) < max_cos or \
                    abs((wb['centre'] - wa['centre']) @ wa['normal']) > cfg['adjacent_merge_offset_m']:
                continue
            keep, drop = (la, ib) if step[ia].sum() >= step[ib].sum() else (lb, ia)
            label[drop] = keep
            runs = _runs(label)
            merged = True
            break
    return runs


def _bridge_gaps(label, contour, walls):
    """A free run between two runs of the same wall is a gap in that wall (door, window,
    unobserved stretch): the boundary follows the wall line and the gap is recorded.
    Modifies label in place and returns (runs, gaps)."""
    gaps = []
    runs = _runs(label)
    bridged = True
    while bridged and len(runs) >= 3:
        bridged = False
        for j, (lab, idx) in enumerate(runs):
            prev_lab, next_lab = runs[j - 1][0], runs[(j + 1) % len(runs)][0]
            if lab < 0 and prev_lab >= 0 and prev_lab == next_lab:
                w = walls[prev_lab]
                a, b = _project(contour[idx[0]], w), _project(contour[idx[-1]], w)
                gaps.append({'wall_line': int(prev_lab), 'start': a.tolist(), 'end': b.tolist(),
                             'width_m': float(np.linalg.norm(b - a)),
                             'evidence': 'inferred: boundary gap within one wall line'})
                label[idx] = prev_lab
                runs = _runs(label)
                bridged = True
                break
    return runs, gaps


def _fill_corners(label, contour, walls, cfg):
    """A free run between two non-parallel walls, lying entirely near their intersection,
    is an occluded or cluttered corner: the walls are extended to meet. Modifies label in
    place and returns (runs, fills)."""
    min_cos = np.cos(np.radians(cfg['corner_min_angle_deg']))
    fills = []
    runs = _runs(label)
    filled = True
    while filled and len(runs) >= 3:
        filled = False
        for j, (lab, idx) in enumerate(runs):
            a, b = runs[j - 1][0], runs[(j + 1) % len(runs)][0]
            if lab >= 0 or a < 0 or b < 0 or a == b:
                continue
            if abs(walls[a]['direction'] @ walls[b]['direction']) > min_cos:
                continue
            x = _intersect(walls[a], walls[b])
            if x is None or np.max(np.linalg.norm(contour[idx] - x, axis=1)) > cfg['corner_fill_radius_m']:
                continue
            fills.append({'corner': x.tolist(), 'walls': [int(a), int(b)],
                          'evidence': 'inferred: walls extended to meet across an unexplained corner'})
            label[idx] = a
            runs = _runs(label)
            filled = True
            break
    return runs, fills


def snap_polygon(contour, walls, cfg):
    """Wall-snapped polygon from a closed contour (N, 2). Returns None if invalid."""
    n = len(contour)
    label = np.full(n, -1)
    best = np.full(n, np.inf)
    m = cfg['extent_margin_m']
    for k, w in enumerate(walls):
        rel = contour - w['centre']
        d = np.abs(rel @ w['normal'])
        s = rel @ w['direction']
        ok = (d < cfg['snap_distance_m']) & (s > w['extent'][0] - m) & (s < w['extent'][1] + m) & (d < best)
        label[ok] = k
        best[ok] = d[ok]
    explained = float((label >= 0).mean())

    step = np.linalg.norm(np.roll(contour, -1, axis=0) - contour, axis=1)
    runs = _runs(label)
    while len(runs) > 1:
        lens = [step[idx].sum() for _, idx in runs]
        j = int(np.argmin(lens))
        if lens[j] >= cfg['min_run_m']:
            break
        prev, nxt = runs[j - 1], runs[(j + 1) % len(runs)]
        absorb = prev if step[prev[1]].sum() >= step[nxt[1]].sum() else nxt
        label[runs[j][1]] = absorb[0]
        runs = _runs(label)

    runs = _merge_adjacent(label, walls, step, cfg)
    runs, gaps = _bridge_gaps(label, contour, walls)
    runs = _merge_adjacent(label, walls, step, cfg)
    runs, corner_fills = _fill_corners(label, contour, walls, cfg)

    run_verts = []
    for lab, idx in runs:
        pts = contour[idx]
        if lab >= 0:
            w = walls[lab]
            run_verts.append([_project(pts[0], w), _project(pts[-1], w)])
        else:
            simp = cv2.approxPolyDP(pts.astype(np.float32).reshape(-1, 1, 2), cfg['free_simplify_m'], False)
            run_verts.append([p for p in simp[:, 0, :].astype(float)])

    min_cos = np.cos(np.radians(cfg['corner_min_angle_deg']))
    for i in range(len(runs)):
        j = (i + 1) % len(runs)
        a, b = runs[i][0], runs[j][0]
        if a < 0 or b < 0 or a == b or len(runs) < 2:
            continue
        wa, wb = walls[a], walls[b]
        if abs(wa['direction'] @ wb['direction']) > min_cos:
            continue
        x = _intersect(wa, wb)
        if x is None:
            continue
        if np.linalg.norm(x - run_verts[i][-1]) <= cfg['max_corner_shift_m'] and \
                np.linalg.norm(x - run_verts[j][0]) <= cfg['max_corner_shift_m']:
            run_verts[i][-1] = x
            run_verts[j][0] = x

    poly = _clean(np.array([p for rv in run_verts for p in rv]), cfg['min_edge_m'], cfg['collinear_m'])
    if len(poly) < 3:
        return None
    area = 0.5 * float(np.dot(poly[:, 0], np.roll(poly[:, 1], -1)) - np.dot(poly[:, 1], np.roll(poly[:, 0], -1)))
    if area < 0:
        poly = poly[::-1]
    return {'polygon': poly, 'simple': is_simple_polygon(poly), 'contour_explained_fraction': explained,
            'runs': len(runs), 'wall_runs': sum(lab >= 0 for lab, _ in runs), 'gap_candidates': gaps,
            'corner_fills': corner_fills}
