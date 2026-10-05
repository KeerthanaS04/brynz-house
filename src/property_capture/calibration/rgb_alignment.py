"""Check that depth maps line up with RGB frames, spatially and in time.

Assumptions under test (assumptions.md): B-15, depth intrinsics are the RGB
intrinsics scaled to the depth resolution (depth is a resampled, aligned view
of RGB); and B-07, video frame i pairs with depth/odometry frame i + offset.

Spatial: in low-motion frames, depth discontinuities (edges of objects at
different distances) should coincide with image edges. For each frame, the
shift of the RGB edge-strength image that maximizes edge strength at depth
edges is found on a sub-pixel grid. Aligned data peaks at zero shift. The same
is done per image quadrant: a scale or rotation error between depth and RGB
would make the quadrants disagree.

Temporal: the same score is computed with the video frames before and after
the paired one. The paired frame should align best when the camera moves.
"""
import cv2
import numpy as np


def depth_edges(depth_m, conf, size, cfg):
    """Boolean edge mask at `size` (w, h) where depth jumps (relative) between neighbours."""
    valid = (conf >= cfg['confidence_min']) & (depth_m > 0)
    d = cv2.resize(np.where(valid, depth_m, 0).astype(np.float32), size, interpolation=cv2.INTER_NEAREST)
    v = cv2.resize(valid.astype(np.uint8), size, interpolation=cv2.INTER_NEAREST) > 0
    logd = np.log(np.where(v, d, 1.0))
    gx = cv2.Sobel(logd, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(logd, cv2.CV_32F, 0, 1, ksize=3)
    mag = np.hypot(gx, gy)
    inner = cv2.erode(v.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
    if not inner.any():
        return inner
    thr = max(np.percentile(mag[inner], cfg['edge_percentile']), cfg['edge_min_log_jump'])
    return inner & (mag >= thr)


def image_gradient(gray):
    g = gray.astype(np.float32)
    mag = np.hypot(cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3), cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3))
    return mag / max(float(mag.mean()), 1e-6)


def shift_scores(edges, grad, shifts):
    """Mean image-gradient strength at depth-edge pixels after shifting the image by each (dx, dy)."""
    out = np.zeros(len(shifts))
    h, w = grad.shape
    for k, (dx, dy) in enumerate(shifts):
        M = np.float32([[1, 0, -dx], [0, 1, -dy]])            # sample grad at p + (dx, dy)
        g = cv2.warpAffine(grad, M, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
        out[k] = g[edges].mean() if edges.any() else np.nan
    return out


def shift_grid(max_px, step):
    r = np.arange(-max_px, max_px + 1e-9, step)
    return np.array([(dx, dy) for dy in r for dx in r])


def _parabola_peak(sm, s0, sp):
    """Offset (in steps) of the vertex of a parabola through three equally spaced samples."""
    den = sm - 2 * s0 + sp
    return 0.0 if den >= 0 else float(np.clip(0.5 * (sm - sp) / den, -0.5, 0.5))


def best_shift(edges, grad, shifts):
    """Grid peak refined to sub-step precision by a parabola through the neighbouring scores in
    x and in y (the edge masks are pixel-quantized, so the raw grid peak sits on whole pixels)."""
    s = shift_scores(edges, grad, shifts)
    best, k = refine_peak(shifts, s)
    return best, s[k], s


def refine_peak(shifts, s):
    k = int(np.nanargmax(s))
    best = shifts[k].astype(float).copy()
    step = float(np.min(np.diff(np.unique(shifts[:, 0])))) if len(np.unique(shifts[:, 0])) > 1 else 0.0
    if step:
        lookup = {tuple(np.round(p, 6)): v for p, v in zip(shifts, s)}
        for axis in (0, 1):
            lo, hi = best.copy(), best.copy()
            lo[axis] -= step
            hi[axis] += step
            sm, sp = lookup.get(tuple(np.round(lo, 6))), lookup.get(tuple(np.round(hi, 6)))
            if sm is not None and sp is not None:
                best[axis] = shifts[k][axis] + step * _parabola_peak(sm, s[k], sp)
    return best, k


def _motion(T, ids):
    ids = np.asarray([i for i in ids if 1 <= i < len(T) - 1])
    rot = np.array([np.arccos(np.clip((np.trace(T[i - 1, :3, :3].T @ T[i + 1, :3, :3]) - 1) / 2, -1, 1))
                    for i in ids])
    trans = np.linalg.norm(T[ids + 1, :3, 3] - T[ids - 1, :3, 3], axis=1)
    return ids, rot, trans


def _spread(ids, n):
    if len(ids) <= n:
        return [int(i) for i in ids]
    return [int(ids[i]) for i in np.linspace(0, len(ids) - 1, n).astype(int)]


def select_frames(T, ids, n, cfg):
    """Low-motion frames (least rotation and translation to neighbours), spread over the capture:
    sharp images for the spatial check."""
    ids, rot, trans = _motion(T, ids)
    calm = ids[(rot <= np.percentile(rot, cfg['calm_percentile'])) &
               (trans <= np.percentile(trans, cfg['calm_percentile']))]
    return _spread(calm, n)


def select_moving_frames(T, ids, n, cfg):
    """Frames with moderate rotation: neighbouring video frames differ enough for the temporal
    check to tell them apart, without the motion blur of the fastest turns."""
    ids, rot, _ = _motion(T, ids)
    lo, hi = np.percentile(rot, cfg['temporal_motion_percentiles'])
    return _spread(ids[(rot >= lo) & (rot <= hi)], n)


def assess(per_frame, quadrants, temporal, cfg, px_scale):
    """Summary in depth pixels (px_scale = working pixels per depth pixel)."""
    shifts = np.array([f['shift_px'] for f in per_frame]) / px_scale
    med = np.median(shifts, axis=0)
    q = np.array([quadrants[k] for k in sorted(quadrants)]) / px_scale
    q_spread = float(np.max(np.linalg.norm(q - q.mean(axis=0), axis=1))) if len(q) else None
    wins = [t['paired_is_best'] for t in temporal if t['paired_is_best'] is not None]
    consistent = q_spread is None or q_spread <= cfg['quadrant_max_spread_px']
    if not consistent:
        status = 'inconsistent'          # quadrants disagree: scale/rotation error between depth and RGB
    elif np.linalg.norm(med) <= cfg['aligned_max_shift_px']:
        status = 'aligned'
    elif np.linalg.norm(med) <= cfg['max_correctable_shift_px']:
        status = 'constant_offset'       # same offset everywhere: correct with depth_to_rgb_offset_px
    else:
        status = 'inconsistent'
    return {
        'frames': len(per_frame),
        'status': status,
        'spatially_aligned': status == 'aligned',
        'depth_to_rgb_offset_px': med.tolist(),
        'depth_to_rgb_offset_note': 'RGB content sits this many depth pixels from the depth pixel that sees it; '
                                    'apply via depth_pixel_to_rgb when projecting between depth and RGB',
        'median_shift_depth_px': med.tolist(),
        'shift_spread_depth_px': (np.percentile(shifts, 75, axis=0) - np.percentile(shifts, 25, axis=0)).tolist(),
        'quadrant_shifts_depth_px': {k: (np.asarray(v) / px_scale).tolist() for k, v in quadrants.items()},
        'quadrant_max_deviation_depth_px': q_spread,
        'median_edge_contrast_at_best': float(np.median([f['score'] for f in per_frame])),
        'temporal_paired_frame_best_fraction': float(np.mean(wins)) if wins else None,
        'temporal_frames_compared': len(wins),
    }


def check_alignment(cap, T, frame_ids, video_path, offset_finder, cfg, dcfg):
    """Run the spatial and temporal checks; returns (summary, details for previews)."""
    from ..ingestion.video import scan_video
    s = cfg['work_scale']
    size = (cap.depth_size[0] * s, cap.depth_size[1] * s)
    picks = select_frames(T, frame_ids, cfg['samples'], cfg)
    moving = select_moving_frames(T, frame_ids, cfg['temporal_samples'], cfg)
    # Video index for depth frame f is f - k with k in {0..2} (found below); keep f-3 .. f+1.
    keep = {f + d for f in set(picks) | set(moving) for d in range(-3, 2)}
    pts, gray = scan_video(video_path, keep, size)
    align = offset_finder(pts, cap.timestamps)
    k = align['best_offset'] if align else 0
    shifts = shift_grid(cfg['max_shift_px'] * s, cfg['shift_step_px'] * s)

    per_frame, temporal, pooled = [], [], {'top_left': [], 'top_right': [], 'bottom_left': [], 'bottom_right': []}
    previews = []
    h, w = size[1], size[0]
    for f in picks:
        vi = f - k
        if vi not in gray:
            continue
        depth = cap.load_depth_raw(f).astype(float) * dcfg['unit_scale_m']
        edges = depth_edges(depth, cap.load_confidence(f), size, cfg)
        if edges.sum() < cfg['min_edge_pixels']:
            continue
        grad = image_gradient(gray[vi])
        d, score, _ = best_shift(edges, grad, shifts)
        per_frame.append({'frame': f, 'video_index': vi, 'shift_px': d.tolist(), 'score': float(score),
                          'edge_pixels': int(edges.sum())})
        for name, (rs, cs) in {'top_left': (slice(0, h // 2), slice(0, w // 2)),
                               'top_right': (slice(0, h // 2), slice(w // 2, w)),
                               'bottom_left': (slice(h // 2, h), slice(0, w // 2)),
                               'bottom_right': (slice(h // 2, h), slice(w // 2, w))}.items():
            m = np.zeros_like(edges)
            m[rs, cs] = edges[rs, cs]
            if m.sum() >= cfg['min_edge_pixels'] // 4:
                pooled[name].append((m, grad))
        if len(previews) < cfg['preview_frames']:
            previews.append((gray[vi], edges))

    # Temporal: with the camera moving, the paired video frame should line up with the depth
    # better than the frames just before and after it.
    for f in moving:
        vi = f - k
        if not all(vi + dv in gray for dv in (-1, 0, 1)):
            continue
        depth = cap.load_depth_raw(f).astype(float) * dcfg['unit_scale_m']
        edges = depth_edges(depth, cap.load_confidence(f), size, cfg)
        if edges.sum() < cfg['min_edge_pixels']:
            continue
        neigh = {dv: float(best_shift(edges, image_gradient(gray[vi + dv]), shifts)[1]) for dv in (-1, 0, 1)}
        temporal.append({'frame': f, 'scores': {str(kk): v for kk, v in neigh.items()},
                         'paired_is_best': max(neigh, key=neigh.get) == 0})

    quadrants = {}
    for name, items in pooled.items():
        if len(items) >= 3:
            total = sum(shift_scores(m, g, shifts) for m, g in items)
            quadrants[name] = refine_peak(shifts, total)[0].tolist()
    summary = assess(per_frame, quadrants, temporal, cfg, s) if per_frame else {'frames': 0}
    summary.update({'video_pairing': {k_: align[k_] for k_ in ('mapping', 'best_offset', 'best_is_exact',
                                                               'best_is_unique')} if align else None,
                    'method': 'sub-pixel shift maximizing RGB edge strength at depth discontinuities',
                    'working_resolution': list(size)})
    return summary, {'per_frame': per_frame, 'temporal': temporal}, previews


def depth_pixel_to_rgb(u, v, depth_size, rgb_size, offset_depth_px=(0.0, 0.0)):
    """RGB pixel seeing the same point as depth pixel (u, v): pixel-centre scaling between the two
    resolutions plus the measured constant depth-to-RGB offset (in depth pixels)."""
    s = rgb_size[0] / depth_size[0]
    return ((np.asarray(u, float) + offset_depth_px[0] + 0.5) * s - 0.5,
            (np.asarray(v, float) + offset_depth_px[1] + 0.5) * s - 0.5)


def render_previews(path, previews):
    tiles = []
    for gray, edges in previews:
        rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        rgb[edges] = (0, 0, 255)
        tiles.append(rgb)
    if tiles:
        cv2.imwrite(str(path), np.vstack([np.hstack(tiles[i:i + 2]) if len(tiles[i:i + 2]) == 2 else
                                          np.hstack([tiles[i], np.zeros_like(tiles[i])])
                                          for i in range(0, len(tiles), 2)]))
