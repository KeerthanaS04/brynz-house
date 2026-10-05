"""Plane fitting (SVD, RANSAC) and horizontal plane detection."""
import numpy as np


def fit_plane(P):
    """Least-squares plane through points -> (unit normal, centroid)."""
    c = P.mean(axis=0)
    _, _, vt = np.linalg.svd(P - c, full_matrices=False)
    return vt[2], c


def ransac_plane(P, threshold, iterations, rng, normal_hint=None, max_angle_deg=None):
    """RANSAC plane, optionally restricted to normals within max_angle_deg of normal_hint.

    Returns dict(normal, point, inliers, rms_m) with normal oriented along
    normal_hint when given, or None if no valid hypothesis was found.
    """
    n_pts = len(P)
    if n_pts < 3:
        return None
    min_cos = np.cos(np.radians(max_angle_deg)) if normal_hint is not None and max_angle_deg else None
    best = None
    for _ in range(iterations):
        s = P[rng.choice(n_pts, 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        norm = np.linalg.norm(n)
        if norm < 1e-12:
            continue
        n /= norm
        if min_cos is not None and abs(n @ normal_hint) < min_cos:
            continue
        inl = np.abs((P - s[0]) @ n) < threshold
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < 3:
        return None
    n, c = fit_plane(P[best])
    if normal_hint is not None and n @ normal_hint < 0:
        n = -n
    resid = (P[best] - c) @ n
    return {'normal': n, 'point': c, 'inliers': best, 'rms_m': float(np.sqrt(np.mean(resid ** 2)))}


def find_horizontal_plane(P, up, region, cfg, rng, min_fraction):
    """Strongest horizontal plane among points whose height along `up` satisfies `region(h)`.

    A height histogram picks the densest slab; RANSAC restricted to near-
    horizontal normals refines it. Returns None when support is below
    min_fraction of all points.
    """
    h = P @ up
    step = cfg['histogram_bin_m']
    edges = np.arange(h.min(), h.max() + 2 * step, step)
    hist, _ = np.histogram(h, edges)
    centres = (edges[:-1] + edges[1:]) / 2
    allowed = region(centres)
    if not allowed.any():
        return None
    j = int(np.argmax(np.where(allowed, hist, -1)))
    win = cfg['peak_window_m']
    sel = np.abs(h - centres[j]) < win
    support = sel.sum() / len(P)
    if support < min_fraction:
        return None
    pl = ransac_plane(P[sel], cfg['ransac_threshold_m'], cfg['ransac_iterations'], rng,
                      normal_hint=up, max_angle_deg=cfg['max_tilt_deg'])
    if pl is None:
        return None
    idx = np.flatnonzero(sel)[pl['inliers']]
    return {
        'normal': pl['normal'], 'point': pl['point'], 'rms_m': pl['rms_m'],
        'inlier_index': idx,
        'histogram_peak_height_m': float(centres[j]),
        'support_fraction': float(support),
        'tilt_from_up_deg': float(np.degrees(np.arccos(np.clip(abs(pl['normal'] @ up), 0, 1)))),
    }
