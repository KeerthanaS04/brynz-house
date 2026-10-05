"""Drift and tracking-jump correction by keyframe-to-map registration.

Device odometry is used as the initial guess. Keyframes (by camera motion, plus
forced keyframes at tracking jumps) are registered one by one to a voxel map
built from previously corrected keyframes, with point-to-plane ICP restricted
to 4 degrees of freedom: translation and rotation about the gravity axis.
Roll and pitch are left to the odometry, whose gravity alignment is confirmed
by the accelerometer check (calibration/gravity.py). Revisiting an area
re-aligns it to where it was first mapped, which acts as implicit loop closure.

A keyframe's correction is accepted only when the registration is well
supported; otherwise the previous correction is carried forward and the
keyframe is flagged. Corrections between keyframes are interpolated. Raw and
corrected trajectories are both kept.

Correction convention: corrected camera-to-world = C @ raw camera-to-world,
with C a rotation by theta about the up axis through the world origin plus a
translation t.
"""
import numpy as np

from ..geometry.transforms import backproject, invert, transform

_OFF = 1 << 20


def _pack(keys):
    k = keys.astype(np.int64) + _OFF
    return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]


def rotation_about(axis, theta):
    """Rodrigues rotation matrix about a unit axis."""
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * K @ K


def correction_matrix(theta, t, up):
    C = np.eye(4)
    C[:3, :3] = rotation_about(up, theta)
    C[:3, 3] = t
    return C


# ---- tracking jumps and keyframes ----

def detect_jumps(positions, threshold):
    """Frame indices i where the step from i-1 to i exceeds threshold."""
    steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    return [int(i + 1) for i in np.flatnonzero(steps > threshold)]


def select_keyframes(T, frame_ids, cfg, forced):
    """Keyframes from frame_ids by motion since the last keyframe; forced frames always included."""
    keys = [frame_ids[0]]
    for i in frame_ids[1:]:
        a, b = T[keys[-1]], T[i]
        moved = np.linalg.norm(b[:3, 3] - a[:3, 3])
        cos = np.clip((np.trace(a[:3, :3].T @ b[:3, :3]) - 1) / 2, -1, 1)
        if moved >= cfg['keyframe_translation_m'] or np.degrees(np.arccos(cos)) >= cfg['keyframe_rotation_deg']:
            keys.append(i)
    keys = sorted(set(keys) | {f for j in forced for f in (j - 1, j) if 0 <= f < len(T)})
    return keys


# ---- keyframe points with normals ----

def keyframe_points(cap, i, axes, cfg, dcfg):
    """Camera-frame points and normals from one depth frame (high-confidence pixels only)."""
    step = cfg['pixel_step']
    d = cap.load_depth_raw(i).astype(float) * dcfg['unit_scale_m']
    c = cap.load_confidence(i)
    K = cap.depth_intrinsics(i)
    H, W = d.shape
    v, u = np.mgrid[0:H:step, 0:W:step]
    P = backproject(u.astype(float), v.astype(float), d[v, u], K, axes)
    du = np.full_like(P, np.nan)
    dv = np.full_like(P, np.nan)
    du[:, :-1] = P[:, 1:] - P[:, :-1]
    dv[:-1, :] = P[1:, :] - P[:-1, :]
    n = np.cross(du, dv)
    norm = np.linalg.norm(n, axis=-1, keepdims=True)
    with np.errstate(invalid='ignore', divide='ignore'):
        n = n / norm
    dd = d[v, u]
    ok = (c[v, u] >= cfg['confidence_min']) & (dd > dcfg['min_m']) & (dd < cfg['max_depth_m'])
    ok &= np.isfinite(n).all(axis=-1) & (norm[..., 0] > 0)
    # neighbours must lie on the same surface (no normals across depth edges)
    ok[:, :-1] &= np.abs(dd[:, 1:] - dd[:, :-1]) < cfg['edge_depth_jump_m']
    ok[:-1, :] &= np.abs(dd[1:, :] - dd[:-1, :]) < cfg['edge_depth_jump_m']
    P, n = P[ok], n[ok]
    view = P / np.linalg.norm(P, axis=1, keepdims=True)
    flip = np.sum(n * view, axis=1) > 0
    n[flip] = -n[flip]
    grazing = np.abs(np.sum(n * view, axis=1)) < np.cos(np.radians(cfg['max_incidence_deg']))
    return P[~grazing], n[~grazing]


# ---- voxel map with nearest-neighbour lookup ----

class VoxelMap:
    """One averaged point + normal per voxel; nearest-neighbour search over the 27 adjacent voxels."""

    def __init__(self, voxel):
        self.voxel = voxel
        self.keys = np.empty(0, np.int64)
        self.points = np.empty((0, 3))
        self.normals = np.empty((0, 3))
        self.birth = np.empty(0, np.int64)

    def __len__(self):
        return len(self.keys)

    def add(self, P, n, birth=0):
        """Add points to voxels not yet in the map (the first observation of a voxel anchors it).
        `birth` records which keyframe created each voxel."""
        k = _pack(np.floor(P / self.voxel))
        uk, first = np.unique(k, return_index=True)
        if len(self.keys):
            pos = np.clip(np.searchsorted(self.keys, uk), 0, len(self.keys) - 1)
            new = self.keys[pos] != uk
            uk, first = uk[new], first[new]
        keys = np.concatenate([self.keys, uk])
        order = np.argsort(keys, kind='stable')
        self.keys = keys[order]
        self.points = np.concatenate([self.points, P[first]])[order]
        self.normals = np.concatenate([self.normals, n[first]])[order]
        self.birth = np.concatenate([self.birth, np.full(len(uk), birth, np.int64)])[order]

    def nearest(self, Q):
        """Index of the nearest map point within the 27 voxels around each query (-1 if none) and distance.
        Exact when the true nearest point is within one voxel; farther matches are approximate, which
        only affects early ICP iterations before the correspondence gate shrinks."""
        if not len(self.keys):
            return np.full(len(Q), -1), np.full(len(Q), np.inf)
        base = np.floor(Q / self.voxel).astype(np.int64)
        best = np.full(len(Q), -1)
        dist = np.full(len(Q), np.inf)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    k = _pack(base + np.array([dx, dy, dz]))
                    pos = np.clip(np.searchsorted(self.keys, k), 0, len(self.keys) - 1)
                    hit = self.keys[pos] == k
                    d = np.where(hit, np.linalg.norm(self.points[pos] - Q, axis=1), np.inf)
                    better = d < dist
                    best[better], dist[better] = pos[better], d[better]
        return best, dist


# ---- 4-DoF point-to-plane ICP ----

def icp_4dof(P, vmap, up, C0, cfg):
    """Register world points P (already moved by the raw pose) to the map, starting from C0.

    Returns (C, info). Rotation is about `up` through the points' centroid; the result is
    re-expressed about the world origin.
    """
    C = C0.copy()
    info = {'converged': False, 'iterations': 0}
    max_d = cfg['icp_max_distance_m']
    for it in range(cfg['icp_iterations']):
        Q = transform(C, P)
        idx, d = vmap.nearest(Q)
        ok = (idx >= 0) & (d < max_d)
        info.update({'iterations': it + 1, 'inliers': int(ok.sum()), 'inlier_fraction': float(ok.mean())})
        if ok.sum() < cfg['icp_min_inliers']:
            return C, {**info, 'reason': 'too few correspondences'}
        q, nq, p = vmap.points[idx[ok]], vmap.normals[idx[ok]], Q[ok]
        c = p.mean(axis=0)
        r = np.sum((p - q) * nq, axis=1)
        w = np.where(np.abs(r) < cfg['icp_huber_m'], 1.0, cfg['icp_huber_m'] / np.abs(r))
        J = np.column_stack([np.sum(nq * np.cross(up, p - c), axis=1), nq])
        A = (J * w[:, None]).T @ J + cfg['icp_damping'] * np.eye(4)
        b = -(J * w[:, None]).T @ r
        x = np.linalg.solve(A, b)
        R = rotation_about(up, x[0])
        step = np.eye(4)
        step[:3, :3] = R
        step[:3, 3] = c - R @ c + x[1:]
        C = step @ C
        info['rms_m'] = float(np.sqrt(np.average(r ** 2, weights=w)))
        info['eigen_min'] = float(np.linalg.eigvalsh((J * w[:, None]).T @ J / max(len(r), 1)).min())
        info['matched_birth'] = int(np.median(vmap.birth[idx[ok]]))
        if abs(x[0]) < 1e-5 and np.linalg.norm(x[1:]) < 1e-4:
            info['converged'] = True
            break
        max_d = max(cfg['icp_min_distance_m'], max_d * 0.7)
    return C, info


def _yaw_and_t(C, up):
    R = C[:3, :3]
    e = np.array([1.0, 0, 0]) if abs(up[0]) < 0.9 else np.array([0, 0, 1.0])
    e = e - (e @ up) * up
    e /= np.linalg.norm(e)
    Re = R @ e
    theta = np.arctan2(np.cross(e, Re) @ up, e @ Re)
    return float(theta), C[:3, 3].copy()


def interpolate_corrections(n_frames, key_frames, key_C, up):
    """Per-frame corrections by linear interpolation of yaw and translation between keyframes."""
    yaw_t = [_yaw_and_t(C, up) for C in key_C]
    yaws = np.unwrap([y for y, _ in yaw_t])
    ts = np.array([t for _, t in yaw_t])
    f = np.arange(n_frames)
    yaw = np.interp(f, key_frames, yaws)
    t = np.column_stack([np.interp(f, key_frames, ts[:, k]) for k in range(3)])
    return np.stack([correction_matrix(yaw[i], t[i], up) for i in range(n_frames)])


def correct_drift(cap, T_raw, axes, up, cfg, dcfg):
    """Corrected camera-to-world poses plus a report of keyframes, jumps and corrections."""
    jumps = detect_jumps(cap.positions, cfg['jump_threshold_m'])
    frame_ids = list(range(0, len(cap), cfg['frame_stride']))
    keys = select_keyframes(T_raw, frame_ids, cfg, jumps)
    jump_keys = {j for j in jumps}
    steps = np.r_[0.0, np.linalg.norm(np.diff(T_raw[:, :3, 3], axis=0), axis=1)]
    jump_size = {j: float(steps[j]) for j in jumps}
    steps[list(jumps)] = 0.0                       # a jump is a tracking error, not distance walked
    walked = np.cumsum(steps)
    vmap = VoxelMap(cfg['map_voxel_m'])
    key_C, report = [], []
    C = np.eye(4)
    for k_i, f in enumerate(keys):
        Pc, nc = keyframe_points(cap, f, axes, cfg, dcfg)
        if len(Pc) > cfg['max_points_per_keyframe']:
            sel = np.linspace(0, len(Pc) - 1, cfg['max_points_per_keyframe']).astype(int)
            Pc, nc = Pc[sel], nc[sel]
        P = transform(T_raw[f], Pc)
        n_w = nc @ T_raw[f][:3, :3].T
        entry = {'frame': int(f), 'points': int(len(P)), 'jump': f in jump_keys}
        if k_i == 0 or len(vmap) == 0:
            entry['status'] = 'anchor'
        else:
            C_new, info = icp_4dof(P, vmap, up, C, cfg)
            entry.update(info)
            dyaw, dt = _yaw_and_t(invert(C) @ C_new, up)
            # Odometry drift grows with distance walked, so the plausible correction change is
            # bounded by the distance walked since the matched map region was first scanned:
            # small when tracking new territory, larger when closing a loop on old map.
            birth_frame = keys[min(info.get('matched_birth', k_i), k_i)]
            dist = float(walked[f] - walked[birth_frame])
            jumped = sum(s for j, s in jump_size.items() if birth_frame < j <= f)
            limit_t = cfg['drift_base_m'] + cfg['drift_rate'] * dist + cfg['jump_allowance'] * jumped
            limit_r = cfg['drift_base_deg'] + cfg['drift_rate_deg_per_m'] * dist + \
                (cfg['jump_allowance_deg'] if jumped else 0.0)
            entry.update({'walked_since_matched_map_m': dist, 'limit_translation_m': limit_t,
                          'limit_yaw_deg': limit_r})
            reasons = []
            if info.get('reason'):
                reasons.append(info['reason'])
            if info.get('inlier_fraction', 0) < cfg['icp_min_inlier_fraction']:
                reasons.append('low inlier fraction')
            if info.get('eigen_min', 0) < cfg['icp_min_eigen']:
                reasons.append('degenerate geometry')
            if np.linalg.norm(dt) > limit_t or abs(np.degrees(dyaw)) > limit_r:
                reasons.append('correction change too large')
            entry['delta_translation_m'] = float(np.linalg.norm(dt))
            entry['delta_yaw_deg'] = float(np.degrees(dyaw))
            if reasons:
                entry['status'] = 'rejected: ' + ', '.join(reasons)
            else:
                entry['status'] = 'accepted'
                C = C_new
        key_C.append(C.copy())
        # Only keyframes whose alignment is trusted extend the map; a rejected keyframe's
        # geometry at the carried-forward correction could be misaligned and would bias
        # every later registration against it.
        if entry['status'] in ('anchor', 'accepted'):
            vmap.add(transform(C, P), n_w @ C[:3, :3].T, birth=k_i)
        yaw, t = _yaw_and_t(C, up)
        entry['correction_yaw_deg'] = float(np.degrees(yaw))
        entry['correction_translation_m'] = t.tolist()
        report.append(entry)

    Cs = interpolate_corrections(len(cap), keys, key_C, up)
    T_corr = np.einsum('nij,njk->nik', Cs, T_raw)
    statuses = [e['status'] for e in report]
    final_yaw, final_t = _yaw_and_t(key_C[-1], up)
    summary = {
        'method': '4-DoF point-to-plane ICP of keyframes to a voxel map of earlier corrected keyframes',
        'jumps_detected_at_frames': jumps,
        'keyframes': len(keys),
        'accepted': statuses.count('accepted'),
        'rejected': sum(s.startswith('rejected') for s in statuses),
        'map_voxels': len(vmap),
        'final_correction_yaw_deg': float(np.degrees(final_yaw)),
        'final_correction_translation_m': float(np.linalg.norm(final_t)),
        'max_correction_translation_m': float(max(np.linalg.norm(e['correction_translation_m']) for e in report)),
        'max_correction_yaw_deg': float(max(abs(e['correction_yaw_deg']) for e in report)),
    }
    return T_corr, summary, report


# ---- drift evidence without ground truth ----

def revisit_pairs(cap, T, axes, cfg, dcfg, exclude=()):
    """Frame pairs far apart in time that see the same surfaces.

    Candidates are frames >= revisit_min_gap_s apart within a loose distance and view-angle
    bound; a pair is kept when at least revisit_min_overlap of one frame's depth projects into
    the other's view under T (raw poses here, which favours raw in the comparison). Frames in
    `exclude` (e.g. keyframes used to build loop constraints) are never used, so the pairs
    stay held out from the correction.
    """
    from .conventions import _pair_residual
    fwd = T[:, :3, 2] * (1 if axes == 'opencv' else -1)
    pos = T[:, :3, 3]
    ts = cap.timestamps
    excluded = set(int(e) for e in exclude)
    ids = np.array([i for i in range(0, len(T), cfg['frame_stride']) if i not in excluded])
    cand = []
    for a_i, a in enumerate(ids):
        b = ids[a_i + 1:]
        b = b[ts[b] - ts[a] >= cfg['revisit_min_gap_s']]
        if not len(b):
            continue
        d = np.linalg.norm(pos[b] - pos[a], axis=1)
        ok = (d < cfg['revisit_max_distance_m']) & \
            (fwd[b] @ fwd[a] > np.cos(np.radians(cfg['revisit_max_angle_deg'])))
        if ok.any():
            cand.append((int(a), int(b[ok][np.argmin(d[ok])])))
    if len(cand) > cfg['revisit_max_candidates']:
        sel = np.linspace(0, len(cand) - 1, cfg['revisit_max_candidates']).astype(int)
        cand = [cand[i] for i in sel]
    cache = {}

    def frame(i):
        if i not in cache:
            cache[i] = (cap.load_depth_raw(i).astype(float) * dcfg['unit_scale_m'], cap.load_confidence(i))
        return cache[i]

    pairs = []
    for a, b in cand:
        (da, ca), (db, _) = frame(a), frame(b)
        r = _pair_residual(da, ca, cap.depth_intrinsics(a), T[a], db, cap.depth_intrinsics(b), T[b], axes,
                           dcfg['confidence_min'], cfg['pixel_step'], dcfg['min_m'], dcfg['max_m'])
        if r and r['overlap_fraction'] >= cfg['revisit_min_overlap']:
            pairs.append((a, b))
    if len(pairs) > cfg['revisit_max_pairs']:
        sel = np.linspace(0, len(pairs) - 1, cfg['revisit_max_pairs']).astype(int)
        pairs = [pairs[i] for i in sel]
    return pairs


def revisit_consistency(cap, trajectories, pairs, axes, cfg, dcfg):
    """Depth agreement between revisit pairs under each trajectory (lower residual = less drift).

    A pair is informative only if at least one trajectory makes it agree within
    revisit_informative_max_m; pairs that disagree under every trajectory (occlusion,
    different surfaces) say nothing about drift and are dropped for all trajectories alike.
    """
    from .conventions import _pair_residual
    names = list(trajectories)
    if not pairs:
        return {**{n: None for n in names}, 'informative_pairs': 0}
    frames = sorted({i for p in pairs for i in p})
    depth = {i: cap.load_depth_raw(i).astype(float) * dcfg['unit_scale_m'] for i in frames}
    conf = {i: cap.load_confidence(i) for i in frames}
    per = {n: [] for n in names}
    for a, b in pairs:
        res = {n: _pair_residual(depth[a], conf[a], cap.depth_intrinsics(a), T[a], depth[b],
                                 cap.depth_intrinsics(b), T[b], axes, dcfg['confidence_min'], cfg['pixel_step'],
                                 dcfg['min_m'], dcfg['max_m']) for n, T in trajectories.items()}
        meds = [r['median_abs_residual_m'] for r in res.values() if r and r['median_abs_residual_m'] is not None]
        if len(meds) == len(names) and min(meds) <= cfg['revisit_informative_max_m']:
            for n in names:
                per[n].append(res[n])
    out = {'informative_pairs': len(per[names[0]]), 'pairs_dropped_uninformative': len(pairs) - len(per[names[0]])}
    for n in names:
        rs = per[n]
        out[n] = ({'median_abs_residual_m': float(np.median([r['median_abs_residual_m'] for r in rs])),
                   'mean_within_5cm': float(np.mean([r['within_5cm'] for r in rs])),
                   'pairs': len(rs)} if rs else None)
    return out
