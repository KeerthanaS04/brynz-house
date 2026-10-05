"""4-DoF pose-graph drift correction (heading about gravity + position).

Nodes are keyframes. Each node's state is a heading correction delta (rotation
about the up axis applied to the device orientation) and a corrected position;
roll and pitch stay as measured by the device (gravity-aligned, confirmed by
calibration/gravity.py). This is the 4-DoF formulation used by visual-inertial
systems for loop closure (e.g. VINS-Mono).

Edges constrain the relative heading and the relative position expressed in the
first node's corrected heading frame:
  r_ij = [delta_j - delta_i - z_theta,  R(-delta_i)(p_j - p_i) - z_p]
* Odometry edges between consecutive keyframes keep the device's relative
  motion (z_theta = 0, z_p = raw displacement), weighted by the drift model in
  configs/default.yaml (assumptions.md B-22), so consistent poses barely move.
* Loop edges come from revisits: two keyframes >= revisit_min_gap_s apart that
  see the same surfaces are registered directly to each other (4-DoF
  point-to-plane ICP), and the measured relative pose is added if the
  registration is well supported and plausible for the distance walked.
Loop edges are robustly weighted; edges still inconsistent after solving are
removed and the graph is solved again.
"""
import numpy as np

from ..geometry.transforms import transform
from .conventions import _pair_residual
from .drift import (VoxelMap, _yaw_and_t, detect_jumps, icp_4dof, interpolate_corrections, keyframe_points,
                    rotation_about, select_keyframes)


def _skew(u):
    return np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])


def edge_residual(xi, xj, z, up):
    """Residual of one edge and its Jacobians w.r.t. node states (delta, p)."""
    di, pi = xi[0], xi[1:]
    dj, pj = xj[0], xj[1:]
    Rm = rotation_about(up, -di)
    v = Rm @ (pj - pi)
    r = np.r_[dj - di - z[0], v - z[1:]]
    Ji = np.zeros((4, 4))
    Jj = np.zeros((4, 4))
    Ji[0, 0], Jj[0, 0] = -1.0, 1.0
    Ji[1:, 0] = -_skew(up) @ v
    Ji[1:, 1:] = -Rm
    Jj[1:, 1:] = Rm
    return r, Ji, Jj


def solve_pose_graph(x0, edges, up, iterations=10, prior_weight=1e6):
    """Gauss-Newton on node states x (N, 4). Edges: dicts with i, j, z (4,), info (4,), huber (or None).
    Node 0 is held at its initial state. Returns optimized states and final per-edge chi values."""
    x = x0.copy()
    n = len(x)
    for _ in range(iterations):
        H = np.zeros((4 * n, 4 * n))
        g = np.zeros(4 * n)
        for e in edges:
            r, Ji, Jj = edge_residual(x[e['i']], x[e['j']], e['z'], up)
            w = e['info'].copy()
            if e.get('huber'):
                chi = np.sqrt(np.sum(w * r * r))
                if chi > e['huber']:
                    w *= e['huber'] / chi
            si, sj = slice(4 * e['i'], 4 * e['i'] + 4), slice(4 * e['j'], 4 * e['j'] + 4)
            WJi, WJj = Ji.T * w, Jj.T * w
            H[si, si] += WJi @ Ji
            H[sj, sj] += WJj @ Jj
            H[si, sj] += WJi @ Jj
            H[sj, si] += WJj @ Ji
            g[si] += WJi @ r
            g[sj] += WJj @ r
        H[:4, :4] += prior_weight * np.eye(4)
        g[:4] += prior_weight * (x[0] - x0[0])
        dx = np.linalg.solve(H, -g).reshape(n, 4)
        x += dx
        if np.abs(dx).max() < 1e-7:
            break
    chis = [float(np.sqrt(np.sum(e['info'] * edge_residual(x[e['i']], x[e['j']], e['z'], up)[0] ** 2)))
            for e in edges]
    return x, chis


def _loop_candidates(cap, T, keys, axes, cfg, dcfg):
    fwd = T[:, :3, 2] * (1 if axes == 'opencv' else -1)
    pos = T[:, :3, 3]
    ts = cap.timestamps
    keys = np.asarray(keys)
    cand = []
    for a_i, a in enumerate(keys):
        b = keys[a_i + 1:]
        b = b[ts[b] - ts[a] >= cfg['revisit_min_gap_s']]
        if not len(b):
            continue
        d = np.linalg.norm(pos[b] - pos[a], axis=1)
        ok = (d < cfg['revisit_max_distance_m']) & \
            (fwd[b] @ fwd[a] > np.cos(np.radians(cfg['revisit_max_angle_deg'])))
        cand += [(int(a), int(x)) for x in b[ok]]
    if len(cand) > cfg['loop_max_candidates']:
        sel = np.linspace(0, len(cand) - 1, cfg['loop_max_candidates']).astype(int)
        cand = [cand[i] for i in sel]
    out = []
    for a, b in cand:
        da = cap.load_depth_raw(a).astype(float) * dcfg['unit_scale_m']
        db = cap.load_depth_raw(b).astype(float) * dcfg['unit_scale_m']
        r = _pair_residual(da, cap.load_confidence(a), cap.depth_intrinsics(a), T[a], db, cap.depth_intrinsics(b),
                           T[b], axes, dcfg['confidence_min'], cfg['pixel_step'], dcfg['min_m'], dcfg['max_m'])
        if r and r['overlap_fraction'] >= cfg['revisit_min_overlap']:
            out.append((a, b))
    return out


def correct_drift_pose_graph(cap, T_raw, axes, up, cfg, dcfg):
    """Corrected camera-to-world poses plus a summary and per-loop report."""
    jumps = detect_jumps(cap.positions, cfg['jump_threshold_m'])
    frame_ids = list(range(0, len(cap), cfg['frame_stride']))
    keys = select_keyframes(T_raw, frame_ids, cfg, jumps)
    k_index = {f: i for i, f in enumerate(keys)}
    steps = np.r_[0.0, np.linalg.norm(np.diff(T_raw[:, :3, 3], axis=0), axis=1)]
    jump_size = {j: float(steps[j]) for j in jumps}
    steps[list(jumps)] = 0.0
    walked = np.cumsum(steps)

    pts = {}

    def world_points(f):
        if f not in pts:
            Pc, nc = keyframe_points(cap, f, axes, cfg, dcfg)
            if len(Pc) > cfg['max_points_per_keyframe']:
                sel = np.linspace(0, len(Pc) - 1, cfg['max_points_per_keyframe']).astype(int)
                Pc, nc = Pc[sel], nc[sel]
            pts[f] = (transform(T_raw[f], Pc), nc @ T_raw[f][:3, :3].T)
        return pts[f]

    x0 = np.column_stack([np.zeros(len(keys)), T_raw[keys, :3, 3]])
    edges = []
    for k in range(len(keys) - 1):
        a, b = keys[k], keys[k + 1]
        dist = float(walked[b] - walked[a])
        jumped = jump_size.get(b, 0.0) if b in jump_size else 0.0
        s_t = cfg['odom_sigma_base_m'] + cfg['drift_rate'] * dist + cfg['jump_allowance'] * jumped
        s_r = np.radians(cfg['odom_sigma_base_deg'] + cfg['drift_rate_deg_per_m'] * dist +
                         (cfg['jump_allowance_deg'] if jumped else 0.0))
        edges.append({'i': k, 'j': k + 1, 'kind': 'odometry',
                      'z': np.r_[0.0, T_raw[b, :3, 3] - T_raw[a, :3, 3]],
                      'info': np.r_[1 / s_r ** 2, np.full(3, 1 / s_t ** 2)]})

    def submap(f):
        vm = VoxelMap(cfg['map_voxel_m'])
        i = k_index[f]
        for g in keys[max(0, i - cfg['loop_submap_keyframes']): i + cfg['loop_submap_keyframes'] + 1]:
            P, n = world_points(g)
            vm.add(P, n)
        return vm

    loops = []
    for a, b in _loop_candidates(cap, T_raw, keys, axes, cfg, dcfg):
        ia = k_index[a]
        Pb, _ = world_points(b)
        C, info = icp_4dof(Pb, submap(a), up, np.eye(4), cfg)
        # Forward-backward check: registering a onto b must undo registering b onto a;
        # ill-conditioned registrations (e.g. heading weakly constrained) disagree.
        Pa, _ = world_points(a)
        C_back, _ = icp_4dof(Pa, submap(b), up, np.eye(4), cfg)
        E = C_back @ C
        fb_yaw = abs(np.degrees(_yaw_and_t(E, up)[0]))
        fb_disp = float(np.mean(np.linalg.norm(transform(E, Pb) - Pb, axis=1)))
        yaw, t = _yaw_and_t(C, up)
        pb_new = C[:3, :3] @ T_raw[b, :3, 3] + C[:3, 3]
        shift = float(np.linalg.norm(pb_new - T_raw[b, :3, 3]))
        dist = float(walked[b] - walked[a])
        jumped = sum(s for j, s in jump_size.items() if a < j <= b)
        lim_t = cfg['drift_base_m'] + cfg['drift_rate'] * dist + cfg['jump_allowance'] * jumped
        lim_r = cfg['drift_base_deg'] + cfg['drift_rate_deg_per_m'] * dist + (cfg['jump_allowance_deg'] if jumped else 0)
        reasons = []
        if info.get('reason'):
            reasons.append(info['reason'])
        if info.get('inlier_fraction', 0) < cfg['loop_min_inlier_fraction']:
            reasons.append('low inlier fraction')
        if info.get('eigen_min', 0) < cfg['icp_min_eigen']:
            reasons.append('degenerate geometry')
        if info.get('rms_m', 1) > cfg['loop_max_rms_m']:
            reasons.append('high residual')
        if shift > lim_t or abs(np.degrees(yaw)) > lim_r:
            reasons.append('implausible for distance walked')
        if fb_yaw > cfg['loop_fb_max_deg'] or fb_disp > cfg['loop_fb_max_m']:
            reasons.append('forward-backward inconsistent')
        entry = {'a': a, 'b': b, 'walked_m': dist, 'shift_m': shift, 'yaw_deg': float(np.degrees(yaw)),
                 'fb_yaw_deg': fb_yaw, 'fb_displacement_m': fb_disp,
                 'inlier_fraction': info.get('inlier_fraction'), 'rms_m': info.get('rms_m'),
                 'eigen_min': info.get('eigen_min'), 'status': 'rejected: ' + ', '.join(reasons) if reasons else 'used'}
        loops.append(entry)
        if not reasons:
            edges.append({'i': ia, 'j': k_index[b], 'kind': 'loop', 'loop': len(loops) - 1,
                          'z': np.r_[yaw, pb_new - T_raw[a, :3, 3]],
                          'info': np.r_[1 / np.radians(cfg['loop_sigma_deg']) ** 2,
                                        np.full(3, 1 / cfg['loop_sigma_m'] ** 2)],
                          'huber': cfg['loop_huber']})

    x, chis = solve_pose_graph(x0, edges, up, cfg['graph_iterations'])
    removed = 0
    for e, chi in zip(edges, chis):
        if e['kind'] == 'loop' and chi > cfg['loop_max_chi']:
            loops[e['loop']]['status'] = f'removed after solve: inconsistent (chi {chi:.1f})'
            removed += 1
    if removed:
        edges = [e for e, chi in zip(edges, chis) if not (e['kind'] == 'loop' and chi > cfg['loop_max_chi'])]
        x, chis = solve_pose_graph(x0, edges, up, cfg['graph_iterations'])

    key_C = []
    for k, f in enumerate(keys):
        R = rotation_about(up, x[k, 0])
        C = np.eye(4)
        C[:3, :3] = R
        C[:3, 3] = x[k, 1:] - R @ T_raw[f, :3, 3]
        key_C.append(C)
    Cs = interpolate_corrections(len(cap), keys, key_C, up)
    T_corr = np.einsum('nij,njk->nik', Cs, T_raw)

    used = [l for l in loops if l['status'] == 'used']
    shifts = np.linalg.norm(x[:, 1:] - x0[:, 1:], axis=1)
    summary = {
        'method': '4-DoF pose graph: device odometry edges + loop edges from pairwise ICP at revisits',
        'jumps_detected_at_frames': jumps,
        'keyframes': len(keys),
        'loop_candidates': len(loops),
        'loops_used': len(used),
        'loops_rejected': sum(l['status'].startswith('rejected') for l in loops),
        'loops_removed_after_solve': removed,
        'max_correction_translation_m': float(shifts.max()),
        'max_correction_yaw_deg': float(np.degrees(np.abs(x[:, 0]).max())),
        'final_correction_translation_m': float(shifts[-1]),
        'final_correction_yaw_deg': float(np.degrees(x[-1, 0])),
        # keep the keys used by the keyframe-to-map summary so reports stay comparable
        'accepted': len(used), 'rejected': len(loops) - len(used),
    }
    return T_corr, summary, {'keyframes': [int(k) for k in keys], 'loops': loops}
