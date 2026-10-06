"""Video tier, step 2: depth per frame, one metric trajectory, and frames the room stages accept.

Inputs: the selected frames and SfM pieces of a step-1 video run (video_sfm.py) and a pretrained
monocular depth network (weights fetched by scripts/fetch_weights.py). Nothing recorded by the
phone other than the video is read (CLAUDE_updated.md rule 6; tests/unit/test_video_depth.py).

1. Depth per frame from the network. Its shape is good but its scale drifts from frame to frame
   (single_room vs LiDAR: within 5% after one factor per frame, but that factor changes by 7%
   between consecutive frames), so each frame gets a depth correction factor.
2. SfM pieces are split into runs of consecutive frames. Inside a run, a frame's correction comes
   from the ratio of network depth to SfM depth at its 3D points.
3. Consecutive frames are linked by feature matches and the network depth of the first frame
   (PnP). This needs no parallax, so it bridges the turns on the spot where SfM breaks; the
   second frame's correction makes its depth agree with the first frame's matched points.
   One unit of length thus carries through runs and links. Metres: the network's scale on the
   median frame (uncalibrated). A run whose motion disagrees with the links is rejected.
4. Up direction: with the phone held upright the camera x axis stays horizontal, so up is the
   direction most perpendicular to every camera x axis; the floor plane refines it later.
5. VideoFrames presents frames, network depth and intrinsics like an RGB-D capture, so fusion,
   floor plan and openings run unchanged.
"""
from pathlib import Path

import cv2
import numpy as np
import pycolmap

from ..geometry.transforms import backproject, invert, scale_intrinsics


class DepthModel:
    """Pretrained monocular depth network (Hugging Face transformers); predicts metres per pixel."""

    def __init__(self, weights_dir, device='auto'):
        import torch
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation
        weights_dir = Path(weights_dir)
        if not weights_dir.exists():
            raise FileNotFoundError(f'{weights_dir} not found: run python scripts/fetch_weights.py')
        self.torch = torch
        self.device = ('cuda' if torch.cuda.is_available() else 'cpu') if device == 'auto' else device
        self.processor = AutoImageProcessor.from_pretrained(weights_dir)
        self.model = AutoModelForDepthEstimation.from_pretrained(weights_dir).to(self.device).eval()

    def predict(self, bgr, size):
        """Depth in metres, resampled to `size` (width, height)."""
        inputs = self.processor(images=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), return_tensors='pt').to(self.device)
        with self.torch.no_grad():
            pred = self.model(**inputs).predicted_depth
        # the processor resizes without cropping or padding, so the prediction covers the whole frame
        d = self.torch.nn.functional.interpolate(pred[:, None], size=(size[1], size[0]), mode='bilinear',
                                                 align_corners=False)
        return d[0, 0].float().cpu().numpy()


def depth_confidence(depth, max_rel_jump):
    """2 where depth is smooth, 0 where it jumps by more than max_rel_jump between neighbouring
    pixels: the network blurs object edges, which would put points in mid-air."""
    ld = np.log(np.maximum(depth, 1e-3))
    jump = np.zeros_like(ld)
    dx, dy = np.abs(np.diff(ld, axis=1)), np.abs(np.diff(ld, axis=0))
    jump[:, 1:] = np.maximum(jump[:, 1:], dx)
    jump[:, :-1] = np.maximum(jump[:, :-1], dx)
    jump[1:, :] = np.maximum(jump[1:, :], dy)
    jump[:-1, :] = np.maximum(jump[:-1, :], dy)
    return np.where(jump < max_rel_jump, 2, 0).astype(np.uint8)


def colmap_intrinsics(camera):
    """3x3 K in this project's pixel convention (integer coordinates at pixel centres;
    COLMAP puts them at +0.5). Radial distortion is ignored."""
    p = np.asarray(camera.params, float)
    model = str(camera.model).split('.')[-1]
    if model in ('SIMPLE_PINHOLE', 'SIMPLE_RADIAL', 'RADIAL'):
        fx = fy = p[0]
        cx, cy = p[1], p[2]
    elif model in ('PINHOLE', 'OPENCV'):
        fx, fy, cx, cy = p[:4]
    else:
        raise ValueError(f'unsupported camera model {model}')
    return np.array([[fx, 0, cx - 0.5], [0, fy, cy - 0.5], [0, 0, 1.0]])


def _cam_from_world(image):
    cfw = image.cam_from_world
    cfw = cfw() if callable(cfw) else cfw
    M = np.eye(4)
    M[:3, :] = np.asarray(cfw.matrix())
    return M


def load_pieces(sparse_dir):
    """Every SfM model of a step-1 run: per model, {video_index: camera-to-world (SfM units)} and the model."""
    pieces = []
    for d in sorted(Path(sparse_dir).iterdir(), key=lambda p: int(p.name) if p.name.isdigit() else -1):
        if not d.is_dir() or not d.name.isdigit():
            continue
        rec = pycolmap.Reconstruction(str(d))
        poses = {int(Path(im.name).stem): invert(_cam_from_world(im)) for im in rec.images.values() if im.has_pose}
        if poses:
            pieces.append({'model': int(d.name), 'rec': rec, 'poses': poses})
    return pieces


def scale_samples(rec, depth_of, conf_of, K_depth, depth_scale, min_points):
    """Per registered image: median ratio of network depth to SfM depth at its 3D points."""
    out = {}
    for img in rec.images.values():
        if not img.has_pose:
            continue
        idx = int(Path(img.name).stem)
        if idx not in depth_of:
            continue
        uv, X = [], []
        for p2 in img.points2D:
            if p2.has_point3D():
                uv.append(p2.xy)
                X.append(rec.points3D[p2.point3D_id].xyz)
        if len(uv) < min_points:
            continue
        uv, X = np.asarray(uv, float) - 0.5, np.asarray(X, float)
        z = (X @ _cam_from_world(img)[:3, :3].T + _cam_from_world(img)[:3, 3])[:, 2]
        D, C = depth_of[idx], conf_of[idx]
        u = np.round((uv[:, 0] + 0.5) * depth_scale - 0.5).astype(int)
        v = np.round((uv[:, 1] + 0.5) * depth_scale - 0.5).astype(int)
        ok = (z > 0) & (u >= 0) & (u < D.shape[1]) & (v >= 0) & (v < D.shape[0])
        u, v, z = u[ok], v[ok], z[ok]
        good = C[v, u] > 0
        if good.sum() < min_points:
            continue
        out[idx] = float(np.median(D[v[good], u[good]] / z[good]))
    return out


def split_runs(piece_indices, order, max_gap):
    """Split a piece's frames into runs whose neighbours are at most max_gap selected frames apart."""
    pos = sorted(order[i] for i in piece_indices if i in order)
    runs, cur = [], []
    for p in pos:
        if cur and p - cur[-1] > max_gap + 1:
            runs.append(cur)
            cur = []
        cur.append(p)
    if cur:
        runs.append(cur)
    return runs


class FrameMatcher:
    """SIFT on downscaled frames; relative pose between two frames from matches and the first frame's depth."""

    def __init__(self, frame_paths, K_image, image_size, cfg):
        self.paths = frame_paths
        self.cfg = cfg
        self.s = cfg['match_width'] / image_size[0]
        self.K = scale_intrinsics(K_image, self.s)
        self.sift = cv2.SIFT_create(nfeatures=cfg['sift_features'])
        self.matcher = cv2.BFMatcher(cv2.NORM_L2)
        self.cache = {}

    def features(self, idx):
        if idx not in self.cache:
            img = cv2.imread(str(self.paths[idx]), cv2.IMREAD_GRAYSCALE)
            img = cv2.resize(img, (int(round(img.shape[1] * self.s)), int(round(img.shape[0] * self.s))),
                             interpolation=cv2.INTER_AREA)
            kp, desc = self.sift.detectAndCompute(img, None)
            self.cache[idx] = (np.array([k.pt for k in kp], float).reshape(-1, 2), desc)
        return self.cache[idx]

    def _sample(self, depth, uv):
        ds = depth.shape[1] / self.cfg['match_width']
        u = np.clip(np.round((uv[:, 0] + 0.5) * ds - 0.5).astype(int), 0, depth.shape[1] - 1)
        v = np.clip(np.round((uv[:, 1] + 0.5) * ds - 0.5).astype(int), 0, depth.shape[0] - 1)
        return v, u

    def relative(self, a, b, depth_a, conf_a, depth_b, conf_b):
        """(b_from_a, depth scale of b relative to a) or None, with match statistics.

        Translation is in units of depth_a. The scale is the factor that makes depth_b agree with
        a's matched points seen from b: the network's scale changes from frame to frame."""
        pa, da = self.features(a)
        pb, db = self.features(b)
        stats = {'matches': 0, 'inliers': 0}
        if da is None or db is None or len(pa) < 8 or len(pb) < 8:
            return None, stats
        pairs = self.matcher.knnMatch(da, db, k=2)
        good = [m[0] for m in pairs if len(m) == 2 and m[0].distance < self.cfg['ratio_test'] * m[1].distance]
        stats['matches'] = len(good)
        if len(good) < self.cfg['pnp_min_inliers']:
            return None, stats
        ia = np.array([m.queryIdx for m in good])
        ib = np.array([m.trainIdx for m in good])
        ua, ub = pa[ia], pb[ib]
        v, u = self._sample(depth_a, ua)
        d = depth_a[v, u]
        ok = (conf_a[v, u] > 0) & (d > self.cfg['min_m']) & (d < self.cfg['max_m'])
        if ok.sum() < self.cfg['pnp_min_inliers']:
            return None, stats
        P = backproject(ua[ok, 0], ua[ok, 1], d[ok], self.K, 'opencv')
        found, rvec, tvec, inl = cv2.solvePnPRansac(
            P.astype(np.float64), ub[ok].astype(np.float64), self.K, None, iterationsCount=1000,
            reprojectionError=self.cfg['pnp_reprojection_px'], confidence=0.999, flags=cv2.SOLVEPNP_EPNP)
        n_in = 0 if inl is None else len(inl)
        stats.update({'inliers': n_in, 'inlier_ratio': n_in / max(int(ok.sum()), 1)})
        if not found or n_in < self.cfg['pnp_min_inliers'] or stats['inlier_ratio'] < self.cfg['pnp_min_inlier_ratio']:
            return None, stats
        inl = inl.ravel()
        rvec, tvec = cv2.solvePnPRefineLM(P[inl].astype(np.float64), ub[ok][inl].astype(np.float64), self.K, None,
                                          rvec, tvec)
        T = np.eye(4)
        T[:3, :3] = cv2.Rodrigues(rvec)[0]
        T[:3, 3] = tvec.ravel()
        zb = (P[inl] @ T[:3, :3].T + T[:3, 3])[:, 2]
        vb, ub_ = self._sample(depth_b, ub[ok][inl])
        good = (conf_b[vb, ub_] > 0) & (depth_b[vb, ub_] > self.cfg['min_m']) & (zb > 0)
        stats['scale_points'] = int(good.sum())
        if good.sum() < self.cfg['pnp_min_inliers']:
            return None, stats
        r = zb[good] / depth_b[vb, ub_][good]
        scale = float(np.median(r))
        stats.update(depth_scale=scale, depth_scale_iqr_rel=float(np.subtract(*np.percentile(r, [75, 25])) / scale))
        if not 1 / self.cfg['link_max_scale_change'] <= scale <= self.cfg['link_max_scale_change']:
            stats['rejected'] = 'implausible depth scale change between neighbouring frames'
            return None, stats
        return (T, scale), stats


def extract_bridge_frames(video_path, selected, out_dir, image_size, max_gap, step):
    """Frames inside gaps of more than max_gap video frames between selected frames (step 1 drops
    blurred frames, so fast turns leave gaps that nothing can link across): every step-th frame,
    blurred or not, resized like the selected frames. Returns {video index: path}."""
    want = set()
    for a, b in zip(selected, selected[1:]):
        if b - a > max_gap:
            want.update(range(a + step, b - step // 2, step))
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = {}
    cap = cv2.VideoCapture(str(video_path))
    i, last = 0, max(want, default=-1)
    while i <= last:
        ok, frame = cap.read()
        if not ok:
            break
        if i in want:
            p = out_dir / f'{i:06d}.jpg'
            cv2.imwrite(str(p), cv2.resize(frame, image_size, interpolation=cv2.INTER_AREA),
                        [cv2.IMWRITE_JPEG_QUALITY, 95])
            paths[i] = p
        i += 1
    cap.release()
    return paths


def _rot_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def _scaled(T, s):
    T = T.copy()
    T[:3, 3] *= s
    return T


def link_frames(seq, runs, relative, cfg):
    """How each frame is placed: start of a component, inside an SfM run already reached in this
    component, or linked to the last placed frame. After max_bridge_frames failed links in a row
    the trajectory starts a new component.

    seq: video indices in order. runs: dicts with 'run_id', 'frames', 'accepted'.
    relative(a, b) -> (b_from_a, depth scale of b relative to a) or None, both computed with a's
    network depth as is.
    """
    run_of = {i: r for r in runs if r['accepted'] for i in r['frames']}
    reached = {}                                      # run id -> component where the chain reached it
    rows, prev, comp, misses = [], None, 0, 0
    placed = []                                       # video indices placed in the current component
    for idx in seq:
        r = run_of.get(idx)
        row = {'video_index': idx, 'run': r['run_id'] if r is not None else None}
        if r is not None and reached.get(r['run_id']) == comp:
            row['source'] = 'sfm_run'
        elif prev is None:
            row['source'] = 'component_start'
        else:
            rel = relative(prev, idx)
            if rel is None:
                misses += 1
                if misses > cfg['max_bridge_frames']:
                    prev, comp, misses, placed = None, comp + 1, 0, []
                continue
            row.update(source='depth_link', parent=prev, rel=rel)
            # redundant links from earlier frames: a wrong link then disagrees with its neighbours
            # (a single link cannot be checked: real network scale jumps look the same)
            extra = []
            for a in placed[-1 - cfg['extra_links']:-1][::-1]:
                rel2 = relative(a, idx)
                if rel2 is not None:
                    extra.append((a, rel2[1]))
            row['extra'] = extra
        if r is not None and r['run_id'] not in reached:
            reached[r['run_id']] = comp
        elif r is not None and reached[r['run_id']] != comp:
            row['run'] = None                         # run reached in an earlier component: frame is linked
        row['component'] = comp
        rows.append(row)
        placed.append(idx)
        prev, misses = idx, 0
    return rows


def solve_depth_corrections(rows, runs, cfg):
    """Depth correction per frame (network depth x correction = depth in one unit per component).

    Least squares in log scale per component, so errors do not multiply along the chain:
      link a -> b:        log k_b - log k_a = log(link scale)      sigma scale_sigma_link (Cauchy;
                          parent link and extra links from earlier frames)
      consecutive frames
      of one SfM run:     log k_h - log k_g = log(r_g / r_h)       sigma scale_sigma_run
                          (r = network / SfM depth: k = run scale / r)
      every frame:        log k = 0                                sigma scale_sigma_prior
    The prior is the network's own scale: loose, it only stops the chain from drifting."""
    by_id = {r['run_id']: r for r in runs}
    k, info = {}, {}
    comps = sorted({row['component'] for row in rows})
    for comp in comps:
        cr = [row for row in rows if row['component'] == comp]
        pos = {row['video_index']: n for n, row in enumerate(cr)}
        cons = []                                     # (i, j, value, sigma, robust): log k_j - log k_i = value
        last_in_run = {}
        for row in cr:
            j = pos[row['video_index']]
            if row['source'] == 'depth_link':
                cons.append((pos[row['parent']], j, float(np.log(row['rel'][1])), cfg['scale_sigma_link'], True))
                for a, kb in row.get('extra', []):
                    cons.append((pos[a], j, float(np.log(kb)), cfg['scale_sigma_link'], True))
            if row['run'] is not None:
                run = by_id[row['run']]
                prev = last_in_run.get(row['run'])
                if prev is not None:
                    rg = run['ratio'].get(cr[prev]['video_index'], run['ratio_median'])
                    rh = run['ratio'].get(row['video_index'], run['ratio_median'])
                    cons.append((prev, j, float(np.log(rg / rh)), cfg['scale_sigma_run'], False))
                last_in_run[row['run']] = j
        n = len(cr)
        w_rob = np.ones(len(cons))
        for _ in range(cfg['scale_irls_iterations']):
            A = np.eye(n) / cfg['scale_sigma_prior'] ** 2
            b = np.zeros(n)
            for (i, j, v, s, _), wr in zip(cons, w_rob):
                w = wr / s ** 2
                A[i, i] += w
                A[j, j] += w
                A[i, j] -= w
                A[j, i] -= w
                b[i] -= w * v
                b[j] += w * v
            x = np.linalg.solve(A, b)
            res = np.array([(x[j] - x[i] - v) / s for i, j, v, s, _ in cons])
            # Cauchy weights: a link's pull shrinks as its residual grows, so one wrong link
            # cannot bend the chain (Huber still pulls with constant force)
            c = cfg['scale_robust_c']
            w_rob = np.array([1.0 / (1.0 + (e / c) ** 2) if rob else 1.0 for e, (*_, rob) in zip(res, cons)])
        for row, xi in zip(cr, x):
            k[row['video_index']] = float(np.exp(xi))
        robust = np.array([c[4] for c in cons], bool)
        info[comp] = {'frames': n, 'constraints': len(cons),
                      'links_downweighted': int((w_rob[robust] < 0.5).sum()) if robust.any() else 0,
                      'link_residual_sigma_p50_p90': (np.percentile(np.abs(res[robust]), [50, 90]).tolist()
                                                      if robust.any() else None)}
    return k, info


def compose_poses(rows, runs, k):
    """Poses from links and runs with the solved depth corrections. A run's scale is the median of
    k x (network / SfM depth) over its frames; its frame is fixed by its first frame."""
    by_id = {r['run_id']: r for r in runs}
    run_scale, anchor, poses = {}, {}, {}
    for row in rows:
        if row['run'] is not None and row['run'] not in run_scale:
            run = by_id[row['run']]
            members = [x['video_index'] for x in rows if x['run'] == row['run']]
            run_scale[row['run']] = float(np.median([k[i] * run['ratio'].get(i, run['ratio_median'])
                                                     for i in members]))
    for row in rows:
        idx = row['video_index']
        if row['source'] == 'sfm_run':
            run = by_id[row['run']]
            poses[idx] = anchor[row['run']] @ _scaled(run['poses'][idx], run_scale[row['run']])
            continue
        if row['source'] == 'component_start':
            poses[idx] = np.eye(4)
        else:
            T, _ = row['rel']
            poses[idx] = poses[row['parent']] @ invert(_scaled(T, k[row['parent']]))
        if row['run'] is not None:
            run = by_id[row['run']]
            anchor[row['run']] = poses[idx] @ invert(_scaled(run['poses'][idx], run_scale[row['run']]))
    for r in runs:
        if r['run_id'] in run_scale:
            r['scale'] = run_scale[r['run_id']]
    return poses


def chain_trajectory(seq, runs, relative, cfg):
    """One pose and one depth correction per frame, one unit of length per component.

    runs: dicts with 'run_id', 'frames', 'poses' (camera-to-world, SfM units), 'ratio'
    ({video index: network depth / SfM depth}), 'ratio_median' and 'accepted'."""
    rows = link_frames(seq, runs, relative, cfg)
    k, info = solve_depth_corrections(rows, runs, cfg)
    poses = compose_poses(rows, runs, k)
    out = [{key: v for key, v in row.items() if key != 'rel'} | {'depth_correction': k[row['video_index']]}
           for row in rows]
    return poses, k, out, info


def estimate_up(R_c2w):
    """Up from camera x axes (horizontal when the phone is held upright). Returns up and how well
    the x axes constrain it (second / first singular value: near 0 means one viewing direction only)."""
    X = np.array([R[:, 0] for R in R_c2w])
    _, s, vt = np.linalg.svd(X, full_matrices=False)
    up = vt[2]
    cam_up = -np.array([R[:, 1] for R in R_c2w]).mean(axis=0)    # OpenCV camera y points down
    if up @ cam_up < 0:
        up = -up
    return up, {'x_axis_spread': float(s[1] / s[0]), 'x_axis_residual': float(s[2] / s[0])}


class VideoFrames:
    """Stand-in for ingestion.rgbd.RGBDCapture built from video frames and network depth, so the
    shared stages (fusion, floor plan, openings) run unchanged. Depth is served in millimetres."""

    def __init__(self, capture_id, video_indices, pts_s, depth_of, conf_of, K_depth, depth_size):
        self.capture_id = capture_id
        self.frames = np.asarray(video_indices, int)
        self.timestamps = np.asarray(pts_s, float)
        self.depth_size = depth_size
        self._depth, self._conf, self._K = depth_of, conf_of, K_depth

    def __len__(self):
        return len(self.frames)

    def depth_intrinsics(self, i):
        return self._K

    def load_depth_raw(self, i):
        return np.round(self._depth[int(self.frames[i])] * 1000.0).clip(0, 65535).astype(np.uint16)

    def load_confidence(self, i):
        return self._conf[int(self.frames[i])]
