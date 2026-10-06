import ast
import re
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

from property_capture.geometry.transforms import invert
from property_capture.modalities.video_depth import (VideoFrames, chain_trajectory, colmap_intrinsics,
                                                     depth_confidence, estimate_up, extract_bridge_frames,
                                                     solve_depth_corrections, split_runs)
from property_capture.reconstruction.fusion import fuse

SRC = Path(__file__).resolve().parents[2] / 'src' / 'property_capture'


def _imports(path):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    return {n.module or '' for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)} | \
           {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}


def test_video_plan_never_reads_device_data():
    """CLAUDE_updated.md rule 6: video tier code must not touch device depth, poses or IMU."""
    for name in ('modalities/video_depth.py', 'video_pipeline.py', 'plan_outputs.py'):
        path = SRC / name
        imports = _imports(path)
        assert not any('ingestion' in m or 'registration' in m or 'video_oracle' in m or m.endswith('evaluate')
                       for m in imports), (name, imports)
        text = path.read_text(encoding='utf-8').split('"""', 2)[2]
        for word in ('odometry', 'imu', 'camera_matrix', 'load_capture'):
            assert not re.search(rf'\b{word}', text, re.IGNORECASE), (name, word)


def test_depth_confidence_drops_edges_only():
    d = np.full((20, 30), 2.0)
    d[:, 15:] = 3.0                                    # 50% jump between columns 14 and 15
    d[:, :15] += np.linspace(0, 0.05, 15)              # gentle slope: kept
    c = depth_confidence(d, 0.05)
    assert (c[:, 14:16] == 0).all()
    assert (c[:, :14] == 2).all() and (c[:, 16:] == 2).all()


def test_colmap_intrinsics_moves_principal_point_half_pixel():
    cam = SimpleNamespace(model='CameraModelId.SIMPLE_RADIAL', params=[1000.0, 800.0, 600.0, 0.01])
    K = colmap_intrinsics(cam)
    assert np.allclose(K, [[1000, 0, 799.5], [0, 1000, 599.5], [0, 0, 1]])


def test_split_runs_breaks_at_gaps():
    order = {v: n for n, v in enumerate(range(0, 200, 10))}   # selected frames 0, 10, ..., 190
    piece = {0: None, 10: None, 20: None, 50: None, 60: None, 150: None}
    assert split_runs(piece, order, max_gap=2) == [[0, 1, 2, 5, 6], [15]]
    assert split_runs(piece, order, max_gap=1) == [[0, 1, 2], [5, 6], [15]]


SOLVE = {'scale_sigma_link': 1e-4, 'scale_sigma_run': 1e-4, 'scale_sigma_prior': 10.0, 'scale_robust_c': 2.0,
         'extra_links': 2, 'scale_irls_iterations': 3}


def test_scale_solve_does_not_let_one_bad_link_drift_the_chain():
    """Links say each frame's scale equals the previous one, except one wrong link (x1.5). Extra
    links from the two frames before the parent disagree with it, so the solve rejects it."""
    n = 60
    rows = [{'video_index': 0, 'source': 'component_start', 'run': None, 'component': 0}]
    for i in range(1, n):
        kb = 1.5 if i == 30 else 1.0
        rows.append({'video_index': i, 'source': 'depth_link', 'parent': i - 1, 'rel': (np.eye(4), kb),
                     'extra': [(a, 1.0) for a in (i - 2, i - 3) if a >= 0], 'run': None, 'component': 0})
    cfg = {'scale_sigma_link': 0.03, 'scale_sigma_run': 0.01, 'scale_sigma_prior': 0.25, 'scale_robust_c': 2.0,
           'scale_irls_iterations': 5}
    k, info = solve_depth_corrections(rows, [], cfg)
    ks = np.array([k[i] for i in range(n)])
    assert np.abs(np.log(ks)).max() < 0.05                       # chaining alone would end at 1.5
    assert info[0]['links_downweighted'] >= 1


def _pose(yaw, pos):
    c, s = np.cos(yaw), np.sin(yaw)
    T = np.eye(4)
    T[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    T[:3, 3] = pos
    return T


def test_chain_joins_runs_with_links_and_one_depth_scale():
    gt = {i: _pose(0.3 * i, [0.2 * i, 0, 0.05 * i * i]) for i in range(10)}
    gt = {i: invert(gt[0]) @ T for i, T in gt.items()}          # trajectory starts at the identity
    # network depth of frame i is net[i] x true depth: its scale changes from frame to frame
    net = {i: 1.0 + 0.1 * ((-1) ** i) * (i % 3) for i in range(10)}
    # a run of frames 3..6 in its own frame (rotated, shifted) and SfM units (x 0.37)
    W, s_sfm = _pose(1.1, [5, 2, -3]), 0.37
    sfm = {i: W @ gt[i] for i in (3, 4, 5, 6)}
    for T in sfm.values():
        T[:3, 3] *= s_sfm
    run = {'run_id': 'r0', 'frames': [3, 4, 5, 6], 'accepted': True, 'poses': sfm,
           'ratio': {i: net[i] / s_sfm for i in (3, 4, 5, 6)}, 'ratio_median': 1 / s_sfm}
    calls = []

    def relative(a, b):
        """b_from_a with translation in units of a's network depth, and b's depth scale relative to a."""
        calls.append((a, b))
        if (a, b) == (7, 8):
            return None                                         # one failed link: frame 8 is skipped
        T = invert(gt[b]) @ gt[a]
        T[:3, 3] *= net[a]
        return T, net[a] / net[b]

    cfg = dict(SOLVE, max_bridge_frames=3)
    poses, corr, rows, _ = chain_trajectory(list(range(10)), [run], relative, cfg)
    assert sorted(poses) == [0, 1, 2, 3, 4, 5, 6, 7, 9]
    # one unit for the whole trajectory: corrected depth = c x true depth for every frame
    c = corr[0] * net[0]
    for i, T in poses.items():
        expected = gt[i].copy()
        expected[:3, 3] *= c
        assert np.allclose(T, expected, atol=1e-6), i
        assert np.isclose(corr[i] * net[i], c, rtol=1e-6), i
    assert {r['video_index']: r['source'] for r in rows}[5] == 'sfm_run'
    assert (4, 5) not in calls and (7, 9) in calls              # run frames use SfM; 9 links back to 7


def test_chain_starts_new_component_after_failures():
    gt = {i: _pose(0, [0.1 * i, 0, 0]) for i in range(8)}
    poses, corr, rows, _ = chain_trajectory(
        list(range(8)), [], lambda a, b: None if b in (3, 4) else (invert(gt[b]) @ gt[a], 1.0),
        dict(SOLVE, max_bridge_frames=1))
    # 3 fails (1 miss allowed), 4 fails too: the trajectory restarts at 5 in a new component
    assert [(r['video_index'], r['component']) for r in rows] == [(0, 0), (1, 0), (2, 0), (5, 1), (6, 1), (7, 1)]
    assert np.allclose(poses[5], np.eye(4)) and np.allclose(poses[7], invert(gt[5]) @ gt[7])


def test_estimate_up_from_camera_x_axes():
    up_true = np.array([0.1, -0.98, 0.17])
    up_true /= np.linalg.norm(up_true)
    rng = np.random.default_rng(0)
    Rs = []
    for yaw in np.linspace(0, 2 * np.pi, 24, endpoint=False):
        e1 = np.cross(up_true, [0, 0, 1.0])
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(up_true, e1)
        x = np.cos(yaw) * e1 + np.sin(yaw) * e2                 # horizontal camera x axis
        pitch = rng.uniform(-0.4, 0.4)
        fwd = np.cos(pitch) * np.cross(up_true, x) + np.sin(pitch) * up_true
        y = np.cross(fwd, x)                                    # OpenCV: y down, z forward
        Rs.append(np.column_stack([x, y, fwd]))
    up, info = estimate_up(Rs)
    assert np.degrees(np.arccos(up @ up_true)) < 0.5 and info['x_axis_spread'] > 0.5


def test_bridge_frames_fill_long_gaps_only(tmp_path):
    video = tmp_path / 'v.mp4'
    w = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'mp4v'), 30, (64, 48))
    for i in range(60):
        w.write(np.full((48, 64, 3), i * 4, np.uint8))
    w.release()
    paths = extract_bridge_frames(video, [0, 5, 10, 40, 45], tmp_path / 'b', (32, 24), max_gap=10, step=3)
    assert sorted(paths) == [13, 16, 19, 22, 25, 28, 31, 34, 37]         # only inside 10 -> 40
    assert cv2.imread(str(paths[13])).shape == (24, 32, 3)


def test_video_frames_feed_fusion():
    K = np.array([[100.0, 0, 31.5], [0, 100.0, 23.5], [0, 0, 1]])
    depth = {7: np.full((48, 64), 2.0)}                         # a wall 2 m in front of the camera
    vf = VideoFrames('cap', [7], [0.25], depth, {7: np.full((48, 64), 2, np.uint8)}, K, (64, 48))
    assert vf.load_depth_raw(0).dtype == np.uint16 and vf.load_depth_raw(0)[0, 0] == 2000
    out = fuse(vf, np.eye(4)[None], 'opencv', [0], {'unit_scale_m': 0.001, 'min_m': 0.2, 'max_m': 5.0,
                                                    'confidence_min': 1},
               {'pixel_step': 1, 'voxel_size_m': 0.05, 'min_points_per_voxel': 1})
    assert np.allclose(out['points'][:, 2], 2.0, atol=0.03)
