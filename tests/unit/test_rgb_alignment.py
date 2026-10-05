from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from property_capture.calibration.rgb_alignment import (assess, best_shift, depth_edges, depth_pixel_to_rgb,
                                                        image_gradient, select_frames, select_moving_frames,
                                                        shift_grid)
from property_capture.ingestion.video import align_video_to_odometry

with open(Path(__file__).resolve().parents[2] / 'configs' / 'default.yaml', encoding='utf-8') as f:
    RCFG = yaml.safe_load(f)['rgb']

SIZE = (512, 384)


def _scene():
    """Depth map (256x192) with boxes at different distances, and full confidence."""
    d = np.full((192, 256), 3.0)
    d[40:120, 30:110] = 1.5
    d[100:170, 150:230] = 2.2
    d[20:60, 170:240] = 1.0
    return d, np.full(d.shape, 2, np.uint8)


def _image_from_depth(d, shift):
    """Grayscale image whose intensity edges follow the depth boxes, shifted by (dx, dy) working px."""
    img = cv2.resize((d * 60).astype(np.float32), SIZE, interpolation=cv2.INTER_NEAREST)
    M = np.float32([[1, 0, shift[0]], [0, 1, shift[1]]])
    img = cv2.warpAffine(img, M, SIZE, flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
    return np.clip(img + np.random.default_rng(0).normal(0, 2, img.shape), 0, 255).astype(np.uint8)


@pytest.mark.parametrize('shift', [(0.0, 0.0), (2.0, -1.5), (-3.0, 1.0)])
def test_best_shift_recovers_known_offset(shift):
    d, c = _scene()
    edges = depth_edges(d, c, SIZE, RCFG)
    assert edges.sum() > RCFG['min_edge_pixels']
    grid = shift_grid(RCFG['max_shift_px'] * 2, RCFG['shift_step_px'] * 2)
    found, score, _ = best_shift(edges, image_gradient(_image_from_depth(d, shift)), grid)
    assert np.allclose(found, shift, atol=0.5)
    assert score > 1.5                                     # edges much stronger than average


@pytest.mark.parametrize('shift', [(1.3, 0.0), (0.0, -0.6), (-0.4, 0.7)])
def test_sub_pixel_shift(shift):
    d, c = _scene()
    edges = depth_edges(d, c, SIZE, RCFG)
    grid = shift_grid(RCFG['max_shift_px'] * 2, RCFG['shift_step_px'] * 2)
    found, _, _ = best_shift(edges, image_gradient(_image_from_depth(d, shift)), grid)
    # depth edges are ~2 working px wide, so the score peak is flat-topped: resolution is about
    # +/-0.5 working px = +/-0.25 depth px, half the 0.5 depth px alignment threshold
    assert np.allclose(found, shift, atol=0.5)


def test_moving_frames_come_from_the_moderate_rotation_band():
    T = np.tile(np.eye(4), (300, 1, 1))
    rates = np.linspace(0.0005, 0.03, 300)                  # rotation speed increasing over the capture
    yaw = np.cumsum(rates)
    T[:, 0, 0], T[:, 0, 2], T[:, 2, 0], T[:, 2, 2] = np.cos(yaw), np.sin(yaw), -np.sin(yaw), np.cos(yaw)
    picks = select_moving_frames(T, list(range(0, 300, 3)), 10, RCFG)
    lo, hi = RCFG['temporal_motion_percentiles']
    assert picks and all(lo / 100 * 300 - 6 <= p <= hi / 100 * 300 + 6 for p in picks)


def _frames(dx):
    return [{'shift_px': [2 * dx, 0.0], 'score': 3.0} for _ in range(10)]   # working px = 2 x depth px


@pytest.mark.parametrize('dx, quad, status', [
    (0.2, {'a': [0.4, 0], 'b': [0.5, 0]}, 'aligned'),
    (-0.8, {'a': [-1.6, 0], 'b': [-1.5, 0]}, 'constant_offset'),      # same offset in every quadrant
    (-0.5, {'a': [-1.0, 0], 'b': [1.5, 0]}, 'inconsistent'),          # quadrants disagree: scale error
    (-3.0, {'a': [-6.0, 0], 'b': [-6.0, 0]}, 'inconsistent'),         # too large to be a correctable offset
])
def test_alignment_status(dx, quad, status):
    out = assess(_frames(dx), quad, [], RCFG, px_scale=2)
    assert out['status'] == status
    assert np.isclose(out['depth_to_rgb_offset_px'][0], dx)


def test_depth_pixel_to_rgb_uses_pixel_centres_and_offset():
    u, v = depth_pixel_to_rgb(127.5, 95.5, (256, 192), (1920, 1440))
    assert np.isclose(u, 959.5) and np.isclose(v, 719.5)             # image centre maps to image centre
    u2, _ = depth_pixel_to_rgb(127.5, 95.5, (256, 192), (1920, 1440), offset_depth_px=(-0.5, 0.0))
    assert np.isclose(u2, 959.5 - 0.5 * 7.5)


def test_video_pairing_finds_missing_first_frame():
    rng = np.random.default_rng(1)
    dt = np.where(rng.random(400) < 0.25, 2, 1) / 60.0     # 60 Hz clock with dropped frames
    t_odo = 100 + np.r_[0, np.cumsum(dt)]
    pts = t_odo[1:] - t_odo[1]                             # video lacks the first frame
    a = align_video_to_odometry(pts, t_odo)
    assert a['best_offset'] == 1 and a['best_is_exact'] and a['best_is_unique']


def test_select_frames_prefers_low_motion():
    T = np.tile(np.eye(4), (300, 1, 1))
    T[:, 0, 3] = np.cumsum(np.where(np.arange(300) < 150, 0.05, 0.001))   # fast first half, slow second
    picks = select_frames(T, list(range(0, 300, 3)), 10, RCFG)
    assert len(picks) == 10 and min(picks) >= 150          # frame 150's neighbours 149 and 151 move 2 mm
