from pathlib import Path

import numpy as np
import yaml

from property_capture.rooms.floorplan import extract_room, polygon_area
from property_capture.rooms.walls import (detect_wall_segments, group_collinear, is_simple_polygon,
                                          snap_polygon, tall_cells)

with open(Path(__file__).resolve().parents[2] / 'configs' / 'default.yaml', encoding='utf-8') as f:
    CFG = yaml.safe_load(f)['floorplan']
WCFG = CFG['walls']


def _room_cells(w, d, rng, door=None, step=0.04, jitter=0.01):
    """Tall-cell-like 2D points on a w x d rectangle; optional door gap (x0, x1) in the y=0 wall."""
    xs, ys = np.arange(0, w + step / 2, step), np.arange(0, d + step / 2, step)
    pts = np.vstack([np.column_stack([xs, np.zeros_like(xs)]), np.column_stack([xs, np.full_like(xs, d)]),
                     np.column_stack([np.zeros_like(ys), ys]), np.column_stack([np.full_like(ys, w), ys])])
    if door:
        pts = pts[~((pts[:, 1] < 1e-9) & (pts[:, 0] > door[0]) & (pts[:, 0] < door[1]))]
    pts = pts + rng.normal(scale=jitter, size=pts.shape)
    floor = rng.uniform([0.1, 0.1], [w - 0.1, d - 0.1], size=(20000, 2))
    return pts, floor


def test_tall_cells_keeps_walls_and_drops_low_furniture():
    rng = np.random.default_rng(0)
    wall = np.column_stack([rng.uniform(0, 2, 3000), np.zeros(3000)])
    wall_h = rng.uniform(0.3, 2.0, 3000)
    table = np.column_stack([rng.uniform(0, 1, 3000), rng.uniform(1, 2, 3000)])
    table_h = rng.uniform(0.7, 0.8, 3000)
    centres, stats = tall_cells(np.vstack([wall, table]), np.r_[wall_h, table_h], 0.3, 2.0, WCFG)
    assert stats['tall_cells'] > 0
    assert np.all(np.abs(centres[:, 1]) < 0.05)


def test_detects_four_walls_with_door_split():
    rng = np.random.default_rng(1)
    pts, _ = _room_cells(4.0, 3.0, rng, door=(1.0, 1.9))
    segs = detect_wall_segments(pts, WCFG, rng)
    walls = group_collinear(segs, WCFG)
    assert len(walls) == 4
    door_wall = [w for w in walls if abs(w['direction'][0]) > 0.99 and abs(w['centre'][1]) < 0.1]
    assert len(door_wall) == 1 and len(door_wall[0]['segments']) == 2
    assert all(seg['rms_m'] < 0.02 for seg in segs)


def test_snapped_polygon_is_the_rectangle():
    rng = np.random.default_rng(2)
    pts, floor = _room_cells(4.0, 3.0, rng, door=(1.0, 1.9))
    room = extract_room(pts, floor, CFG, wall_min_points=1)
    walls = group_collinear(detect_wall_segments(pts, WCFG, rng), WCFG)
    snapped = snap_polygon(room['contour_m'], walls, WCFG)
    poly = snapped['polygon']
    assert snapped['simple']
    assert len(poly) == 4
    assert abs(polygon_area(poly) - 12.0) < 0.15
    lengths = sorted(np.linalg.norm(np.roll(poly, -1, axis=0) - poly, axis=1))
    assert np.allclose(lengths, [3, 3, 4, 4], atol=0.04)
    assert len(snapped['gap_candidates']) == 1
    assert abs(snapped['gap_candidates'][0]['width_m'] - 0.9) < 0.12


def _hidden_corner(radius, seed):
    rng = np.random.default_rng(seed)
    pts, floor = _room_cells(4.0, 3.0, rng)
    corner = np.array([4.0, 3.0])
    pts = pts[np.linalg.norm(pts - corner, axis=1) > 0.4]
    floor = floor[np.linalg.norm(floor - corner, axis=1) > radius]
    room = extract_room(pts, floor, CFG, wall_min_points=1)
    walls = group_collinear(detect_wall_segments(pts, WCFG, rng), WCFG)
    return snap_polygon(room['contour_m'], walls, WCFG), corner


def test_hidden_corner_is_filled_by_extending_walls():
    snapped, corner = _hidden_corner(0.5, 4)
    assert snapped['simple'] and len(snapped['polygon']) == 4
    assert abs(polygon_area(snapped['polygon']) - 12.0) < 0.15
    assert len(snapped['corner_fills']) == 1
    assert np.allclose(snapped['corner_fills'][0]['corner'], corner, atol=0.03)


def test_large_unexplained_region_is_not_filled():
    snapped, _ = _hidden_corner(1.2, 5)
    assert snapped['corner_fills'] == []
    assert polygon_area(snapped['polygon']) < 11.5


def test_is_simple_polygon():
    square = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], float)
    bowtie = np.array([[0, 0], [1, 1], [1, 0], [0, 1]], float)
    assert is_simple_polygon(square)
    assert not is_simple_polygon(bowtie)
