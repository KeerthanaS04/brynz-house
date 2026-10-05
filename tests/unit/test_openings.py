from pathlib import Path

import numpy as np
import pytest
import yaml

from property_capture.rooms.openings import WallGrid

with open(Path(__file__).resolve().parents[2] / 'configs' / 'default.yaml', encoding='utf-8') as f:
    OCFG = yaml.safe_load(f)['openings']


def _scene_rays(rng, holes, cams, n=6000, beyond=2.0):
    """Wall from (4, 0) to (4, 4) (room on the x < 4 side, so the wall runs +y and its outward normal is +x).
    holes: list of (y0, y1, z0, z1). Rays from each camera to random wall points; rays inside a hole
    continue `beyond` metres past the wall."""
    frames = []
    for cam in cams:
        y = rng.uniform(0, 4, n)
        z = rng.uniform(0, 2.6, n)
        through = np.zeros(n, bool)
        for y0, y1, z0, z1 in holes:
            through |= (y > y0) & (y < y1) & (z > z0) & (z < z1)
        c2, ch = np.array(cam[:2]), cam[2]
        wall_pt = np.column_stack([np.full(n, 4.0), y])
        # extend through-rays from the camera past the wall point
        d = wall_pt - c2
        scale = np.where(through, 1 + beyond / np.abs(d[:, 0]), 1.0)
        p2 = c2 + d * scale[:, None]
        ph = ch + (z - ch) * scale
        frames.append((c2, ch, p2, ph))
    return frames


def _grid(frames):
    g = WallGrid([4.0, 0.0], [4.0, 4.0], 2.6, OCFG)
    for c2, ch, p2, ph in frames:
        g.accumulate(c2, ch, p2, ph)
    return g


CAMS = [(1.5, 1.0, 1.4), (2.0, 2.5, 1.5), (1.0, 3.5, 1.3), (2.5, 1.8, 1.4)]


def test_door_is_found_with_width_sill_and_head():
    rng = np.random.default_rng(0)
    g = _grid(_scene_rays(rng, [(1.0, 1.9, 0.0, 2.05)], CAMS))
    ops = [o for o in g.openings() if o['bounded']]
    assert len(ops) == 1
    o = ops[0]
    assert o['type'] == 'door'
    assert abs(o['width_m'] - 0.90) <= 0.02                  # within the PDF opening-width tolerance
    assert o['sill_m'] <= 0.02 and abs(o['head_m'] - 2.05) <= 0.04
    assert abs(o['centre'][1] - 1.45) < 0.03


def test_window_is_classified_by_sill():
    rng = np.random.default_rng(1)
    g = _grid(_scene_rays(rng, [(2.4, 3.6, 0.9, 2.1)], CAMS))
    ops = [o for o in g.openings() if o['bounded']]
    assert len(ops) == 1 and ops[0]['type'] == 'window'
    assert abs(ops[0]['width_m'] - 1.2) <= 0.02 and abs(ops[0]['sill_m'] - 0.9) <= 0.04


def test_unseen_wall_is_not_an_opening():
    # no rays at all reach y in [1, 2]: unseen, not seen-through
    rng = np.random.default_rng(2)
    frames = []
    for c2, ch, p2, ph in _scene_rays(rng, [], CAMS):
        keep = (p2[:, 1] < 1.0) | (p2[:, 1] > 2.0)
        frames.append((c2, ch, p2[keep], ph[keep]))
    assert _grid(frames).openings() == []


def test_open_end_without_jamb_is_not_bounded():
    # the wall's last metre is open: the room continues, no jamb beyond it
    rng = np.random.default_rng(3)
    ops = _grid(_scene_rays(rng, [(3.0, 4.01, 0.0, 2.6)], CAMS)).openings()
    assert ops and not any(o['bounded'] for o in ops)


def test_door_at_the_end_of_an_edge_uses_the_jamb_beyond_it():
    # the outline edge stops at y = 2.0 (wall snapping split it at the doorway), but the wall continues
    rng = np.random.default_rng(4)
    frames = _scene_rays(rng, [(1.4, 2.1, 0.0, 2.05)], CAMS)
    g = WallGrid([4.0, 0.0], [4.0, 2.0], 2.6, OCFG)
    for c2, ch, p2, ph in frames:
        g.accumulate(c2, ch, p2, ph)
    ops = g.openings()
    assert len(ops) == 1 and ops[0]['bounded'] and ops[0]['type'] == 'door'
    assert abs(ops[0]['width_m'] - 0.70) <= 0.02
    assert ops[0]['s_end_m'] > 2.0                           # extends onto the next stretch of wall


def test_floor_to_ceiling_opening_is_a_passage():
    rng = np.random.default_rng(5)
    ops = [o for o in _grid(_scene_rays(rng, [(1.0, 2.2, 0.0, 2.7)], CAMS)).openings() if o['bounded']]
    assert len(ops) == 1 and ops[0]['type'] == 'passage'


def test_camera_outside_the_room_side_is_ignored():
    g = WallGrid([4.0, 0.0], [4.0, 4.0], 2.6, OCFG)
    g.accumulate(np.array([5.0, 1.0]), 1.4, np.array([[3.0, 1.0]]), np.array([1.0]))
    assert g.wall.sum() == 0 and g.through.sum() == 0
