from pathlib import Path

import numpy as np
import yaml

from property_capture.rooms.shared_walls import pair_shared_walls
from property_capture.rooms.wall_alignment import _area, align_shared_walls

with open(Path(__file__).resolve().parents[2] / 'configs' / 'default.yaml', encoding='utf-8') as f:
    SCFG = yaml.safe_load(f)['floorplan']['shared_walls']


def _rect(x0, y0, x1, y1, rid):
    return {'room_id': rid, 'polygon': np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], float)}


def _skewed_pair():
    """A's right face is vertical at x=4; B's left face runs from x=4.10 to x=4.18 over 3 m (1.5 deg)."""
    a = _rect(0, 0, 4, 3, 'A')
    b = {'room_id': 'B', 'polygon': np.array([[4.10, 0], [7, 0], [7, 3], [4.18, 3]], float)}
    return a, b


def test_skewed_faces_become_parallel_and_measurable():
    a, b = _skewed_pair()
    walls = pair_shared_walls([a, b], SCFG)
    assert walls[0]['thickness_m'] is None                     # 8 cm divergence > 4 cm
    area_a, area_b = _area(a['polygon']), _area(b['polygon'])
    report = align_shared_walls([a, b], walls, SCFG)
    assert report[0]['status'] == 'aligned' and 0.5 < report[0]['max_rotation_deg'] < 1.0
    after = pair_shared_walls([a, b], SCFG)[0]
    assert after['thickness_status'] == 'measured'
    assert after['thickness_variation_m'] < 1e-6
    assert np.isclose(after['thickness_m'], 0.14, atol=0.005)  # mean gap (10 -> 18 cm) preserved
    assert abs(_area(a['polygon']) - area_a) < 0.02 * area_a and abs(_area(b['polygon']) - area_b) < 0.02 * area_b


def test_rotation_between_parallel_neighbours_keeps_area_exactly():
    a, b = _skewed_pair()
    area_b = _area(b['polygon'])
    align_shared_walls([a, b], pair_shared_walls([a, b], SCFG), SCFG)
    assert np.isclose(_area(b['polygon']), area_b)            # the two corner triangles cancel


def test_alignment_exceeding_the_area_limit_is_skipped_and_unchanged():
    a, b = _skewed_pair()
    b['polygon'][1] = [7, -0.6]                                # sloped bottom edge: rotation now changes area
    before = (a['polygon'].copy(), b['polygon'].copy())
    report = align_shared_walls([a, b], pair_shared_walls([a, b], SCFG), {**SCFG, 'align_max_area_change': 0.0})
    assert report[0]['status'] == 'skipped' and 'area would change' in report[0]['reason']
    assert np.allclose(a['polygon'], before[0]) and np.allclose(b['polygon'], before[1])


def test_alignment_that_would_overlap_another_room_is_skipped():
    a, b = _skewed_pair()
    # room C sits right of B's top-left corner region; rotating B's face moves that corner left (x 4.18 -> ~4.14)
    # and down the top edge, so make C occupy the strip B would grow into
    c = {'room_id': 'C', 'polygon': np.array([[4.10, 3.0], [4.18, 3.0], [4.14, 2.0]], float)}
    b['polygon'] = np.array([[4.10, 0], [7, 0], [7, 3], [4.18, 3]], float)
    walls = [w for w in pair_shared_walls([a, b], SCFG)]
    report = align_shared_walls([a, b, c], walls, {**SCFG, 'align_max_new_overlap_m2': 0.0})
    assert report[0]['status'] == 'skipped' and 'overlap' in report[0]['reason']


def test_parallel_wall_is_unchanged():
    a, b = _rect(0, 0, 4, 3, 'A'), _rect(4.12, 0, 7, 3, 'B')
    before = (a['polygon'].copy(), b['polygon'].copy())
    report = align_shared_walls([a, b], pair_shared_walls([a, b], SCFG), SCFG)
    assert report[0]['status'] == 'aligned' and report[0]['max_rotation_deg'] < 1e-6
    assert np.allclose(a['polygon'], before[0]) and np.allclose(b['polygon'], before[1])
