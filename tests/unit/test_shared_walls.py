from pathlib import Path

import numpy as np
import pytest
import yaml

from property_capture.rooms.shared_walls import pair_faces, pair_shared_walls

with open(Path(__file__).resolve().parents[2] / 'configs' / 'default.yaml', encoding='utf-8') as f:
    SCFG = yaml.safe_load(f)['floorplan']['shared_walls']


def _rect(x0, y0, x1, y1, rid):
    return {'room_id': rid, 'polygon': np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], float)}  # CCW


def test_two_rooms_share_a_12cm_wall():
    a, b = _rect(0, 0, 4, 3, 'A'), _rect(4.12, 0.5, 7, 3, 'B')
    walls = pair_shared_walls([a, b], SCFG)
    assert len(walls) == 1
    w = walls[0]
    assert np.isclose(w['thickness_m'], 0.12) and np.isclose(w['overlap_m'], 2.5)
    assert np.isclose(w['gap_area_m2'], 0.30)
    assert np.allclose(w['centrelines'][0], [[4.06, 0.5], [4.06, 3.0]])
    assert w['rooms'] == ['A', 'B'] and w['thickness_status'] == 'measured'


def test_outline_jog_is_excluded_from_wall_thickness():
    # B's face is 16 cm from A along 2.5 m, but jogs out to 34 cm along a short 0.5 m stretch
    a = _rect(0, 0, 4, 3, 'A')
    b = {'room_id': 'B', 'polygon': np.array([[4.16, 0], [7, 0], [7, 3], [4.34, 3], [4.34, 2.5],
                                              [4.16, 2.5]], float)}
    walls = pair_shared_walls([a, b], SCFG)
    assert len(walls) == 1
    w = walls[0]
    assert np.isclose(w['thickness_m'], 0.16)
    assert len(w['excluded_outline_jogs']) == 1 and np.isclose(w['excluded_outline_jogs'][0]['thickness_m'], 0.34)


def test_non_parallel_faces_give_no_thickness():
    a = _rect(0, 0, 4, 3, 'A')
    b = {'room_id': 'B', 'polygon': np.array([[4.05, 0], [7, 0], [7, 3], [4.15, 3]], float)}   # diverge 10 cm
    w = pair_shared_walls([a, b], SCFG)[0]
    assert w['thickness_m'] is None and 'not parallel' in w['thickness_status']
    assert np.isclose(w['candidate_thickness_m'], 0.10, atol=0.005)


def test_slightly_skewed_wall_reports_thickness_variation():
    a = _rect(0, 0, 4, 3, 'A')
    b = {'room_id': 'B', 'polygon': np.array([[4.10, 0], [7, 0], [7, 3], [4.14, 3]], float)}
    p = pair_faces(a, b, SCFG)[0]
    assert np.isclose(p['thickness_m'], 0.12, atol=1e-3)
    assert np.isclose(p['thickness_variation_m'], 0.04, atol=1e-3)


@pytest.mark.parametrize('b, why', [
    (_rect(4.6, 0, 7, 3, 'B'), 'too far apart for a wall'),
    (_rect(4.01, 0, 7, 3, 'B'), 'too thin for a wall'),
    (_rect(4.1, 2.9, 7, 6, 'B'), 'overlap too short'),
])
def test_non_walls_are_not_paired(b, why):
    assert pair_faces(_rect(0, 0, 4, 3, 'A'), b, SCFG) == [], why


def test_rotated_rooms_still_pair():
    th = np.radians(30)
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    a, b = _rect(0, 0, 4, 3, 'A'), _rect(4.15, 0, 7, 3, 'B')
    a['polygon'], b['polygon'] = a['polygon'] @ R.T, b['polygon'] @ R.T
    p = pair_faces(a, b, SCFG)
    assert len(p) == 1 and np.isclose(p[0]['thickness_m'], 0.15)
