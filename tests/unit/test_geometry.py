import numpy as np

from property_capture.calibration.gravity import axis_aligned_rotations, nearest_index
from property_capture.geometry.planes import ransac_plane
from property_capture.rooms.floorplan import extract_room, polygon_area

FLOORPLAN_CFG = {'grid_m': 0.02, 'wall_min_points': 2, 'close_m': 0.10, 'simplify_m': 0.05,
                 'wall_support_m': 0.05, 'observed_bin_m': 0.05, 'min_observed_fraction': 0.5}


def test_ransac_recovers_tilted_plane():
    rng = np.random.default_rng(0)
    xy = rng.uniform(-2, 2, size=(4000, 2))
    n_true = np.array([0.05, 0.02, 1.0]) / np.linalg.norm([0.05, 0.02, 1.0])
    z = -(n_true[0] * xy[:, 0] + n_true[1] * xy[:, 1]) / n_true[2]
    P = np.column_stack([xy, z]) + rng.normal(scale=0.003, size=(4000, 3))
    P = np.vstack([P, rng.uniform(-2, 2, size=(1000, 3))])
    pl = ransac_plane(P, 0.01, 300, rng, normal_hint=np.array([0, 0, 1.0]), max_angle_deg=10)
    assert np.degrees(np.arccos(pl['normal'] @ n_true)) < 0.5
    assert pl['rms_m'] < 0.005


def test_polygon_area_sign_and_value():
    sq = np.array([[0, 0], [4, 0], [4, 3], [0, 3]], float)
    assert np.isclose(polygon_area(sq), 12)
    assert np.isclose(polygon_area(sq[::-1]), -12)


def _rect_room(w, d, rng, gap=None):
    """Wall points on the boundary of a w x d room; optional (x0, x1) gap in the y=0 wall."""
    pts = []
    for t in rng.uniform(0, 1, 6000):
        pts += [[t * w, 0], [t * w, d], [0, t * d], [w, t * d]]
    pts = np.array(pts) + rng.normal(scale=0.005, size=(len(pts), 2))
    if gap:
        pts = pts[~((np.abs(pts[:, 1]) < 0.05) & (pts[:, 0] > gap[0]) & (pts[:, 0] < gap[1]))]
    floor = rng.uniform([0.1, 0.1], [w - 0.1, d - 0.1], size=(20000, 2))
    return pts, floor


def test_rectangular_room_dimensions():
    rng = np.random.default_rng(2)
    wall, floor = _rect_room(4.0, 3.0, rng)
    room = extract_room(wall, floor, FLOORPLAN_CFG)
    assert abs(room['area_m2'] - 12.0) / 12.0 < 0.03
    lengths = sorted(w['length_m'] for w in room['walls'])
    assert len(lengths) == 4
    assert np.allclose(lengths, [3, 3, 4, 4], atol=0.06)
    assert all(w['evidence'] == 'observed' for w in room['walls'])


def test_axis_aligned_rotations_are_24_proper_rotations():
    rots = axis_aligned_rotations()
    assert len(rots) == 24
    assert all(np.isclose(np.linalg.det(R), 1) for R in rots)
    assert len({tuple(R.ravel()) for R in rots}) == 24


def test_nearest_index():
    t = np.array([0.0, 1.0, 2.0, 3.0])
    assert nearest_index(t, np.array([-1, 0.4, 0.6, 2.5001, 9])).tolist() == [0, 0, 1, 3, 3]
