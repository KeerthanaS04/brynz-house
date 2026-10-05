from pathlib import Path

import numpy as np
import yaml

from property_capture.rooms.ceiling import assess_ceiling, level_stats

with open(Path(__file__).resolve().parents[2] / 'configs' / 'default.yaml', encoding='utf-8') as f:
    CCFG = yaml.safe_load(f)['floorplan']['ceiling']


def _surface(rng, x0, x1, y0, y1, level, n=4000, noise=0.003):
    xy = rng.uniform([x0, y0], [x1, y1], size=(n, 2))
    return xy, level + rng.normal(scale=noise, size=n)


def _stats(xy, h):
    return level_stats(xy, h, CCFG['cell_m'], CCFG['min_sigma_m'])


def test_flat_ceiling_is_measured():
    rng = np.random.default_rng(0)
    floor = _stats(*_surface(rng, 0, 4, 0, 3, 0.0))
    ceil = _stats(*_surface(rng, 1, 3, 1, 2, 2.70))           # seen over 2 m2 of a 12 m2 room
    out = assess_ceiling(ceil, floor, floor, CCFG, coverage=0.17)
    assert out['status'] == 'measured', out['reason']
    assert abs(out['height_m'] - 2.70) < 0.003
    assert out['standard_error_m'] < CCFG['max_standard_error_m']


def test_too_few_cells_is_not_measurable():
    rng = np.random.default_rng(1)
    floor = _stats(*_surface(rng, 0, 4, 0, 3, 0.0))
    ceil = _stats(*_surface(rng, 1, 1.4, 1, 1.4, 2.70, n=300))  # four cells
    out = assess_ceiling(ceil, floor, floor, CCFG)
    assert out['status'] == 'not_measurable' and 'cells' in out['reason']


def test_stepped_ceiling_is_not_flat():
    rng = np.random.default_rng(2)
    floor = _stats(*_surface(rng, 0, 4, 0, 3, 0.0))
    a, ha = _surface(rng, 0, 2, 0, 3, 2.50)
    b, hb = _surface(rng, 2, 4, 0, 3, 2.80)
    out = assess_ceiling(_stats(np.vstack([a, b]), np.r_[ha, hb]), floor, floor, CCFG)
    assert out['status'] == 'not_measurable' and 'not flat' in out['reason']


def test_furniture_top_is_implausible():
    rng = np.random.default_rng(3)
    floor = _stats(*_surface(rng, 0, 4, 0, 3, 0.0))
    top = _stats(*_surface(rng, 0, 2, 0, 1, 1.65))
    out = assess_ceiling(top, floor, floor, CCFG)
    assert out['status'] == 'not_measurable' and 'implausible' in out['reason']
    assert abs(out['candidate_height_m'] - 1.65) < 0.01


def test_uneven_floor_makes_height_imprecise():
    # flat ceiling, but the room's floor has two levels 4 cm apart (e.g. a raised platform):
    # the floor level is uncertain, so the height is not reported
    rng = np.random.default_rng(4)
    a, ha = _surface(rng, 0, 0.75, 0, 1.0, 0.00, n=800)
    b, hb = _surface(rng, 0.75, 1.5, 0, 1.0, 0.04, n=800)
    floor_room = _stats(np.vstack([a, b]), np.r_[ha, hb])
    ceil = _stats(*_surface(rng, 0, 1.5, 0, 1.0, 2.70))
    out = assess_ceiling(ceil, floor_room, floor_room, CCFG)
    assert out['status'] == 'not_measurable' and 'standard error' in out['reason']


def test_floor_falls_back_to_all_points_when_room_floor_unseen():
    rng = np.random.default_rng(5)
    floor_all = _stats(*_surface(rng, 0, 4, 0, 3, 0.01))
    floor_room = _stats(*_surface(rng, 0, 0.3, 0, 0.3, 0.05, n=50))   # one or two cells only
    ceil = _stats(*_surface(rng, 0, 4, 0, 3, 2.71))
    out = assess_ceiling(ceil, floor_room, floor_all, CCFG)
    assert out['floor_source'] == 'all floor points'
    assert abs(out['height_m'] - 2.70) < 0.003


def test_level_uses_cells_not_points():
    # a dense patch must not outweigh the rest of the ceiling
    rng = np.random.default_rng(6)
    xy, h = _surface(rng, 0, 4, 0, 3, 2.70, n=2000)
    dense, hd = _surface(rng, 0, 0.25, 0, 0.25, 2.60, n=20000)
    s = _stats(np.vstack([xy, dense]), np.r_[h, hd])
    assert abs(s['level_m'] - 2.70) < 0.002
