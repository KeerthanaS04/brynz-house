from pathlib import Path

import numpy as np
import pytest
import yaml

from property_capture.rooms.segmentation import overlap_area, resolve_overlaps, segment_rooms, to_m

with open(Path(__file__).resolve().parents[2] / 'configs' / 'default.yaml', encoding='utf-8') as f:
    SCFG = yaml.safe_load(f)['floorplan']['segmentation']

GRID = {'origin': np.array([-0.5, -0.5]), 'cell_m': 0.02, 'shape': (200, 400)}


def _mask(boxes):
    rr, cc = np.mgrid[0:GRID['shape'][0], 0:GRID['shape'][1]]
    xy = to_m(np.stack([cc, rr], axis=-1), GRID)
    m = np.zeros(GRID['shape'], np.uint8)
    for x0, y0, x1, y1 in boxes:
        m[(xy[..., 0] >= x0) & (xy[..., 0] <= x1) & (xy[..., 1] >= y0) & (xy[..., 1] <= y1)] = 255
    return m


def _seg(a, b):
    a, b = np.array(a, float), np.array(b, float)
    L = np.linalg.norm(b - a)
    return {'centre': (a + b) / 2, 'direction': (b - a) / L, 's0': -L / 2, 's1': L / 2}


def _two_rooms(door):
    """4x3 room A and 3x3 room B sharing the wall x=4, with a gap (door) at y in door."""
    mask = _mask([(0, 0, 7, 3)])
    segs = [_seg((4, 0), (4, door[0])), _seg((4, door[1]), (4, 3)),
            _seg((0, 0), (7, 0)), _seg((0, 3), (7, 3)), _seg((0, 0), (0, 3)), _seg((7, 0), (7, 3))]
    return mask, segs


def test_two_rooms_split_at_door():
    mask, segs = _two_rooms((1.0, 1.9))
    cam = np.column_stack([np.linspace(0.5, 6.5, 200), np.full(200, 1.5)])
    labels, rooms, conns, stats = segment_rooms(mask, segs, GRID, cam, SCFG)
    assert stats['rooms'] == 2
    # pixel areas exclude the 6 cm wall barriers, so compare sides of the shared wall instead
    x = to_m(np.stack(np.mgrid[0:GRID['shape'][0], 0:GRID['shape'][1]][::-1], axis=-1), GRID)[..., 0]
    for r in rooms:
        sel = labels == r['label']
        left = np.mean(x[sel] < 4.0)
        assert left > 0.99 or left < 0.01
    assert np.isclose(sum(r['area_px'] for r in rooms) * 0.02 ** 2, stats['free_area_m2'])
    openings = [c for c in conns if c['type'] == 'opening']
    assert len(openings) == 1
    assert abs(openings[0]['opening_width_m'] - 0.9) < 0.15
    assert abs(openings[0]['location'][0] - 4.0) < 0.1 and 1.0 < openings[0]['location'][1] < 1.9
    assert openings[0]['camera_transitions'] >= 1
    assert all(r['visited'] for r in rooms)


def test_thick_wall_without_door_is_shared():
    mask = _mask([(0, 0, 7, 3)])
    segs = [_seg((4.0, 0), (4.0, 3)), _seg((4.15, 0), (4.15, 3)),
            _seg((0, 0), (7, 0)), _seg((0, 3), (7, 3)), _seg((0, 0), (0, 3)), _seg((7, 0), (7, 3))]
    labels, rooms, conns, stats = segment_rooms(mask, segs, GRID, np.array([[1.0, 1.5]]), SCFG)
    assert stats['rooms'] == 2
    assert [c['type'] for c in conns] == ['shared_wall']
    assert abs(conns[0]['shared_boundary_m'] - 3.0) < 0.2


def test_resolve_overlaps_subtracts_only_the_overlap():
    big = np.array([[0, 0], [4.3, 0], [4.3, 3], [0, 3]], float)
    small = np.array([[3.8, 0], [7, 0], [7, 3], [3.8, 3]], float)    # overlaps big over 0.5 m x 3 m
    other = np.array([[0, 3.5], [2, 3.5], [2, 5], [0, 5]], float)    # touches nothing
    rooms = [{'polygon': big, 'area_m2': 12.9}, {'polygon': small, 'area_m2': 9.6},
             {'polygon': other, 'area_m2': 3.0}]
    assert abs(overlap_area([big, small, other]) - 1.5) < 1e-9
    resolve_overlaps(rooms)
    assert overlap_area([r['polygon'] for r in rooms]) < 1e-9
    assert rooms[0]['clipped_area_m2'] == 0 and np.array_equal(rooms[0]['polygon'], big)
    assert abs(rooms[1]['clipped_area_m2'] - 1.5) < 1e-6
    assert len(rooms[1]['polygon']) == 4                              # still a clean rectangle
    assert rooms[2]['clipped_area_m2'] == 0 and np.array_equal(rooms[2]['polygon'], other)


def test_wide_open_plan_opening_stays_one_room():
    mask, segs = _two_rooms((0.5, 2.5))
    labels, rooms, conns, stats = segment_rooms(mask, segs, GRID, np.array([[1.0, 1.5]]), SCFG)
    assert stats['rooms'] == 1


def test_single_room_without_walls():
    mask = _mask([(0, 0, 4, 3)])
    labels, rooms, conns, stats = segment_rooms(mask, [], GRID, np.array([[2.0, 1.5]]), SCFG)
    assert stats['rooms'] == 1 and conns == []
    assert abs(rooms[0]['area_px'] * 0.02 ** 2 - 12.0) < 0.2


def test_room_without_camera_is_not_visited():
    mask, segs = _two_rooms((1.0, 1.9))
    cam = np.column_stack([np.linspace(0.5, 3.5, 100), np.full(100, 1.5)])
    labels, rooms, conns, stats = segment_rooms(mask, segs, GRID, cam, SCFG)
    by_x = {round(float(np.mean(np.nonzero(labels == r['label'])[1]))): r for r in rooms}
    left, right = by_x[min(by_x)], by_x[max(by_x)]
    assert left['visited'] and not right['visited']


def test_overlap_area_tolerates_degenerate_polygons():
    square = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], float)
    assert overlap_area([square, np.array([[0.5, 0.5], [2, 2]]), square + 0.5]) == pytest.approx(0.25)
