from pathlib import Path

import numpy as np
import yaml

from property_capture.geometry.transforms import invert, transform
from property_capture.registration.drift import (VoxelMap, _yaw_and_t, correction_matrix, detect_jumps,
                                                 icp_4dof, interpolate_corrections, select_keyframes)

with open(Path(__file__).resolve().parents[2] / 'configs' / 'default.yaml', encoding='utf-8') as f:
    CFG = yaml.safe_load(f)['drift']
UP = np.array([0.0, 1.0, 0.0])


def _room_surfaces(rng, n=6000, w=4.0, d=3.0, h=2.5):
    """Points and normals on the floor and four walls of a w x d room (y up)."""
    pts, nrm = [], []
    for _ in range(n):
        s = rng.integers(5)
        a, b = rng.uniform(0, 1, 2)
        if s == 0:
            pts.append([a * w, 0, b * d]); nrm.append([0, 1, 0])
        elif s == 1:
            pts.append([0, b * h, a * d]); nrm.append([1, 0, 0])
        elif s == 2:
            pts.append([w, b * h, a * d]); nrm.append([-1, 0, 0])
        elif s == 3:
            pts.append([a * w, b * h, 0]); nrm.append([0, 0, 1])
        else:
            pts.append([a * w, b * h, d]); nrm.append([0, 0, -1])
    return np.array(pts), np.array(nrm, float)


def test_yaw_and_translation_round_trip():
    C = correction_matrix(np.radians(7.5), np.array([0.1, -0.02, 0.3]), UP)
    yaw, t = _yaw_and_t(C, UP)
    assert np.isclose(np.degrees(yaw), 7.5) and np.allclose(t, [0.1, -0.02, 0.3])
    assert np.allclose(C[:3, :3] @ UP, UP)                 # gravity axis unchanged


def test_voxel_map_nearest_matches_brute_force():
    rng = np.random.default_rng(0)
    P = rng.uniform(0, 2, size=(3000, 3))
    vm = VoxelMap(0.05)
    vm.add(P, np.tile([0, 1.0, 0], (len(P), 1)))
    Q = rng.uniform(0.2, 1.8, size=(300, 3))
    idx, d = vm.nearest(Q)
    brute = np.linalg.norm(vm.points[None, :, :] - Q[:, None, :], axis=2).min(axis=1)
    found = idx >= 0
    # exact whenever the true nearest point is within one voxel; never closer than the truth
    within = brute < 0.05
    assert np.all(found[within]) and np.allclose(d[within], brute[within])
    assert np.all(d[found] >= brute[found] - 1e-12)


def test_voxel_map_keeps_first_observation_and_its_keyframe():
    vm = VoxelMap(0.05)
    nrm = np.tile([0, 1.0, 0], (2, 1))
    vm.add(np.array([[0.01, 0.01, 0.01], [1.01, 0.01, 0.01]]), nrm, birth=0)
    vm.add(np.array([[0.02, 0.02, 0.02], [2.01, 0.01, 0.01]]), nrm, birth=5)   # first point: same voxel
    assert len(vm) == 3
    idx, _ = vm.nearest(np.array([[0.02, 0.02, 0.02], [2.0, 0.0, 0.0]]))
    assert vm.birth[idx].tolist() == [0, 5]
    assert np.allclose(vm.points[idx[0]], [0.01, 0.01, 0.01])


def test_icp_recovers_known_4dof_offset():
    rng = np.random.default_rng(1)
    P, n = _room_surfaces(rng)
    vm = VoxelMap(0.05)
    vm.add(P, n)
    true_C = correction_matrix(np.radians(2.0), np.array([0.04, 0.01, -0.03]), UP)
    drifted = transform(invert(true_C), P[::2])            # what the drifted odometry would show
    C, info = icp_4dof(drifted, vm, UP, np.eye(4), CFG)
    yaw, t = _yaw_and_t(C, UP)
    assert abs(np.degrees(yaw) - 2.0) < 0.2
    assert np.allclose(t, true_C[:3, 3], atol=0.01)
    assert info['eigen_min'] > CFG['icp_min_eigen']


def test_single_wall_is_flagged_degenerate():
    rng = np.random.default_rng(2)
    P = np.column_stack([rng.uniform(0, 4, 3000), rng.uniform(0, 2.5, 3000), np.zeros(3000)])
    n = np.tile([0, 0, 1.0], (3000, 1))
    vm = VoxelMap(0.05)
    vm.add(P, n)
    _, info = icp_4dof(P[::2] + [0.05, 0, 0.02], vm, UP, np.eye(4), CFG)
    assert info['eigen_min'] < CFG['icp_min_eigen']        # sliding along the wall is unobservable


def test_detect_jumps():
    pos = np.cumsum(np.full((20, 3), 0.01), axis=0)
    pos[12:] += [0.3, 0, 0]
    assert detect_jumps(pos, 0.1) == [12]


def test_keyframes_include_both_sides_of_a_jump():
    T = np.tile(np.eye(4), (50, 1, 1))
    T[:, 0, 3] = np.linspace(0, 0.5, 50)                   # 1 cm per frame, below keyframe spacing
    keys = select_keyframes(T, list(range(0, 50, 3)), CFG, forced=[31])
    assert 30 in keys and 31 in keys and keys[0] == 0


def test_interpolated_corrections_between_keyframes():
    C0 = correction_matrix(0.0, np.zeros(3), UP)
    C1 = correction_matrix(np.radians(4.0), np.array([0.2, 0, 0]), UP)
    Cs = interpolate_corrections(11, [0, 10], [C0, C1], UP)
    yaw, t = _yaw_and_t(Cs[5], UP)
    assert np.isclose(np.degrees(yaw), 2.0) and np.allclose(t, [0.1, 0, 0])
    assert np.allclose(Cs[0], C0) and np.allclose(Cs[10], C1)
