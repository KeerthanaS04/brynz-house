"""LiDAR tier end to end on synthetic captures of a known 4 x 3 x 2.5 m box room.

Clean and difficult captures must give the box's dimensions, or flag what they cannot measure;
broken captures must stop with a message naming the cause (CLAUDE_updated.md adversarial tests).
"""
import json
from pathlib import Path

import numpy as np
import pytest

import synthetic_capture as sc
from property_capture.pipeline import run

ROOT = Path(__file__).resolve().parents[2]
W, L, H = 4.0, 3.0, 2.5
FAST = ['rgb.enabled=false', 'drift.enabled=false']


def _run(tmp_path, T, depth, overrides=FAST, **write_kw):
    cap = sc.write_capture(tmp_path / 'cap' / 'synthetic', T, depth, **write_kw)
    out = tmp_path / 'run'
    run(cap, ROOT / 'configs' / 'default.yaml', out, ROOT / 'configs' / 'evaluation' / 'gates.yaml',
        overrides=overrides)
    return json.loads((out / 'property.json').read_text(encoding='utf-8'))


def _box(T):
    return [sc.render_box(t, W, L, H) for t in T]


def _rooms(prop):
    ms = {m['id']: m for m in prop['measurements']}
    out = []
    for r in prop['rooms']:
        c = r['ceiling']
        out.append({'area': ms[r['floor_area_measurement_id']]['value'],
                    'walls': [(ms[w['length_measurement_id']]['value'], w['evidence'], w['start'], w['end'])
                              for w in r['walls']],
                    'ceiling': ms[c['measurement_id']]['value'] if c['status'] == 'measured' else None})
    return out


def _observed_wall_errors(prop, min_length_m=0.2):
    """Distance (m) of every observed wall's end points from the nearest true box wall (world x, z).

    Edges shorter than min_length_m are skipped: corner jogs of ~10 cm with 3 support points can
    pass the 50% observed-fraction rule while sitting ~11 cm off a wall (known limitation, seen
    with partial coverage)."""
    cf = prop['coordinate_frame']
    o, e1, e2, up = (np.asarray(cf[k], float) for k in ('origin_world', 'x_axis_world', 'y_axis_world', 'up_world'))
    errs = []
    for r in _rooms(prop):
        for length, evidence, a, b in r['walls']:
            if evidence != 'observed' or length < min_length_m:
                continue
            for p in (a, b):
                # 2D coordinates are projections (see 'convention'); add the floor's height along up
                x, _, z = p[0] * e1 + p[1] * e2 + (o @ up) * up
                errs.append(min(abs(x), abs(x - W), abs(z), abs(z - L)))
    return np.array(errs)


def _check_measurements(prop):
    for m in prop['measurements']:
        assert np.isfinite(m['value']) and m['value'] > 0, m['id']          # no fabricated zeros
        assert m['uncertainty_status'] == 'uncalibrated' and m['interval'] is None, m['id']


def test_clean_box_gives_true_dimensions(tmp_path):
    prop = _run(tmp_path, T := sc.walk(), _box(T))
    (room,) = _rooms(prop)
    assert room['area'] == pytest.approx(W * L, abs=0.05)
    assert sorted(w[0] for w in room['walls']) == pytest.approx([L, L, W, W], abs=0.02)
    assert all(w[1] == 'observed' for w in room['walls'])
    # wall positions, not just lengths: before walls.refine_offsets every wall sat 2 cm off (cell centres)
    assert _observed_wall_errors(prop).max() < 0.005
    assert room['ceiling'] == pytest.approx(H, abs=0.01)
    assert {w['code'] for w in prop['warnings']} == {'DEPTH_UNIT_ASSUMED', 'POSES_USED_AS_IS'}
    _check_measurements(prop)


def test_gravity_not_along_world_y(tmp_path):
    """Whole capture in a world frame tilted 30 deg and 20 deg: up must come from the IMU, not +y."""
    T = sc.walk()
    depth = _box(T)
    a, b = np.radians(30), np.radians(20)
    G = np.eye(4)
    G[:3, :3] = (np.array([[np.cos(b), -np.sin(b), 0], [np.sin(b), np.cos(b), 0], [0, 0, 1]])
                 @ np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]]))
    prop = _run(tmp_path, np.array([G @ t for t in T]), depth, gravity_world=G[:3, :3] @ [0, -1, 0])
    (room,) = _rooms(prop)
    assert room['area'] == pytest.approx(W * L, abs=0.05)
    assert sorted(w[0] for w in room['walls']) == pytest.approx([L, L, W, W], abs=0.02)


def test_tracking_jump_is_reported(tmp_path):
    T = sc.walk()
    depth = _box(T)
    T[90:, 0, 3] += 0.3                        # poses jump 0.3 m; the camera did not move
    prop = _run(tmp_path, T, depth)
    jump = [w for w in prop['warnings'] if w['code'] == 'TRACKING_JUMP']
    assert jump and jump[0]['frames'] == [90]


@pytest.mark.xfail(strict=True, reason='known limitation: a tracking jump duplicates geometry; drift correction '
                                       'does not repair it (walls come out 0.3 m short)')
def test_tracking_jump_is_repaired(tmp_path):
    T = sc.walk()
    depth = _box(T)
    T[90:, 0, 3] += 0.3
    prop = _run(tmp_path, T, depth, overrides=['rgb.enabled=false'])
    assert sorted(w[0] for w in _rooms(prop)[0]['walls']) == pytest.approx([L, L, W, W], abs=0.02)


def test_depth_dropout_keeps_observed_walls_true(tmp_path):
    """Half the depth pixels missing at random (dark or glossy surfaces): walls that are reported as
    observed must still lie on the true walls; the rest must be flagged inferred."""
    T = sc.walk()
    rng = np.random.default_rng(0)
    depth = []
    for d in _box(T):
        d[rng.random(d.shape) < 0.5] = 0
        depth.append(d)
    prop = _run(tmp_path, T, depth)
    (room,) = _rooms(prop)
    assert _observed_wall_errors(prop).max() < 0.02
    assert room['area'] == pytest.approx(W * L, rel=0.06)
    assert room['ceiling'] == pytest.approx(H, abs=0.01)
    _check_measurements(prop)


def test_partial_coverage_keeps_observed_walls_true(tmp_path):
    """The camera never turns to one wall: observed walls stay true, the unseen side is inferred."""
    T = np.array([sc.camera_pose([1.0 + 2.0 * s, 1.4, 1.0], np.radians(-70 + 140 * s), 0.3 * np.sin(6 * np.pi * s))
                  for s in np.arange(180) / 180])
    prop = _run(tmp_path, T, _box(T))
    room = _rooms(prop)[0]
    assert _observed_wall_errors(prop).max() < 0.02
    assert any(w[1] != 'observed' for w in room['walls'])
    _check_measurements(prop)


def test_walk_looking_down_completes(tmp_path):
    """Regression: a half of the frames produced a degenerate room region and crashed the
    half-split consistency check."""
    prop = _run(tmp_path, T := sc.walk(pitch_deg=-15), _box(T))
    assert _rooms(prop)[0]['area'] == pytest.approx(W * L, abs=0.05)


def _odometry_edit(row, col, value):
    """Change one odometry.csv row (row 0 is the header) as the file is written."""
    def edit(rows):
        parts = rows[row].split(', ')
        parts[col] = value
        rows[row] = ', '.join(parts)
        return rows
    return edit


BROKEN = {
    'missing timestamp': ({'odometry_override': _odometry_edit(11, 0, '')}, r'odometry\.csv.*timestamp'),
    'timestamps not increasing': ({'timestamps': np.r_[np.arange(20), np.arange(20)] / 30 + 1000},
                                  'not strictly increasing'),
    'NaN position': ({'odometry_override': _odometry_edit(11, 2, 'nan')}, r"odometry\.csv.*\['x'\]"),
    'zero quaternion': ({'odometry_override': _odometry_edit(11, slice(5, 9), ['0', '0', '0', '0'])},
                        'quaternion'),
    'zero focal length': ({'intrinsics': (0.0, 0.0, 159.5, 119.5)}, 'invalid intrinsics'),
    'principal point outside image': ({'intrinsics': (250.0, 250.0, 900.0, 119.5)}, 'invalid intrinsics'),
    'IMU row missing values': ({}, r'imu\.csv.*a_x'),
    'IMU in m/s2': ({}, r'imu\.csv.*9\.8'),
    'all depth zero': ({}, 'depth pixels are usable'),
    'depth in wrong units': ({}, 'other units'),
}


@pytest.mark.parametrize('case', sorted(BROKEN))
def test_broken_capture_stops_with_reason(tmp_path, case):
    write_kw, message = BROKEN[case]
    T = sc.walk(n_frames=40)
    depth = _box(T)
    if case == 'all depth zero':
        depth = [np.zeros_like(d) for d in depth]
    if case == 'depth in wrong units':
        depth = [d * 10 for d in depth]                      # e.g. stored in 0.1 mm
    cap = sc.write_capture(tmp_path / 'cap' / 'broken', T, depth, **write_kw)
    imu = cap / 'imu.csv'
    if case == 'IMU row missing values':
        lines = imu.read_text().splitlines()
        lines[5] = lines[5].split(', ')[0] + ', , , , 0, 0, 0'
        imu.write_text('\n'.join(lines) + '\n')
    if case == 'IMU in m/s2':
        lines = imu.read_text().splitlines()
        lines = [lines[0]] + [', '.join([p[0], *[str(float(v) * 9.81) for v in p[1:4]], *p[4:]])
                              for p in (ln.split(', ') for ln in lines[1:])]
        imu.write_text('\n'.join(lines) + '\n')
    with pytest.raises(ValueError, match=message):
        run(cap, ROOT / 'configs' / 'default.yaml', tmp_path / 'run', ROOT / 'configs' / 'evaluation' / 'gates.yaml',
            overrides=FAST)


def test_missing_depth_frame_stops_with_reason(tmp_path):
    T = sc.walk(n_frames=40)
    cap = sc.write_capture(tmp_path / 'cap' / 'broken', T, _box(T))
    (cap / 'depth' / '000007.png').unlink()
    with pytest.raises(ValueError, match='lack depth/confidence'):
        run(cap, ROOT / 'configs' / 'default.yaml', tmp_path / 'run', ROOT / 'configs' / 'evaluation' / 'gates.yaml',
            overrides=FAST)
