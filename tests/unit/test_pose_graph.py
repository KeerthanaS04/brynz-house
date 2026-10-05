import numpy as np

from property_capture.registration.drift import rotation_about
from property_capture.registration.pose_graph import edge_residual, solve_pose_graph

UP = np.array([0.0, 1.0, 0.0])


def test_edge_jacobians_match_finite_differences():
    rng = np.random.default_rng(0)
    xi, xj = rng.normal(size=4), rng.normal(size=4)
    z = rng.normal(size=4)
    _, Ji, Jj = edge_residual(xi, xj, z, UP)
    eps = 1e-6
    for k in range(4):
        d = np.zeros(4)
        d[k] = eps
        num_i = (edge_residual(xi + d, xj, z, UP)[0] - edge_residual(xi - d, xj, z, UP)[0]) / (2 * eps)
        num_j = (edge_residual(xi, xj + d, z, UP)[0] - edge_residual(xi, xj - d, z, UP)[0]) / (2 * eps)
        assert np.allclose(Ji[:, k], num_i, atol=1e-6) and np.allclose(Jj[:, k], num_j, atol=1e-6)


def _square_walk(n_side=10, side=4.0):
    """Positions walking a closed square (y up); returns true positions and per-step headings."""
    pts = []
    for s in range(4):
        start = rotation_about(UP, -np.pi / 2 * s) @ np.array([side, 0, 0])
        for k in range(n_side):
            pts.append(np.array([[0, 0, 0], [side, 0, 0], [side, 0, side], [0, 0, side]][s], float)
                       + (k / n_side) * start)
    pts.append(np.zeros(3))
    return np.array(pts)


def _odometry_edges(P, sigma_t=0.01, sigma_r=np.radians(0.2)):
    return [{'i': k, 'j': k + 1, 'kind': 'odometry', 'z': np.r_[0.0, P[k + 1] - P[k]],
             'info': np.r_[1 / sigma_r ** 2, np.full(3, 1 / sigma_t ** 2)]} for k in range(len(P) - 1)]


def test_consistent_odometry_is_left_unchanged():
    P = _square_walk()
    x0 = np.column_stack([np.zeros(len(P)), P])
    x, _ = solve_pose_graph(x0, _odometry_edges(P), UP)
    assert np.allclose(x, x0, atol=1e-9)


def test_loop_edge_pulls_drifted_trajectory_back():
    truth = _square_walk()
    # odometry with heading drift: each step's displacement rotated by an accumulating yaw error
    steps = np.diff(truth, axis=0)
    yaw_err = np.cumsum(np.full(len(steps), np.radians(0.3)))
    raw = np.vstack([truth[:1], truth[0] + np.cumsum(
        [rotation_about(UP, a) @ s for a, s in zip(yaw_err, steps)], axis=0)])
    end_err_raw = np.linalg.norm(raw[-1] - truth[-1])
    assert end_err_raw > 0.5
    edges = _odometry_edges(raw)
    # loop: the last node is truly at the first node's position, with the true heading offset
    edges.append({'i': 0, 'j': len(raw) - 1, 'kind': 'loop',
                  'z': np.r_[-yaw_err[-1], truth[-1] - truth[0]],
                  'info': np.r_[1 / np.radians(0.5) ** 2, np.full(3, 1 / 0.02 ** 2)], 'huber': 3.0})
    x0 = np.column_stack([np.zeros(len(raw)), raw])
    x, chis = solve_pose_graph(x0, edges, UP)
    assert np.linalg.norm(x[-1, 1:] - truth[-1]) < 0.1 * end_err_raw
    assert np.allclose(x[0], x0[0], atol=1e-6)                 # first node stays anchored
