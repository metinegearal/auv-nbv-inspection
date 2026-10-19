"""Regression checks for replanning continuity without requiring a ROS runtime."""

import numpy as np

from auv_planner_reference.route_planning import (
    generate_smooth_trajectory, path_is_collision_free, plan_multi_waypoint_route,
)
from auv_planner_reference.trajectory_optimization import (
    continuity_cost, estimate_velocity, optimize_trajectory,
)


def trajectory(positions, yaw=0.0):
    positions = np.asarray(positions, dtype=float)
    return np.column_stack((positions, np.broadcast_to(yaw, len(positions))))


def test_velocity_uses_world_pose_history_and_irregular_timestamps():
    velocity = np.array([0.4, -0.2, 0.1])
    history = [(t, np.array([10, 20, -30]) + velocity * t)
               for t in [0, 1, 1.2, 1.6, 2]]
    np.testing.assert_allclose(estimate_velocity(history), velocity)
    np.testing.assert_array_equal(estimate_velocity([(1, np.zeros(3))]), np.zeros(3))
    np.testing.assert_array_equal(estimate_velocity([(1, np.zeros(3))] * 2), np.zeros(3))


def test_candidate_order_prefers_forward_motion_but_allows_required_return():
    forward = (np.array([2.1, 0.1, 0.1]), np.array([3, 0, 0]))
    behind = (np.array([-0.9, 0.1, 0.1]), np.array([-2, 0, 0]))
    start = np.array([0.1, 0.1, 0.1])
    route, targets = plan_multi_waypoint_route(
        start, [behind, forward], set(), 1.0, current_velocity=np.array([0.4, 0, 0]))
    np.testing.assert_allclose(route[0], start)
    np.testing.assert_allclose(targets[0], forward[1])
    route, targets = plan_multi_waypoint_route(
        start, [behind], set(), 1.0, current_velocity=np.array([0.4, 0, 0]))
    np.testing.assert_allclose(route[-1], behind[0])


def test_reversal_and_full_yaw_turn_have_higher_cost():
    forward = trajectory([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]])
    reverse = forward.copy()
    reverse[:, 0] *= -1
    velocity = np.array([0.4, 0, 0])
    assert continuity_cost(reverse, velocity) > continuity_cost(forward, velocity)
    spin = forward.copy()
    spin[:, 3] = np.linspace(0, 2 * np.pi, 4)
    assert continuity_cost(spin, velocity) > continuity_cost(forward, velocity) + 20


def test_recent_retracing_is_penalized_without_penalizing_departure():
    path = trajectory([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]])
    baseline = continuity_cost(path, np.zeros(3))
    assert continuity_cost(path, np.zeros(3), [[0, 0, 0]]) == baseline
    assert continuity_cost(path, np.zeros(3), [[2, 0, 0]]) > baseline


def test_gradual_spatial_loop_is_penalized_even_with_constant_yaw():
    angles = np.linspace(0, 2 * np.pi, 25)
    loop = trajectory(np.column_stack((np.cos(angles), np.sin(angles),
                                       np.zeros(len(angles)))))
    assert continuity_cost(loop, np.zeros(3)) > 20


def test_spline_positions_are_independent_of_target_heading():
    path = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]], dtype=float)
    constant = path + [0, 1, 0]
    turning = path + np.array([[1, 0, 0], [0, 1, 0], [-1, 0, 0], [0, -1, 0]])
    a = generate_smooth_trajectory(path, constant, num_points=30)
    b = generate_smooth_trajectory(path, turning, num_points=30)
    np.testing.assert_allclose(a[:, :3], b[:, :3])


def test_optimizer_improves_entry_and_preserves_boundary_and_step_lengths():
    path = trajectory([[0, i * 0.5, 0] for i in range(8)])
    velocity = np.array([0.4, 0, 0])
    optimized = optimize_trajectory(path, path[0, :3], 0, velocity, set(), 0.1,
                                    max_iterations=20)
    assert continuity_cost(optimized, velocity) < continuity_cost(path, velocity)
    assert optimized[1, 0] > 0.1
    assert np.linalg.norm(optimized[1, :3] - optimized[0, :3]) > 0.2
    np.testing.assert_allclose(optimized[[0, -1], :3], path[[0, -1], :3])
    assert optimized[0, 3] == 0


def test_optimizer_keeps_collision_safe_route_in_narrow_corridor():
    path = trajectory([[0.05, 0.05 + i * 0.2, 0.05] for i in range(8)])
    occupied = {(x, y, z) for x in range(-5, 6) for y in range(-5, 25)
                for z in range(-5, 6) if x != 0 or z != 0}
    optimized = optimize_trajectory(path, path[0, :3], 0, np.array([0.4, 0, 0]),
                                    occupied, 0.1, max_iterations=10)
    assert len(optimized) == len(path)
    assert path_is_collision_free(optimized[:, :3], occupied, 0.1)


def test_collision_check_rejects_single_occupied_point_and_nan():
    assert not path_is_collision_free(np.array([[0.05, 0.05, 0.05]]), {(0, 0, 0)}, 0.1)
    assert not path_is_collision_free(np.array([[np.nan, 0, 0]]), set(), 0.1)


def test_yaw_wrap_is_shortest_rotation_and_stationary_velocity_is_safe():
    path = trajectory([[0, i * 0.5, 0] for i in range(4)],
                      np.deg2rad([-179, -178, -177, -176]))
    optimized = optimize_trajectory(path, path[0, :3], np.deg2rad(179),
                                    np.zeros(3), set(), 0.1, max_iterations=10)
    assert np.all(np.isfinite(optimized))
    assert np.max(np.abs(np.diff(optimized[:, 3]))) < np.deg2rad(10)
    assert optimized[0, 3] == np.deg2rad(179)


def test_delayed_plan_starts_at_latest_pose_and_drops_passed_prefix():
    from auv_planner_reference.route_planning import reconnect_trajectory
    path = trajectory([[i * 0.5, 0, 0] for i in range(9)])
    current = np.array([1.7, 0.1, 0])
    joined = reconnect_trajectory(path, current, 0.2, set(), 0.1)
    np.testing.assert_allclose(joined[0], [1.7, 0.1, 0, 0.2])
    np.testing.assert_allclose(joined[1:, :3], path[4:, :3])
    assert np.all(np.diff(joined[:, 0]) > 0)
    np.testing.assert_allclose(path[0], [0, 0, 0, 0])


def test_delayed_entry_rejects_collision_on_new_connector():
    from auv_planner_reference.route_planning import reconnect_trajectory
    path = trajectory([[0.05, 0.05, 0.05], [1.05, 0.05, 0.05],
                       [2.05, 0.05, 0.05]])
    occupied = {(7, 3, 0)}
    assert path_is_collision_free(path[:, :3], occupied, 0.1)
    joined = reconnect_trajectory(path, [0.55, 0.55, 0.05], 0, occupied, 0.1)
    assert joined.shape == (0, 4)


def test_delayed_entry_preserves_required_turn_and_short_yaw_wrap():
    from auv_planner_reference.route_planning import reconnect_trajectory
    path = trajectory([[0, 0, 0], [2, 0, 0], [2, 2, 0], [1, 2, 0]],
                      np.deg2rad(-179))
    joined = reconnect_trajectory(path, [1.5, 0, 0], np.deg2rad(179), set(), 0.1)
    np.testing.assert_allclose(joined[1:, :3], path[1:, :3])
    assert np.max(np.abs(np.diff(joined[:, 3]))) < np.deg2rad(3)


def test_delayed_entry_handles_stationary_single_and_duplicate_samples():
    from auv_planner_reference.route_planning import reconnect_trajectory
    for path in (trajectory([[0, 0, 0]]), trajectory([[0, 0, 0]] * 3)):
        joined = reconnect_trajectory(path, [0, 0, 0], 0.4, set(), 0.1)
        np.testing.assert_allclose(joined, [[0, 0, 0, 0.4]])
    assert reconnect_trajectory(np.empty((0, 4)), [0, 0, 0], 0, set(), 0.1).shape == (0, 4)
