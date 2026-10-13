"""Exercise mission recovery against the current route-planning interface."""

import time
from unittest.mock import Mock

import numpy as np
import pytest

from python_planner import route_planning
from python_planner.use_cases import mission as mission_module
from python_planner.use_cases.mission import InspectionMission


@pytest.fixture
def mission():
    node = InspectionMission.__new__(InspectionMission)
    node.clock = time
    node.diagnostics = Mock()
    node.telemetry = Mock()
    node.output = Mock()
    node.min_utility = .1
    node.route_max_candidates = 5
    node.camera_offset = np.zeros(3)
    node.velocity_history_window = 1.
    node.trajectory_history_window = 5.
    node.astar_max_steps = 50000
    node.optimize_motion = False
    node.motion_cost_parameters = {'reversal_weight': 4.}
    node.resolution = 1.
    node.collision_check_spacing_factor = .5
    node.trajectory_points = 300
    node.spline_smoothing = 2.
    node.optimizer_max_deviation = .5
    node.optimizer_max_iterations = 40
    node.waypoint_spacing = .5
    node.iteration = 1
    node._motion_snapshot = Mock(return_value=(np.array([.5, .5, .5]), .7, []))
    node._log_nbv_filter = Mock()
    node._publish_trajectory = Mock(return_value=True)
    node.force_replan = False
    node.active_trajectory = np.empty((0, 4))
    node.selected_targets = []
    return node


def candidate(position):
    return {'cluster_id': 0, 'position': np.array(position),
            'target': np.array([4., 2., .5]), 'utility': 1.}


@pytest.mark.parametrize('goal', [[.5, .5, .5], [1.5, .5, .5], [2.5, .5, .5]])
def test_stationary_and_short_routes_publish_without_cubic_spline(mission, goal):
    assert mission._plan_and_publish([candidate(goal)], set())
    trajectory = mission._publish_trajectory.call_args.args[0]
    np.testing.assert_allclose(trajectory[-1, :3], goal)
    assert route_planning.path_is_collision_free(trajectory[:, :3], set(), 1.)
    assert mission._failed_route_target is None
    events = mission._log_nbv_filter.call_args_list
    assert any(call.args[0] == 'trajectory_option' and
               call.args[3]['option'] == 'linear_fallback' and
               call.args[3]['collision_free'] for call in events)


def test_occupied_start_retries_without_blacklisting_or_completion_strike(mission):
    assert not mission._plan_and_publish([candidate([2.5, .5, .5])], {(0, 0, 0)})
    assert mission._plan_failure_reason == 'start_occupied'
    assert mission._failed_route_target is None
    assert not mission._blacklist_failed_target()
    assert mission.force_replan
    mission._publish_trajectory.assert_not_called()


def test_occupied_goal_preserves_failure_reason_and_target(mission):
    target = candidate([2.5, .5, .5])
    assert not mission._plan_and_publish([target], {(2, 0, 0)})
    assert mission._plan_failure_reason == 'goal_occupied'
    np.testing.assert_array_equal(mission._failed_route_target, target['target'])


@pytest.mark.parametrize('optimize', [False, True])
def test_polyline_fallback_preserves_detour_corners_and_scalar_entry_yaw(
        mission, monkeypatch, optimize):
    mission.optimize_motion = optimize
    mission.waypoint_spacing = 2.
    mission.trajectory_points = 2
    mission.optimizer = Mock(side_effect=lambda trajectory, *args, **kwargs: trajectory.copy())
    monkeypatch.setattr(mission_module, 'generate_smooth_trajectory',
                        lambda *args, **kwargs: np.empty((0, 4)))
    occupied = {(1, 0, 0)}
    assert mission._plan_and_publish([candidate([2.5, .5, .5])], occupied)
    trajectory = mission._publish_trajectory.call_args.args[0]
    assert len(trajectory) > mission.trajectory_points
    assert route_planning.path_is_collision_free(trajectory[:, :3], occupied, 1.)
    np.testing.assert_allclose(trajectory[-1, :3], [2.5, .5, .5])
    if optimize:
        assert trajectory[0, 3] == pytest.approx(.7)
        assert mission.optimizer.call_count == 1


def test_fresh_octomap_occupancy_replaces_stale_endpoint_cache(mission):
    mission.use_incremental_map_cache = True
    mission.occupied_voxels = {(100, 100, 100)}
    mission.octree = Mock()
    mission.octree.extractPointCloud.side_effect = [
        (np.array([[.5, .5, .5]]),), (np.array([[2.5, .5, .5]]),),
        (np.empty((0, 3)),)]
    np.testing.assert_array_equal(mission._occupied_points(), [[.5, .5, .5]])
    np.testing.assert_array_equal(mission._occupied_points(), [[2.5, .5, .5]])
    assert mission._occupied_points().shape == (0, 3)


def test_each_timer_cycle_rebuilds_inflation_from_current_octomap(mission):
    mission.finished_mapping = False
    mission.odom_received = mission.rgb_received = True
    mission.depth_frame = np.ones((2, 2))
    mission.depth_sequence = 2
    mission.processed_depth_sequence = 1
    mission._accumulate_depth = Mock()
    mission.octree = Mock()
    mission.octree.extractPointCloud.side_effect = [
        (np.array([[.5, .5, .5]]),), (np.array([[10.5, .5, .5]]),)]
    mission.collision_radius = 1.
    mission.collision_tracking_margin = 0.
    mission.use_incremental_map_cache = True
    mission.collision_voxels = {(99, 99, 99)}
    mission.pending_occupied_voxels = {(99, 99, 99)}
    mission._active_trajectory_is_safe = Mock(return_value=True)
    mission.replan_on_target_arrival = False
    mission.frame_period = 100
    mission.timer_callback()
    mission.depth_sequence += 1
    mission.timer_callback()
    first = mission._active_trajectory_is_safe.call_args_list[0].args[0]
    second = mission._active_trajectory_is_safe.call_args_list[1].args[0]
    assert (0, 0, 0) in first and (0, 0, 0) not in second
    assert (10, 0, 0) in second and (10, 0, 0) not in first
    assert (99, 99, 99) not in first | second


def test_cancel_sends_explicit_empty_array_and_clears_local_route(mission):
    mission.active_trajectory = np.ones((4, 4))
    mission.selected_targets = [np.ones(3)]
    mission._cancel_active_trajectory()
    mission.output.publish.assert_called_once_with([])
    assert mission.active_trajectory.shape == (0, 4)
    assert mission.selected_targets == []


def test_current_position_connector_cannot_cross_wall_even_when_old_route_is_safe(mission):
    mission.resolution = .1
    mission.active_trajectory = np.array([
        [.05, .05, .05, 0.], [1.05, .05, .05, 0.], [2.05, .05, .05, 0.]])
    occupied = {(7, 3, 0)}
    assert route_planning.path_is_collision_free(mission.active_trajectory[:, :3], occupied, .1)
    mission._motion_snapshot.return_value = (np.array([.55, .55, .05]), 0., [])
    assert not mission._active_trajectory_is_safe(occupied)
    assert mission._active_route_failure_reason == 'unsafe_current_route_entry'
    mission._cancel_active_trajectory()
    mission.output.publish.assert_called_once_with([])


def test_stalled_unfinished_route_is_cancelled_instead_of_pushing_forever(mission):
    mission.clock = Mock()
    mission.clock.perf_counter.side_effect = [0., 9., 10.]
    mission.route_stall_timeout_s = 10.
    mission.route_stall_min_progress_m = .15
    mission.active_trajectory = np.array([[.5, .5, .5, 0.], [3.5, .5, .5, 0.]])
    assert mission._active_trajectory_is_safe(set())
    assert mission._active_trajectory_is_safe(set())
    assert not mission._active_trajectory_is_safe(set())
    assert mission._active_route_failure_reason == 'active_route_stalled'
    mission._cancel_active_trajectory()
    mission.output.publish.assert_called_once_with([])
    assert mission._route_progress_time is None


def test_small_progress_resets_stall_timer_and_finished_target_is_not_stalled(mission):
    mission.clock = Mock()
    mission.clock.perf_counter.side_effect = [0., 9., 15., 30.]
    mission.active_trajectory = np.array([[.5, .5, .5, 0.], [3.5, .5, .5, 0.]])
    assert mission._active_trajectory_is_safe(set())
    mission._motion_snapshot.return_value = (np.array([.7, .5, .5]), 0., [])
    assert mission._active_trajectory_is_safe(set())
    assert mission._active_trajectory_is_safe(set())
    mission._motion_snapshot.return_value = (np.array([3.5, .5, .5]), 0., [])
    assert mission._active_trajectory_is_safe(set())


def test_obstacle_on_already_passed_route_does_not_block_safe_remaining_path(mission):
    mission.active_trajectory = np.array([
        [.5, .5, .5, 0.], [1.5, .5, .5, 0.], [2.5, .5, .5, 0.], [3.5, .5, .5, 0.]])
    mission._motion_snapshot.return_value = (np.array([2.5, .5, .5]), 0., [])
    assert mission._active_trajectory_is_safe({(0, 0, 0)})


def test_clock_reset_restarts_route_progress_window(mission):
    mission.clock = Mock()
    mission.clock.perf_counter.side_effect = [20., 10.]
    mission.active_trajectory = np.array([[.5, .5, .5, 0.], [3.5, .5, .5, 0.]])
    assert mission._active_trajectory_is_safe(set())
    assert mission._active_trajectory_is_safe(set())
    assert mission._route_progress_time == 10.


def test_timer_clears_stalled_queue_and_replans_without_waiting_for_period(mission):
    mission.clock = Mock()
    mission.clock.perf_counter.return_value = 10.
    mission.finished_mapping = False
    mission.odom_received = mission.rgb_received = True
    mission.depth_frame = np.ones((2, 2))
    mission.depth_sequence, mission.processed_depth_sequence = 2, 1
    mission._accumulate_depth = Mock()
    mission.octree = Mock()
    mission.octree.extractPointCloud.return_value = (np.empty((0, 3)),)
    mission.collision_radius = mission.collision_tracking_margin = 0.
    mission.replan_on_target_arrival = False
    mission.frame_period = 100
    mission.map_path = 'unused.bt'
    mission.nbv_parameters = {}
    from python_planner.domain.seabed import SeabedEstimate
    mission.seabed_estimate = SeabedEstimate(.2)
    x, y = np.meshgrid(np.arange(10) * .2, np.arange(10) * .2)
    mission.seabed_estimate.update(np.column_stack((x.ravel(), y.ravel(), np.full(x.size, 12.1))))
    mission.nbv_calculator = Mock(return_value=[candidate([3.5, .5, .5])])
    mission.observation_history = []
    mission._expire_target_blacklist = Mock()
    mission.blacklisted_targets = []
    mission.failed_plan_count = 0
    mission.evaluation_file = Mock()
    mission.position = np.array([.5, .5, .5])
    mission._plan_failure_reason = 'test_no_replacement'
    mission._failed_route_target = None
    mission._plan_and_publish = Mock(return_value=False)
    mission.active_trajectory = np.array([[.5, .5, .5, 0.], [3.5, .5, .5, 0.]])
    mission._route_progress_position = mission.position.copy()
    mission._route_progress_time = 0.
    mission.timer_callback()
    mission.output.publish.assert_called_once_with([])
    mission._plan_and_publish.assert_called_once()
    assert mission.active_trajectory.shape == (0, 4)
    parameters = mission.nbv_calculator.call_args.args[4]
    assert parameters['seabed_estimate_z'] == pytest.approx(12.1)
    assert parameters['seabed_sample_count'] == 100


def test_floor_estimation_uses_occupied_map_and_does_not_recount_unchanged_voxels(mission):
    from python_planner.domain.seabed import SeabedEstimate
    x, y = np.meshgrid(np.arange(10) * .2, np.arange(10) * .2)
    points = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, -210.1)))
    mission.seabed_estimate = SeabedEstimate(.2)
    mission.nbv_parameters = {'seabed_enabled': True}
    mission.finished_mapping = False
    mission.odom_received = mission.rgb_received = True
    mission.depth_frame = np.ones((2, 2))
    mission.depth_sequence, mission.processed_depth_sequence = 2, 1
    mission._accumulate_depth = Mock()
    mission.octree = Mock()
    mission.octree.extractPointCloud.return_value = (points,)
    mission.collision_radius = mission.collision_tracking_margin = 0.
    mission._active_trajectory_is_safe = Mock(return_value=True)
    mission.replan_on_target_arrival = False
    mission.frame_period = 100
    mission.timer_callback()
    mission.depth_sequence += 1
    mission.timer_callback()
    assert mission.seabed_estimate.sample_count == 100
    assert mission.seabed_estimate.mean_z == pytest.approx(-210.1)


@pytest.mark.parametrize('parameter,value', [
    ('route_stall_timeout_s', 0.), ('route_stall_timeout_s', float('nan')),
    ('route_stall_min_progress_m', -1.), ('route_stall_min_progress_m', float('inf'))])
def test_invalid_route_stall_settings_are_rejected(parameter, value):
    from python_planner.domain.configuration import load_configuration
    with pytest.raises(ValueError, match=parameter):
        load_configuration(lambda name, default: value if name == parameter else default)


def test_blocked_start_does_not_finish_mission_after_previous_failures(mission):
    mission.finished_mapping = False
    mission.odom_received = mission.rgb_received = True
    mission.depth_frame = np.ones((2, 2))
    mission.depth_sequence = 2
    mission.processed_depth_sequence = 1
    mission._accumulate_depth = Mock()
    mission.octree = Mock()
    mission.octree.extractPointCloud.return_value = (np.array([[.5, .5, .5]]),)
    mission.collision_radius = mission.collision_tracking_margin = 0.
    mission._active_trajectory_is_safe = Mock(return_value=True)
    mission.replan_on_target_arrival = False
    mission.frame_period = 1
    mission.map_path = 'unused.bt'
    mission.nbv_parameters = {}
    mission.nbv_calculator = Mock(return_value=[candidate([2.5, .5, .5])])
    mission.observation_history = []
    mission._expire_target_blacklist = Mock()
    mission.blacklisted_targets = []
    mission.failed_plan_count = 2
    mission.evaluation_file = Mock()
    mission.position = np.array([.5, .5, .5])
    mission.completion = Mock()
    mission.timer_callback()
    assert mission._plan_failure_reason == 'start_occupied'
    assert mission.failed_plan_count == 2
    assert not mission.finished_mapping
    assert mission.force_replan
    mission.completion.publish.assert_not_called()
