"""Exercise rejection diagnostics without starting ROS or the simulator."""

import ast
import importlib.util
import json
import inspect
from functools import partial
from dataclasses import dataclass
import math
from pathlib import Path
from types import SimpleNamespace
import time

import numpy as np
import pytest
from scipy.spatial import KDTree


ROOT = Path(__file__).parents[1]


def load_file(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


routes = load_file(ROOT / 'python_planner/route_planning.py', 'diagnostic_routes')
defaults = load_file(ROOT / 'python_planner/domain/nbv_parameters.py', 'diagnostic_defaults')
telemetry = load_file(
    ROOT.parent / 'telemetry_adapters/telemetry_adapters/file_sinks.py',
    'diagnostic_telemetry')


@pytest.fixture
def pipeline():
    source = ROOT / 'python_planner/use_cases/nbv.py'
    tree = ast.parse(source.read_text())
    nodes = [node for node in tree.body if isinstance(node, ast.FunctionDef)
             or (isinstance(node, ast.Assign)
                 and any(isinstance(target, ast.Name) and target.id == 'DEFAULT_NBV_PARAMETERS'
                         for target in node.targets))]

    class Cloud:
        def __init__(self):
            self.points = np.empty((0, 3))

        def voxel_down_sample(self, voxel_size):
            return self

        def remove_statistical_outlier(self, **kwargs):
            return self, list(range(len(self.points)))

        def select_by_index(self, indices):
            cloud = Cloud()
            cloud.points = self.points[indices]
            return cloud

        def estimate_normals(self, **kwargs):
            self.normals = np.tile([0., 0., 1.], (len(self.points), 1))

        def orient_normals_towards_camera_location(self, position):
            pass

    ns = dict(np=np, time=time, KDTree=KDTree, DEFAULT_NBV_PARAMETERS=defaults.DEFAULT_NBV_PARAMETERS,
              LOGGER=SimpleNamespace(info=lambda *args: None),
              o3d=SimpleNamespace(
                  geometry=SimpleNamespace(PointCloud=Cloud,
                                           KDTreeSearchParamHybrid=lambda **kwargs: None),
                  utility=SimpleNamespace(Vector3dVector=np.asarray)))
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), 'exec'), ns)
    adapter_source = ROOT / 'python_planner/adapters/point_cloud.py'
    adapter_nodes = [n for n in ast.parse(adapter_source.read_text()).body
                     if isinstance(n, ast.ClassDef)]
    ns['dataclass'] = dataclass
    exec(compile(ast.Module(body=adapter_nodes, type_ignores=[]), str(adapter_source), 'exec'), ns)
    ns['calculate_nbv_waypoints'] = partial(ns['calculate_nbv_waypoints'],
        preprocessor=ns['Open3DPreprocessor'](), diagnostics=ns['LOGGER'], clock=time)
    points = np.column_stack((np.linspace(0, 1, 60), np.zeros(60), np.zeros(60)))
    cluster = dict(id=0, centroid=np.array([0., 0., 0.]),
                   avg_normal=np.array([0., 0., 1.]), points=points,
                   normals=np.tile([0., 0., 1.], (60, 1)),
                   global_indices=np.arange(60))
    ns['calculate_pca_complexity'] = lambda cloud, **kwargs: np.ones(len(cloud.points))
    ns['detect_frontiers'] = lambda cloud, **kwargs: list(range(len(cloud.points)))
    ns['cluster_frontiers'] = lambda *args, **kwargs: [cluster]
    ns['generate_cone_viewpoints'] = lambda *args, **kwargs: [dict(position=np.array([0., 0., 1.]))]
    ns['evaluate_viewpoint'] = lambda *args, **kwargs: 1.0
    # Keep rejection/weight fixtures injectable at the scalar mathematical seam.
    # Production batching is compared against the scalar formulas separately.
    ns['evaluate_viewpoints'] = lambda positions, *args, **kwargs: np.array([
        ns['evaluate_viewpoint'](position, *args, **kwargs) for position in positions])
    ns['calculate_kinematic_costs'] = lambda auv, heading, positions, *args, **kwargs: np.array([
        ns['calculate_kinematic_cost'](auv, heading, position, *args, **kwargs)
        for position in positions])
    ns['calculate_kinematic_cost'] = lambda *args, **kwargs: 1.0
    return ns, points


@pytest.mark.parametrize('radius,expected', [(2., np.arange(18, 23)),
                                           (15., np.arange(5, 36))])
def test_focus_window_uses_auv_position_and_keeps_full_cloud_for_neighbors(pipeline, radius, expected):
    ns, _ = pipeline
    points = np.column_stack((np.arange(60.), np.zeros(60), np.zeros(60)))
    recorded = {}

    def detect(cloud, **kwargs):
        recorded.update(kwargs)
        recorded['points'] = np.asarray(cloud.points)
        return []

    ns['detect_frontiers'] = detect
    ns['cluster_frontiers'] = lambda *args, **kwargs: []
    assert ns['calculate_nbv_waypoints'](
        points, np.array([20., 0., 0.]), np.array([1., 0., 0.]), [],
        parameters={'exploration_radius': radius}) == []
    np.testing.assert_array_equal(recorded['focus_indices'], expected)
    np.testing.assert_array_equal(recorded['points'], points)
    np.testing.assert_array_equal(recorded['tree'].data, points)


@pytest.mark.parametrize('radius', [0., -1., np.inf, np.nan])
def test_focus_window_rejects_invalid_radius(pipeline, radius):
    ns, points = pipeline
    with pytest.raises(ValueError, match='exploration radius'):
        ns['calculate_nbv_waypoints'](
            points, np.zeros(3), np.array([1., 0., 0.]), [],
            parameters={'exploration_radius': radius})


def test_empty_focus_returns_before_heavy_math_and_logs_coverage(pipeline):
    ns, points = pipeline
    messages = []

    def unexpected(*args, **kwargs):
        pytest.fail('Empty focus should skip frontier detection and PCA')

    ns['detect_frontiers'] = ns['calculate_pca_complexity'] = unexpected
    result = ns['calculate_nbv_waypoints'](
        points, np.array([30., 0., 0.]), np.array([1., 0., 0.]), [],
        diagnostics=SimpleNamespace(info=messages.append))
    assert result == []
    assert 'Focus Window: Processing 0/60 points within 15.0m' in messages


def test_pca_only_scores_clustered_focus_points_with_global_indices(pipeline):
    ns, _ = pipeline
    points = np.column_stack((np.arange(60.), np.zeros(60), np.zeros(60)))
    selected = np.arange(5, 11)
    recorded = {}

    def cluster(surface, normals, frontier_indices, **kwargs):
        assert set(selected) <= set(frontier_indices)
        return [dict(id=0, centroid=surface[selected].mean(axis=0),
                     avg_normal=np.array([0., 0., 1.]), points=surface[selected],
                     normals=normals[selected], global_indices=selected)]

    def complexity(cloud, **kwargs):
        recorded.update(kwargs)
        return np.ones(len(cloud.points))

    ns['detect_frontiers'] = lambda cloud, **kwargs: kwargs['focus_indices'].tolist()
    ns['cluster_frontiers'] = cluster
    ns['calculate_pca_complexity'] = complexity
    result = ns['calculate_nbv_waypoints'](
        points, np.array([20., 0., 0.]), np.array([1., 0., 0.]), [])
    assert result
    np.testing.assert_array_equal(recorded['indices'], selected)
    np.testing.assert_array_equal(recorded['tree'].data, points)


@pytest.mark.parametrize('case,reason', [
    ('empty', 'occupied_input_empty'),
    ('z', 'world_z_removed_all'),
    ('small', 'minimum_cloud_size'),
    ('frontiers', 'no_frontiers'),
    ('clusters', 'clustering_removed_all'),
    ('cone', 'cone_generated_none'),
    ('gain', 'raw_gain_removed_all'),
])
def test_empty_pipeline_returns_no_candidates(pipeline, case, reason):
    ns, points = pipeline
    parameters = {'z_min_target': -1.0}
    history = []
    if case == 'empty':
        points = np.empty((0, 3))
    elif case == 'z':
        parameters['z_min_target'] = 0.0  # Equality must be rejected too.
    elif case == 'small':
        points = points[:4]
    elif case == 'frontiers':
        ns['detect_frontiers'] = lambda *args, **kwargs: []
        ns['cluster_frontiers'] = lambda *args, **kwargs: []
    elif case == 'clusters':
        ns['cluster_frontiers'] = lambda *args, **kwargs: []
    elif case == 'cone':
        ns['generate_cone_viewpoints'] = lambda *args, **kwargs: []
    elif case == 'gain':
        ns['evaluate_viewpoint'] = lambda *args, **kwargs: 0.0
    result = ns['calculate_nbv_waypoints'](
        points, np.zeros(3), np.array([1., 0., 0.]), history,
        parameters)
    assert result == []


def test_overall_cost_weight_changes_gain_cost_tradeoff(pipeline):
    ns, points = pipeline
    ns['generate_cone_viewpoints'] = lambda *args, **kwargs: [
        dict(position=np.array([0., 0., z])) for z in [1., 2.]]
    ns['evaluate_viewpoint'] = lambda position, *args, **kwargs: position[2]
    ns['calculate_kinematic_cost'] = lambda position, heading, viewpoint, *args, **kwargs: viewpoint[2]
    args = (points, np.zeros(3), np.array([1., 0., 0.]))
    cost_favored = ns['calculate_nbv_waypoints'](*args, parameters={'utility_cost_weight': 2.})
    gain_favored = ns['calculate_nbv_waypoints'](*args, parameters={'utility_cost_weight': .5})
    assert cost_favored[0]['position'][2] == 1.
    assert gain_favored[0]['position'][2] == 2.
    assert gain_favored[0]['utility'] == pytest.approx(.5)
    gain_scaled = ns['calculate_nbv_waypoints'](
        *args, parameters={'utility_gain_weight': 3., 'utility_cost_weight': 2.})
    assert gain_scaled[0]['position'][2] == 2.
    assert gain_scaled[0]['utility'] == pytest.approx(1.)


def test_axis_selection_receives_raw_occupied_map_not_cleaned_target(pipeline):
    ns, points = pipeline
    raw = np.vstack((points, [0., 0., -10.]))
    selected = []
    def axis(target, position, occupied_tree, **kwargs):
        selected.append(occupied_tree.data.copy())
        return np.array([1., 0., 1.]) / np.sqrt(2.)
    ns['select_upward_view_axis'] = axis
    ns['calculate_nbv_waypoints'](
        raw, np.zeros(3), np.array([1., 0., 0.]), parameters={'z_min_target': -1.})
    assert len(selected) == 1
    np.testing.assert_array_equal(selected[0], raw)


def test_timing_preserves_candidates(pipeline):
    ns, points = pipeline
    args = (points, np.zeros(3), np.array([1., 0., 0.]))
    events = []
    baseline = ns['calculate_nbv_waypoints'](*args)
    logged = ns['calculate_nbv_waypoints'](*args, timing_callback=lambda *e: events.append(e))
    assert len(baseline) == len(logged) == 1
    for key in baseline[0]:
        np.testing.assert_equal(baseline[0][key], logged[0][key])
    assert events
    assert all(duration >= 0. for stage, duration in events)


def test_pose_history_removes_matching_frontier(pipeline):
    ns, points = pipeline
    result = ns['calculate_nbv_waypoints'](
        points, np.zeros(3), np.array([1., 0., 0.]),
        [(np.array([-1., 0., 0.]), 0.0)])
    assert result == []


def test_cluster_rejects_groups_below_minimum_sample_count(pipeline):
    ns, _ = pipeline
    source = ast.parse((ROOT / 'python_planner/use_cases/nbv.py').read_text())
    function = next(n for n in source.body if isinstance(n, ast.FunctionDef)
                    and n.name == 'cluster_frontiers')
    exec(compile(ast.Module(body=[function], type_ignores=[]), '<cluster>', 'exec'), ns)
    result = ns['cluster_frontiers'](
        np.array([[1., 0., 0.], [1.1, 0., 0.]]), np.tile([0., 0., 1.], (2, 1)),
        [0, 1], min_samples=5)
    assert result == []


def test_node_nbv_call_matches_restored_signature(pipeline):
    ns, _ = pipeline
    tree = ast.parse((ROOT / 'python_planner/use_cases/mission.py').read_text())
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Attribute) and node.func.attr == 'nbv_calculator']
    assert calls
    for call in calls:
        inspect.signature(ns['calculate_nbv_waypoints']).bind(
            *[object() for _ in call.args], **{kw.arg: object() for kw in call.keywords})


def test_pipeline_passes_complexity_weight_to_gain_calculation(pipeline):
    ns, points = pipeline
    source = ROOT / 'python_planner/use_cases/nbv.py'
    function = next(node for node in ast.parse(source.read_text()).body
                    if isinstance(node, ast.FunctionDef) and node.name == 'evaluate_viewpoint')
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), ns)
    args = (points, np.zeros(3), np.array([1., 0., 0.]))
    zero = ns['calculate_nbv_waypoints'](*args, parameters={'complexity_weight': 0.})
    weighted = ns['calculate_nbv_waypoints'](*args, parameters={'complexity_weight': 4.})
    assert len(zero) == len(weighted) == 1
    assert zero[0]['raw_gain'] > 0.
    # Fixture complexities are all one, so each point's gain increases fivefold.
    assert weighted[0]['raw_gain'] == pytest.approx(5. * zero[0]['raw_gain'])


@pytest.mark.parametrize('occupied,budget,found', [
    ({(2, 0, 0)}, 20, False),
    (set(), 1, False),
    (set(), 20, True),
])
def test_astar_return_matches_occupied_endpoints_and_expansion_limit(occupied, budget, found):
    args = (np.array([.5, .5, .5]), np.array([2.5, .5, .5]), occupied, 1.0)
    assert bool(routes.astar_3d(*args, max_steps=budget)) == found


def test_collision_rejects_occupied_voxel_between_endpoints():
    path = np.array([[.5, .5, .5], [2.5, .5, .5]])
    assert not routes.path_is_collision_free(
        path, {(1, 0, 0)}, 1.0)


def test_route_skips_occupied_viewpoint():
    result, targets = routes.plan_multi_waypoint_route(
        np.array([.5, .5, .5]),
        [(np.array([2.5, .5, .5]), np.array([3., 0., 0.]))],
        {(2, 0, 0)}, 1.0)
    assert len(result) == len(targets) == 0


def test_jsonl_logs_iteration_and_handles_nonfinite_values(tmp_path):
    logger = telemetry.MidrunLogger(tmp_path, prefix='planner')
    logger.record_filter('world_z', 60, 0,
                         {'threshold': np.float64(-298), 'value': np.nan},
                         iteration=100, frame_id=101)
    event = json.loads(logger.filter_path.read_text())
    assert event['iteration'] == 100 and event['frame_id'] == 101
    assert event['input_count'] == 60 and event['output_count'] == 0
    assert event['details'] == {'threshold': -298.0, 'value': 'nan'}


def node_method(name, **extra_globals):
    source = ROOT / 'python_planner/use_cases/mission.py'
    tree = ast.parse(source.read_text())
    node_class = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    method = next(node for node in node_class.body
                  if isinstance(node, ast.FunctionDef) and node.name == name)
    namespace = dict(np=np, math=math,
                     Float32MultiArray=lambda: SimpleNamespace(data=[]),
                     path_is_collision_free=routes.path_is_collision_free)
    namespace.update(extra_globals)
    helpers = [node for node in tree.body if isinstance(node, ast.FunctionDef)
               and node.name in ('body_rotation', 'vehicle_viewpoint')]
    exec(compile(ast.Module(body=[*helpers, method], type_ignores=[]), str(source), 'exec'), namespace)
    # Method fixtures supply the same injected ports as the production composition root.
    output_source = ROOT / 'python_planner/adapters/ros_output.py'
    output_nodes = [n for n in ast.parse(output_source.read_text()).body
                    if isinstance(n, ast.ClassDef) and n.name == 'RosTrajectoryOutput']
    exec(compile(ast.Module(body=output_nodes, type_ignores=[]), str(output_source), 'exec'), namespace)
    def invoke(node, *args, **kwargs):
        node.clock = extra_globals.get('time', time)
        if hasattr(node, 'get_logger'):
            node.diagnostics = node.get_logger()
        if hasattr(node, 'waypoint_pub'):
            node.output = namespace['RosTrajectoryOutput'](node.waypoint_pub)
        return namespace[name](node, *args, **kwargs)
    return invoke


def test_node_reports_utility_removing_all_candidates():
    events = []
    node = SimpleNamespace(min_utility=.1, _log_nbv_filter=lambda *e: events.append(e))
    candidate = dict(cluster_id=0, utility=0., target=np.zeros(3), position=np.ones(3))
    assert node_method('_plan_and_publish')(node, [candidate], set()) is False
    assert node._plan_failure_reason == 'minimum_utility_removed_all'
    assert events[-1][:3] == ('minimum_utility', 1, 0)
    assert events[0][3]['utility'] == 0


def test_depth_range_diagnostics_count_disjoint_rejections():
    events = []
    node = SimpleNamespace(depth_scale=1., depth_stride=1, min_depth=.5, max_depth=4.,
                           latched_roll=0., latched_pitch=0., latched_yaw=0.,
                           latched_position=np.zeros(3), camera_offset=np.zeros(3),
                           _log_nbv_filter=lambda *e: events.append(e))
    points = node_method('_depth_points')(node, np.array([[np.nan, .5, 4., 1.]]))
    assert len(points) == 1
    assert events[0][:3] == ('depth_range', 4, 1)
    assert events[0][3]['nonfinite'] == 1
    assert events[0][3]['at_or_below_min'] == 1
    assert events[0][3]['at_or_above_max'] == 1


def test_disabled_node_logging_does_not_access_logger_or_files():
    node = SimpleNamespace(nbv_filter_logging=False)
    node_method('_log_nbv_filter')(node, 'test', 1, 0, {})


def test_blocked_route_cancellation_reaches_downstream_queue():
    messages = []
    events = []
    node = SimpleNamespace(active_trajectory=np.ones((3, 4)),
                           selected_targets=[np.ones(3)],
                           waypoint_pub=SimpleNamespace(publish=messages.append),
                           _log_nbv_filter=lambda *event: events.append(event))
    node_method('_cancel_active_trajectory')(node)
    assert len(messages) == 1 and messages[0].data == []
    assert node.active_trajectory.shape == (0, 4)
    assert node.selected_targets == []
    assert events[0][:3] == ('trajectory_cancelled', 3, 0)




def test_depth_sensor_mount_offset_rotates_with_vehicle():
    node = SimpleNamespace(depth_scale=1., depth_stride=1, min_depth=.5, max_depth=4.,
                           latched_roll=0., latched_pitch=0., latched_yaw=math.pi / 2,
                           latched_position=np.array([10., 20., -5.]),
                           camera_offset=np.array([1., 0., -.5]),
                           _log_nbv_filter=lambda *args: None)
    depth = np.full((2, 2), np.nan)
    depth[1, 1] = 2.
    points = node_method('_depth_points')(node, depth)
    np.testing.assert_allclose(node._depth_sensor_origin, [10., 21., -5.5])
    np.testing.assert_allclose(points[0], [10., 21. + math.sqrt(2), -5.5 - math.sqrt(2)])


def test_vehicle_viewpoint_places_rotated_mount_at_requested_camera_position():
    camera = np.array([5., 8., -3.])
    target = np.array([4., 10., -4.])
    offset = np.array([.35, .1, -.25])
    source = ROOT / 'python_planner/use_cases/mission.py'
    helpers = [node for node in ast.parse(source.read_text()).body
               if isinstance(node, ast.FunctionDef) and node.name in ('body_rotation', 'vehicle_viewpoint')]
    namespace = dict(np=np, math=math)
    exec(compile(ast.Module(body=helpers, type_ignores=[]), str(source), 'exec'), namespace)
    vehicle = namespace['vehicle_viewpoint'](camera, target, offset)
    yaw = math.atan2(target[1] - vehicle[1], target[0] - vehicle[0])
    np.testing.assert_allclose(vehicle + namespace['body_rotation'](0., 0., yaw) @ offset,
                               camera, atol=1e-7)


@pytest.mark.parametrize('first_spline_safe', [True, False])
def test_rejected_combined_spline_keeps_checked_first_structural_target(first_spline_safe):
    route = np.array([[.5, .5, .5], [1.5, .5, .5], [2.5, .5, .5], [3.5, .5, .5],
                      [3.5, 1.5, .5], [3.5, 2.5, .5], [2.5, 2.5, .5], [1.5, 2.5, .5]])
    targets = np.array([[4., 0., 0.]] * 4 + [[1., 3., 0.]] * 4)
    occupied = {(2, 1, 0)}
    events, published = [], []
    logger = SimpleNamespace(info=lambda *args: None, warning=lambda *args: None)
    node = SimpleNamespace(
        min_utility=.1, camera_offset=np.zeros(3), velocity_history_window=1.,
        trajectory_history_window=5., astar_max_steps=50000, optimize_motion=False,
        motion_cost_parameters={}, resolution=1., route_search_time_budget_s=2.,
        route_max_candidates=4, collision_check_spacing_factor=.5, nbv_filter_logging=True,
        telemetry=SimpleNamespace(record_stage=lambda *args, **kwargs: None), iteration=1,
        trajectory_points=300, spline_smoothing=2., waypoint_spacing=.5,
        _log_nbv_filter=lambda *event: events.append(event),
        _motion_snapshot=lambda: (route[0], 0., []), get_logger=lambda: logger,
        _publish_trajectory=lambda trajectory, occupancy: published.append(trajectory) or True)
    def spline(path, tracked, **kwargs):
        if len(path) == len(route) or not first_spline_safe:
            # Model a combined spline cutting the obstacle inside a safe A* detour.
            positions = np.vstack((path[0], [2.5, 1.5, .5], path[-1]))
            return np.column_stack((positions, np.zeros(3)))
        return routes.generate_smooth_trajectory(path, tracked, **kwargs)
    def route_with_utilities(*args, **kwargs):
        assert kwargs['candidate_utilities'] == [1.]
        return route, targets

    method = node_method('_plan_and_publish', time=time,
                         estimate_velocity=lambda *args: np.zeros(3),
                         plan_multi_waypoint_route=route_with_utilities,
                         generate_smooth_trajectory=spline,
                         resample_trajectory_by_distance=routes.resample_trajectory_by_distance,
                         reconnect_trajectory=routes.reconnect_trajectory)
    # Utility order can differ from A* travel order: do not blame this top target.
    candidate = dict(cluster_id=0, utility=1., target=targets[-1], position=route[-1])
    result = method(node, [candidate], occupied)
    if not first_spline_safe:
        assert not result and published == []
        assert node._plan_failure_reason == 'all_trajectory_options_rejected'
        np.testing.assert_equal(node._failed_route_target, targets[0])
        return
    assert result
    assert len(published) == 1
    assert routes.path_is_collision_free(published[0][:, :3], occupied, 1.)
    np.testing.assert_allclose(published[0][-1, :3], route[3])
    assert all(np.array_equal(target, targets[0]) for target in node.selected_targets)
    options = [event for event in events if event[0] == 'trajectory_option']
    assert [(event[3]['option'], event[3]['collision_free']) for event in options] == [
        ('spline', False), ('first_target_spline', True)]


def test_vehicle_route_is_checked_instead_of_camera_offset_route():
    messages = []
    node = SimpleNamespace(camera_offset=np.array([1., 0., 0.]), resolution=1.,
                           waypoint_pub=SimpleNamespace(publish=messages.append),
                           _log_nbv_filter=lambda *args: None)
    body = np.array([[.5, .5, .5, 0.], [.5, 2.5, .5, 0.]])
    occupied = {(0, 1, 0)}
    assert routes.path_is_collision_free(body[:, :3] + node.camera_offset, occupied, 1.)
    assert node_method('_publish_trajectory')(node, body, occupied) is False
    assert messages == []


def test_wire_validation_rejects_float32_rounding_into_occupied_voxel():
    messages = []
    node = SimpleNamespace(camera_offset=np.array([.35, 0., 0.]), resolution=1.,
                           waypoint_pub=SimpleNamespace(publish=messages.append),
                           _log_nbv_filter=lambda *args: None)
    body = np.array([[.99999999, .5, .5, 0.], [.99999999, 1.5, .5, 0.]])
    occupied = {(1, 0, 0)}
    assert routes.path_is_collision_free(body[:, :3], occupied, 1.)
    assert node_method('_publish_trajectory')(node, body, occupied) is False
    assert messages == []


def test_failed_target_blacklist_expires_and_requests_another_target():
    events = []
    clock = [100.]
    failed = np.array([357.6, 828.4, -296.1])
    node = SimpleNamespace(_failed_route_target=failed, blacklist_cooldown_s=30.,
                           _plan_failure_reason='all_trajectory_options_rejected',
                           blacklisted_targets=[], blacklist_expiries={}, force_replan=False,
                           _log_nbv_filter=lambda *event: events.append(event),
                           get_logger=lambda: SimpleNamespace(warning=lambda *args: None))
    fake_time = SimpleNamespace(perf_counter=lambda: clock[0])
    blacklist = node_method('_blacklist_failed_target', time=fake_time)
    expire = node_method('_expire_target_blacklist', time=fake_time)
    assert blacklist(node)
    assert node.force_replan
    np.testing.assert_equal(node.blacklisted_targets, [failed])
    assert node.blacklist_expiries[tuple(failed)] == 130.
    assert events[-1][3]['expires'] is True
    clock[0] = 129.
    expire(node)
    assert len(node.blacklisted_targets) == 1
    clock[0] = 130.
    expire(node)
    assert node.blacklisted_targets == [] and node.blacklist_expiries == {}
    assert events[-1][0] == 'blacklist_expired'


def test_failed_target_blacklist_deduplicates_nearby_cluster_drift():
    first = np.array([1., 2., 3.])
    node = SimpleNamespace(_failed_route_target=first, blacklist_cooldown_s=30.,
                           _plan_failure_reason='all_trajectory_options_rejected',
                           blacklisted_targets=[], blacklist_expiries={}, force_replan=False,
                           _log_nbv_filter=lambda *args: None,
                           get_logger=lambda: SimpleNamespace(warning=lambda *args: None))
    method = node_method('_blacklist_failed_target', time=SimpleNamespace(perf_counter=lambda: 100.))
    assert method(node)
    node._failed_route_target = first + [.1, 0., 0.]
    assert method(node)
    assert len(node.blacklisted_targets) == len(node.blacklist_expiries) == 1


@pytest.mark.parametrize('failed,cooldown', [(None, 30.), (np.ones(3), 0.)])
def test_global_failure_or_disabled_cooldown_does_not_ban_a_target(failed, cooldown):
    node = SimpleNamespace(_failed_route_target=failed, blacklist_cooldown_s=cooldown,
                           blacklisted_targets=[], blacklist_expiries={}, force_replan=False)
    assert node_method('_blacklist_failed_target')(node) is False
    assert node.blacklisted_targets == [] and not node.force_replan
