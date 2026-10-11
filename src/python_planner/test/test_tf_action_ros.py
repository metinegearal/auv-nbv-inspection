"""Exercise real tf2 buffers and action transport without simulator/model startup."""
import ast
import io
from pathlib import Path
import subprocess
from threading import Lock
import time
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('nbv_interfaces.action')
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from nbv_interfaces.action import ComputeNBV
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from std_msgs.msg import Bool, String, Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener

from python_planner.adapters.transforms import TfCameraTransforms
from python_planner.domain.configuration import load_configuration
from python_planner.use_cases.mission import InspectionMission
from bringup.transforms import camera_transform_arguments


ROOT = Path(__file__).resolve().parents[3]


def transform(parent, child, seconds, position, rotation=None):
    msg = TransformStamped()
    msg.header.frame_id, msg.child_frame_id = parent, child
    msg.header.stamp.sec = seconds
    t, q = msg.transform.translation, msg.transform.rotation
    t.x, t.y, t.z = map(float, position)
    q.x, q.y, q.z, q.w = map(float, (rotation or Rotation.identity()).as_quat())
    return msg


def test_projection_tracks_tf_time_and_sensor_reconfiguration():
    buffer = Buffer()
    buffer.set_transform_static(transform('base_link', 'camera_link', 0, [1., 0., 0.]), 'test')
    buffer.set_transform(transform('map', 'base_link', 10, [10., 0., 0.]), 'test')
    buffer.set_transform(transform('map', 'base_link', 20, [20., 0., 0.]), 'test')
    adapter = TfCameraTransforms(buffer, 'map', 'camera_link', 'base_link')
    points, origin = adapter.project_depth([[0., 0., 2.]], SimpleNamespace(sec=15, nanosec=0))
    np.testing.assert_allclose(origin, [16., 0., 0.])
    np.testing.assert_allclose(points, [[16., 0., 2.]])
    with pytest.raises(TransformException):
        adapter.project_depth([[0., 0., 2.]], SimpleNamespace(sec=21, nanosec=0))
    with pytest.raises(TransformException):
        adapter.project_depth([[0., 0., 2.]], SimpleNamespace(sec=0, nanosec=0))
    buffer.set_transform_static(transform('base_link', 'camera_link', 0, [3., 0., 0.]), 'test')
    points, origin = adapter.project_depth([[0., 0., 2.]], SimpleNamespace(sec=15, nanosec=0))
    np.testing.assert_allclose(origin, [18., 0., 0.])
    np.testing.assert_allclose(points, [[18., 0., 2.]])


def test_inverse_mount_placement_uses_real_tf_rotation():
    buffer = Buffer()
    translation = np.array([.8, .2, -.3])
    rotation = Rotation.from_euler('xyz', [.3, .7, -.4])
    buffer.set_transform_static(
        transform('base_link', 'camera_link', 0, translation, rotation), 'test')
    adapter = TfCameraTransforms(buffer, 'map', 'camera_link', 'base_link')
    camera, target = np.array([5., 8., -3.]), np.array([4., 10., -4.])
    vehicle = adapter.vehicle_viewpoint(camera, target)
    yaw = np.arctan2(target[1] - vehicle[1], target[0] - vehicle[0])
    np.testing.assert_allclose(
        vehicle + Rotation.from_euler('z', yaw).apply(translation), camera, atol=1e-7)


@pytest.mark.parametrize('camera,target', [
    ([0., 0., 1.5], [0., -.4, 0.]),
    # The positive-utility candidate that aborted the user's first planning pass.
    ([351.51388213295496, 830.0500964586737, -297.65825411770925],
     [351.5571428571429, 829.6642857142857, -299.10714285714283]),
])
def test_inverse_placement_handles_close_lateral_viewpoints(camera, target):
    buffer = Buffer()
    mount = np.array([.35, .3875, -.5])
    rotation = Rotation.from_euler('xyz', [.2, .6, -.3])
    buffer.set_transform_static(
        transform('base_link', 'camera_link', 0, mount, rotation), 'test')
    adapter = TfCameraTransforms(buffer, 'map', 'camera_link', 'base_link')
    camera, target = np.asarray(camera), np.asarray(target)
    vehicle = adapter.vehicle_viewpoint(camera, target)
    yaw = np.arctan2(target[1] - vehicle[1], target[0] - vehicle[0])
    np.testing.assert_allclose(
        vehicle + Rotation.from_euler('z', yaw).apply(mount), camera, atol=1e-9)


def test_inverse_placement_rejects_physically_unreachable_viewpoints():
    buffer = Buffer()
    buffer.set_transform_static(
        transform('base_link', 'camera_link', 0, [.35, .3875, -.5]), 'test')
    adapter = TfCameraTransforms(buffer, 'map', 'camera_link', 'base_link')
    with pytest.raises(ValueError, match='too close laterally'):
        adapter.vehicle_viewpoint(np.array([0., 0., 1.5]), np.array([0., .1, 0.]))


# Load the production ROS adapter class, injecting only its costly mission
# composition. All action/timer/lifecycle methods remain production code.
source = ROOT / 'src/python_planner/python_planner/planner_node.py'
node_class = next(n for n in ast.parse(source.read_text()).body
                  if isinstance(n, ast.ClassDef) and n.name == 'NBVUtilityNode')
namespace = dict(Node=Node, ComputeNBV=ComputeNBV, Future=Future,
                 GoalResponse=GoalResponse, CancelResponse=CancelResponse,
                 TransformException=TransformException)
exec(compile(ast.Module(body=[node_class], type_ignores=[]), str(source), 'exec'), namespace)


class Harness(namespace['NBVUtilityNode']):
    def __init__(self, outcome='waiting'):
        Node.__init__(self, 'nbv_transport_test')
        self.world_frame = 'map'
        self._goal_lock = Lock()
        self._goal_reserved = False
        self._goal_handle = self._goal_result = None
        self.outcome, self.passes = outcome, 0
        self.mission = SimpleNamespace(
            trajectory_revision=0, planning_enabled=False, force_replan=False,
            finished_mapping=False, failed_plan_count=0, failure_reason='',
            planning_state='WaitingForSensors', blacklisted_targets=[],
            active_trajectory=np.array([[1., 2., -3., .4], [4., 5., -6., .7]]),
            timer_callback=self.step)
        self.action_server = ActionServer(
            self, ComputeNBV, '/planner/compute_nbv', self.execute_nbv,
            goal_callback=self.goal_callback, cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.accepted_callback)
        self.timer = self.create_timer(.03, self.timer_callback)

    def step(self):
        if not self.mission.planning_enabled:
            return
        if self.outcome == 'startup':
            self.mission.planning_state = 'WaitingForPointCloud'
            self.publish_feedback('WaitingForPointCloud')
            return
        self.passes += 1
        self.mission.planning_state = 'Computing'
        self.publish_feedback('Computing')
        if self.outcome == 'success':
            self.mission.trajectory_revision += 1
        elif self.outcome == 'strikeout':
            self.mission.failed_plan_count += 1
            self.mission.failure_reason = 'no_candidates'
            self.mission.finished_mapping = self.mission.failed_plan_count >= 3
        elif self.outcome == 'error':
            raise RuntimeError('test pipeline exception')

    def destroy_node(self):
        self.action_server.destroy()
        return Node.destroy_node(self)


class NullPorts:
    def __getattr__(self, name):
        return lambda *args, **kwargs: None


class VehicleOutput:
    def encode(self, trajectory):
        wire = np.asarray(trajectory, dtype=np.float32)
        return wire, wire.astype(np.float64)

    def publish(self, values):
        pass


@pytest.fixture
def transport():
    rclpy.init()
    server, client_node = Harness(), Node('nbv_transport_client')
    client = ActionClient(client_node, ComputeNBV, '/planner/compute_nbv')
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server)
    executor.add_node(client_node)
    assert client.wait_for_server(timeout_sec=5.)
    def wait(future=None, condition=None, timeout=5.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if future is not None and future.done():
                return future.result()
            if condition is not None and condition():
                return
        raise AssertionError('ROS transport test timed out')
    yield server, client, client_node, wait
    server.timer.cancel()
    # Drain accepted/execution callbacks before closing a pending monitor goal.
    end = time.monotonic() + .1
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=.01)
    if server._goal_handle is not None:
        server._finish_goal('aborted', 'Test teardown')
    end = time.monotonic() + .1
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=.01)
    executor.shutdown()
    client.destroy()
    server.destroy_node()
    client_node.destroy_node()
    rclpy.shutdown()


def request(client, wait, feedback=None, force=True):
    return wait(client.send_goal_async(
        ComputeNBV.Goal(trigger=True, force_replan=force), feedback_callback=feedback))


def test_action_success_and_feedback(transport):
    server, client, _, wait = transport
    server.outcome = 'success'
    feedback = []
    goal = request(client, wait, lambda msg: feedback.append(msg.feedback.state))
    assert goal.accepted
    result = wait(goal.get_result_async())
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    np.testing.assert_allclose(result.result.trajectory, server.mission.active_trajectory.flatten())
    assert result.result.frame_id == 'map'
    assert 'Computing' in feedback
    assert not server.mission.planning_enabled


def test_real_mission_waits_for_cloud_then_returns_first_route(transport):
    server, client, _, wait = transport
    buffer = Buffer()
    buffer.set_transform_static(transform('map', 'base_link', 0, [350., 828., -298.]), 'test')
    buffer.set_transform_static(
        transform('base_link', 'camera_link', 0, [.35, .3875, -.5],
                  Rotation.from_euler('xyz', [-90., 45., -90.], degrees=True)), 'test')
    candidate = dict(utility=1.,
                    position=np.array([351.51388213295496, 830.0500964586737, -297.65825411770925]),
                    target=np.array([351.5571428571429, 829.6642857142857, -299.10714285714283]))
    config = load_configuration(lambda name, default: {
        'planning_period': 100, 'nbv_filter_logging': False}.get(name, default))
    server.mission = InspectionMission(
        config, NullPorts(), NullPorts(), io.StringIO(), NullPorts(), time,
        VehicleOutput(), lambda *args, **kwargs: [candidate], lambda *args: None,
        'test', TfCameraTransforms(buffer, 'map', 'camera_link', 'base_link'),
        server.publish_feedback)
    mission = server.mission
    mission.odom_received = mission.rgb_received = True
    mission.position = np.array([350., 828., -298.])
    mission.depth_frame = np.ones((2, 2))
    mission._accumulate_depth = lambda: None
    mission._occupied_points = lambda: np.empty((0, 3))
    real_timer = mission.timer_callback
    def frame():
        mission.depth_sequence += 1
        real_timer()
    mission.timer_callback = frame
    feedback = []
    goal = request(client, wait, lambda msg: feedback.append(msg.feedback.state))
    result_future = goal.get_result_async()
    wait(condition=lambda: feedback.count('WaitingForPointCloud') >= 4)
    assert not result_future.done()
    assert mission.failed_plan_count == 0 and not mission.finished_mapping
    mission._occupied_points = lambda: np.column_stack((
        np.arange(40.) + 400., np.full(40, 800.), np.full(40, -300.)))
    result = wait(result_future)
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert len(result.result.trajectory) >= 4 and result.result.strikes == 0
    assert mission.map_ready


def test_single_goal_rejection_and_cancellation(transport):
    server, client, _, wait = transport
    goal = request(client, wait)
    assert goal.accepted
    assert not request(client, wait).accepted
    assert not wait(client.send_goal_async(ComputeNBV.Goal(trigger=False))).accepted
    canceled = wait(goal.cancel_goal_async())
    assert canceled.goals_canceling
    result = wait(goal.get_result_async())
    assert result.status == GoalStatus.STATUS_CANCELED
    assert not server.mission.planning_enabled
    server.outcome = 'success'
    replacement = request(client, wait)
    assert wait(replacement.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED


@pytest.mark.parametrize('outcome,strikeout', [('strikeout', True), ('error', False)])
def test_aborted_action_reports_strikeout_or_pipeline_failure(transport, outcome, strikeout):
    server, client, _, wait = transport
    server.outcome = outcome
    goal = request(client, wait)
    result = wait(goal.get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED
    assert result.result.strikeout is strikeout
    assert len(result.result.trajectory) == 0
    assert result.result.message
    assert result.result.strikes == (3 if strikeout else 0)


@pytest.mark.parametrize('outcome,expected', [('strikeout', 'MissionComplete'), ('waiting', 'Failed')])
def test_cpp_client_stops_on_aborted_result_or_silent_server(transport, outcome, expected):
    server, _, client_node, wait = transport
    executable = ROOT / 'install/motion_control/lib/motion_control/point_iterator'
    if not executable.exists():
        pytest.skip('Build motion_control to run the C++ action integration test')
    server.outcome = outcome
    states, stops, routes = [], [], []
    client_node.create_subscription(String, '/planner/state', lambda m: states.append(m.data), 10)
    client_node.create_subscription(Bool, '/movement/finished_execution', lambda m: stops.append(m.data), 10)
    client_node.create_subscription(Float32MultiArray, '/trajectory/waypoint', lambda m: routes.append(m.data), 10)
    process = subprocess.Popen([
        str(executable), '--ros-args', '-p', 'timer_period_s:=0.05',
        '-p', 'nbv_feedback_timeout_s:=0.4', '-p', 'nbv_goal_timeout_s:=1.5',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        # Simulate Python crashing after accepting a goal: no more timer feedback.
        if outcome == 'waiting':
            wait(condition=lambda: server._goal_handle is not None)
            server.timer.cancel()
        wait(condition=lambda: expected in states and True in stops)
        if outcome == 'waiting':
            server.timer.reset()
            wait(condition=lambda: server._goal_handle is None)
        end = time.monotonic() + .25
        while time.monotonic() < end:
            # State and waypoint topics can deliver pre-stop messages in either order.
            server.executor.spin_once(timeout_sec=.02)
        count = len(routes)
        end = time.monotonic() + .25
        while time.monotonic() < end:
            server.executor.spin_once(timeout_sec=.02)
        assert len(routes) == count
    finally:
        process.terminate()
        process.wait(timeout=5.)


def test_static_camera_publisher_uses_launch_configuration(transport):
    _, _, client_node, wait = transport
    buffer = Buffer(node=client_node)
    listener = TransformListener(buffer, client_node)
    process = subprocess.Popen([
        '/opt/ros/jazzy/lib/tf2_ros/static_transform_publisher',
        *camera_transform_arguments(ROOT / 'src/bringup/config/params.yaml',
                                    ROOT / 'src/bringup/config/params.yaml'),
        '--ros-args', '-r', '__node:=camera_static_transform',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        wait(condition=lambda: buffer.can_transform('base_link', 'camera_link', Time()))
        adapter = TfCameraTransforms(buffer, 'base_link', 'camera_link', 'base_link')
        points, origin = adapter.project_depth([[0., 0., 2.]], SimpleNamespace(sec=1, nanosec=0))
        np.testing.assert_allclose(origin, [.35, .3875, -.5])
        np.testing.assert_allclose(points[0] - origin, [np.sqrt(2), 0., -np.sqrt(2)], atol=1e-8)
    finally:
        process.terminate()
        process.wait(timeout=5.)
        listener.unregister()


def test_cpp_client_executes_vehicle_coordinates_and_ignores_observer_routes(transport):
    server, _, client_node, wait = transport
    executable = ROOT / 'install/motion_control/lib/motion_control/point_iterator'
    if not executable.exists():
        pytest.skip('Build motion_control to run the C++ action integration test')
    server.outcome = 'success'
    routes, stops, states = [], [], []
    client_node.create_subscription(Float32MultiArray, '/trajectory/waypoint', lambda m: routes.append(list(m.data)), 10)
    client_node.create_subscription(Bool, '/movement/finished_execution', lambda m: stops.append(m.data), 10)
    client_node.create_subscription(String, '/planner/state', lambda m: states.append(m.data), 10)
    observer = client_node.create_publisher(Float32MultiArray, 'object/areaWaypoints', 10)
    process = subprocess.Popen([
        str(executable), '--ros-args', '-p', 'timer_period_s:=0.05',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        expected = server.mission.active_trajectory.astype(np.float32).flatten().tolist()
        wait(condition=lambda: expected in routes)
        server.outcome = 'waiting'
        observer.publish(Float32MultiArray(data=[99., 99., 99., 99.]))
        wait(condition=lambda: server._goal_handle is not None)
        assert not server._goal_handle.request.force_replan
        assert [99., 99., 99., 99.] not in routes
        assert False in stops
        # Invalidating an executing route holds movement and forces a new goal.
        observer.publish(Float32MultiArray())
        wait(condition=lambda: True in stops and server._goal_handle is not None
             and server._goal_handle.request.force_replan)
        server.outcome = 'success'
        wait(condition=lambda: stops[-1] is False)
        server.outcome = 'strikeout'
        wait(condition=lambda: 'MissionComplete' in states)
    finally:
        process.terminate()
        process.wait(timeout=5.)


def test_cpp_client_does_not_time_out_healthy_cloud_startup(transport):
    server, _, client_node, wait = transport
    executable = ROOT / 'install/motion_control/lib/motion_control/point_iterator'
    if not executable.exists():
        pytest.skip('Build motion_control to run the C++ action integration test')
    server.outcome = 'startup'
    states, stops = [], []
    client_node.create_subscription(String, '/planner/state', lambda m: states.append(m.data), 10)
    client_node.create_subscription(Bool, '/movement/finished_execution', lambda m: stops.append(m.data), 10)
    process = subprocess.Popen([
        str(executable), '--ros-args', '-p', 'timer_period_s:=0.05',
        '-p', 'nbv_goal_timeout_s:=0.2', '-p', 'nbv_feedback_timeout_s:=2.0',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        wait(condition=lambda: 'WaitingForPointCloud' in states)
        until = time.monotonic() + .7
        wait(condition=lambda: time.monotonic() >= until)
        assert 'Failed' not in states and 'MissionComplete' not in states
        assert True not in stops and server._goal_handle is not None
    finally:
        process.terminate()
        process.wait(timeout=5.)


def test_cpp_client_still_times_out_actual_computation(transport):
    server, _, client_node, wait = transport
    executable = ROOT / 'install/motion_control/lib/motion_control/point_iterator'
    if not executable.exists():
        pytest.skip('Build motion_control to run the C++ action integration test')
    server.outcome = 'waiting'  # Computing feedback continues, but no result.
    states = []
    client_node.create_subscription(String, '/planner/state', lambda m: states.append(m.data), 10)
    process = subprocess.Popen([
        str(executable), '--ros-args', '-p', 'timer_period_s:=0.05',
        '-p', 'nbv_goal_timeout_s:=0.2', '-p', 'nbv_feedback_timeout_s:=1.0',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        wait(condition=lambda: 'Failed' in states)
        wait(condition=lambda: server._goal_handle is None)
    finally:
        process.terminate()
        process.wait(timeout=5.)
