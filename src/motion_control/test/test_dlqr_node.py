"""Exercise ROS callbacks and output of the deterministic controller."""

from unittest.mock import Mock

from geometry_msgs.msg import TwistStamped
from motion_control.thrusters import MAX_THRUST, thrust_to_force
from motion_control.waypoint_node import WaypointControlNode
from nav_msgs.msg import Odometry
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Bool, Float32MultiArray


@pytest.fixture
def node():
    rclpy.init()
    controller = WaypointControlNode(parameter_overrides=[
        Parameter('target_state', value=[0.] * 6),
        Parameter('perfect', value=False),
    ])
    controller.timer.cancel()
    controller.now_seconds = Mock(return_value=10.)
    controller.command_pub = Mock()
    controller.report_pub = Mock()
    controller.force_pub = Mock()
    controller.applied_pub = Mock()
    yield controller
    controller.destroy_node()
    rclpy.shutdown()


def feed_state(node, position=(0., 0., 0.), velocity=(0., 0., 0.), yaw=0.):
    odom = Odometry()
    odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = position
    node.odom_callback(odom)
    dvl = TwistStamped()
    dvl.twist.linear.x, dvl.twist.linear.y, dvl.twist.linear.z = velocity
    node.dvl_callback(dvl)
    mag = MagneticField()
    mag.magnetic_field.x = float(np.cos(yaw))
    mag.magnetic_field.y = float(-np.sin(yaw))
    node.mag_callback(mag)


def command(node):
    return np.array(node.command_pub.publish.call_args.args[0].data)


def test_wait_hover_stale_and_resume(node):
    node.tick()
    np.testing.assert_array_equal(command(node), np.zeros(8))
    feed_state(node)
    node.tick()
    np.testing.assert_allclose(thrust_to_force(command(node), perfect=False)[:3],
                               -node.system.buoyancy, atol=1e-12)
    assert node.monitor is not None
    node.now_seconds.return_value = 11.
    node.tick()
    np.testing.assert_array_equal(command(node), np.zeros(8))
    assert node.monitor is None
    feed_state(node)
    node.tick()
    assert node.response_epoch == 11.


def test_waypoint_world_body_rotation_saturation_and_report(node):
    feed_state(node, yaw=np.pi / 2)
    node.waypoint_callback(Float32MultiArray(data=[100., 0., 0., float(np.pi / 2)]))
    node.tick()
    assert np.max(np.abs(command(node))) == pytest.approx(MAX_THRUST)
    requested = node.force_pub.publish.call_args.args[0].vector
    applied = node.rotation @ thrust_to_force(command(node), perfect=False)[:3]
    requested = np.array([requested.x, requested.y, requested.z])
    assert applied[0] > 0
    assert applied[0] < requested[0]
    np.testing.assert_allclose(applied / np.linalg.norm(applied),
                               requested / np.linalg.norm(requested), atol=1e-12)
    monitor = node.monitor
    node.waypoint_callback(Float32MultiArray(data=[100., 0., 0., float(np.pi / 2)]))
    assert node.monitor is monitor
    node.now_seconds.return_value = 10.1
    feed_state(node, position=(100., 0., 0.))
    node.tick()
    node.tick()
    assert node.report_pub.publish.call_count == 1
    assert list(node.report_pub.publish.call_args.args[0].data) == [100., 0., 0.]


def test_parameter_validation_moving_target_and_velocity_mode(node):
    result = node.set_parameters_atomically([Parameter('target_state', value=[0., float('nan')])])
    assert not result.successful
    np.testing.assert_array_equal(node.target, np.zeros(6))
    result = node.set_parameters_atomically([
        Parameter('target_state', value=[1., 0., 0., 0., 0., 0.])])
    assert result.successful
    feed_state(node, velocity=(1., 0., 0.))
    node.tick()
    node.now_seconds.return_value = 10.1
    feed_state(node, position=(0.1, 0., 0.), velocity=(1., 0., 0.))
    node.tick()
    assert node.monitor is None
    np.testing.assert_allclose(thrust_to_force(command(node), perfect=False)[:3],
                               [3., 0., -node.system.buoyancy_force], atol=1e-10)
    result = node.set_parameters_atomically([
        Parameter('control_mode', value='velocity'),
        Parameter('target_velocity', value=[1., 0., 0.])])
    assert result.successful
    feed_state(node, position=(100., 5., -10.), velocity=(1., 0., 0.))
    node.tick()
    np.testing.assert_allclose(thrust_to_force(command(node), perfect=False)[:3],
                               [3., 0., -node.system.buoyancy_force], atol=1e-10)


def test_finish_invalid_samples_and_clock_reset_stop_motors(node):
    feed_state(node)
    node.tick()
    node.finished_callback(Bool(data=True))
    np.testing.assert_array_equal(command(node), np.zeros(8))
    node.finished_callback(Bool(data=False))
    node.now_seconds.return_value = 9.
    node.tick()
    np.testing.assert_array_equal(command(node), np.zeros(8))
    assert not node.received
    invalid = TwistStamped()
    invalid.twist.linear.x = float('nan')
    node.dvl_callback(invalid)
    assert 'dvl' not in node.received


def test_odom_source_requires_orientation_and_rotates_body_twist(node):
    # The alternate source is fixed at startup, so construct a separate node.
    other = WaypointControlNode(parameter_overrides=[Parameter('state_source', value='odom')])
    try:
        other.timer.cancel()
        message = Odometry()
        message.pose.pose.orientation.w = 0.
        other.odom_callback(message)
        assert 'odom' not in other.received
        message.pose.pose.orientation.z = float(np.sqrt(0.5))
        message.pose.pose.orientation.w = float(np.sqrt(0.5))
        message.twist.twist.linear.x = 1.
        other.odom_callback(message)
        np.testing.assert_allclose(other.rotation @ other.body_velocity, [0., 1., 0.], atol=1e-12)
    finally:
        other.destroy_node()
