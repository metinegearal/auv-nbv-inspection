"""DLQR waypoint controller using measured state directly (no observer)."""

import math

from benchmark_evaluator.domain.response import ResponseMonitor
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Bool, Float32MultiArray

from .dlqr import FeedbackSystem, vector, waypoint_velocity
from .thrusters import force_to_thrust, MAX_THRUST, thrust_to_force


class WaypointControlNode(Node):
    """
    Control a level vehicle at the configured ZOH period.

    Position comes directly from odometry. By default body DVL velocity is
    rotated using magnetometer yaw; odometry twist is explicitly opt-in because
    this repository's LocationSensor bridge commonly leaves it zero.
    """

    def __init__(self, **kwargs):
        super().__init__('waypoint_control_node', **kwargs)
        fixed = {
            'update_period': 1.0 / 30.0, 'mass': 31.02, 'damping': 3.0,
            'perfect': True, 'state_source': 'dvl', 'sensor_timeout': 0.5,
            'state_weights': [80., 80., 80., 16., 16., 16.],
            'velocity_weights': [10., 10., 10.], 'force_weights': [0.1, 0.1, 0.1],
            'initial_waypoint_x': 10., 'initial_waypoint_y': 10.,
            'initial_waypoint_z': 5., 'initial_waypoint_yaw': 0.,
        }
        for name, value in fixed.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        for name, value in {
            'control_mode': 'state', 'target_velocity': [0., 0., 0.],
            'target_state': [0., 0., 0., *[self.value(f'initial_waypoint_{a}') for a in 'xyz']],
            'stopping_distance': 0.25, 'log_period': 1.,
            'waypoint_cruise_speed': 0.0, 'waypoint_braking_acceleration': 0.8,
            'kp_yaw': 0.6, 'max_yaw_command': 10.,
        }.items():
            self.declare_parameter(name, value)
        self.validate_startup()
        self.system = FeedbackSystem(
            self.value('update_period'), mass=self.value('mass'),
            damping=self.value('damping'), perfect=self.value('perfect'),
            state_weights=self.value('state_weights'),
            velocity_weights=self.value('velocity_weights'),
            force_weights=self.value('force_weights'))
        self.state = np.zeros(6)
        self.rotation = np.eye(3)
        self.yaw = 0.
        self.target_yaw = self.value('initial_waypoint_yaw')
        self.received = {}
        self.target = vector(self.value('target_state'), 6, 'target_state')
        initial = np.r_[np.zeros(3), [self.value(f'initial_waypoint_{a}') for a in 'xyz']]
        self.waypoint_active = np.array_equal(self.target, initial)
        self.next_waypoint = None
        self.target_epoch = None
        self.monitor = None
        self.response_epoch = None
        self.last_log = None
        self.last_tick = None
        self.finished = False
        self.reported = False
        self.was_ready = False
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.add_post_set_parameters_callback(self.parameters_changed)
        self.command_pub = self.create_publisher(Float32MultiArray, '/holo/cmd', 10)
        self.report_pub = self.create_publisher(Float32MultiArray, '/motion/report', 10)
        self.force_pub = self.create_publisher(Vector3Stamped, '/motion/commanded_force', 10)
        self.applied_pub = self.create_publisher(Vector3Stamped, '/motion/applied_force', 10)
        self.create_subscription(Odometry, 'holocean/odom', self.odom_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(TwistStamped, 'holocean/dvl', self.dvl_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(MagneticField, 'holocean/mag', self.mag_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/trajectory/waypoint',
                                 self.waypoint_callback, 10)
        self.create_subscription(Bool, '/movement/finished_execution',
                                 self.finished_callback, 10)
        self.timer = self.create_timer(self.system.dt, self.tick)
        for axis, values in self.system.damping_metrics().items():
            self.get_logger().info(
                f'Nominal DLQR {axis}: s={values["poles"]}; '
                f'zeta={values["zeta"]:.4f}; wn={values["wn"]:.4f} rad/s; '
                f'dominant tau={values["dominant_tau"]:.4f}s')
        self.get_logger().info('DLQR waiting for fresh direct state; motors held at zero')

    def value(self, name):
        return self.get_parameter(name).value

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def validate_startup(self):
        if self.value('state_source') not in ('dvl', 'odom'):
            raise ValueError('state_source must be dvl or odom')
        for name in ('sensor_timeout', 'update_period'):
            if not math.isfinite(self.value(name)) or self.value(name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.value('sensor_timeout') <= self.value('update_period'):
            raise ValueError('sensor_timeout must exceed update_period')
        for name in ('initial_waypoint_x', 'initial_waypoint_y',
                     'initial_waypoint_z', 'initial_waypoint_yaw'):
            if not math.isfinite(self.value(name)):
                raise ValueError(f'{name} must be finite')
        result = self.validate_parameters([
            self.get_parameter(name) for name in (
                'control_mode', 'target_state', 'target_velocity', 'stopping_distance',
                'log_period', 'kp_yaw', 'max_yaw_command',
                'waypoint_cruise_speed', 'waypoint_braking_acceleration')])
        if not result.successful:
            raise ValueError(result.reason)

    def validate_parameters(self, parameters):
        try:
            for parameter in parameters:
                name, value = parameter.name, parameter.value
                if name in ('target_state', 'target_velocity'):
                    vector(value, 6 if name == 'target_state' else 3, name)
                elif name == 'control_mode' and value not in ('state', 'velocity'):
                    raise ValueError('control_mode must be state or velocity')
                elif name == 'waypoint_cruise_speed':
                    if not math.isfinite(value) or value < 0:
                        raise ValueError('waypoint_cruise_speed must be finite and nonnegative')
                elif name in ('stopping_distance', 'log_period', 'kp_yaw', 'max_yaw_command',
                              'waypoint_braking_acceleration'):
                    if not math.isfinite(value) or value <= 0:
                        raise ValueError(f'{name} must be finite and positive')
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def parameters_changed(self, parameters):
        for parameter in parameters:
            if parameter.name == 'target_state':
                self.waypoint_active = False
                self.next_waypoint = None
                self.set_target(parameter.value)
            elif parameter.name == 'control_mode':
                self.reset_response()

    def reset_response(self):
        self.monitor = None
        self.response_epoch = None
        self.target_epoch = None
        self.reported = False

    def set_target(self, target):
        target = vector(target, 6, 'target')
        if not np.array_equal(self.target, target):
            self.target = target
            self.reset_response()

    def waypoint_callback(self, message):
        if len(message.data) == 0:
            if self.waypoint_active:
                self.set_target(np.r_[np.zeros(3), self.state[3:]])
                self.waypoint_active = False
                self.next_waypoint = None
            return
        if len(message.data) < 3:
            self.get_logger().warning('Ignoring waypoint with fewer than three values')
            return
        try:
            if len(message.data) not in (3, 4) and len(message.data) % 4:
                raise ValueError('waypoint queue must contain complete [x,y,z,yaw] points')
            position = vector(message.data[:3], 3, 'waypoint')
            following = vector(message.data[4:7], 3, 'next waypoint') if len(message.data) >= 8 else None
            yaw = float(message.data[3]) if len(message.data) >= 4 else self.yaw
            if not math.isfinite(yaw):
                raise ValueError('waypoint yaw must be finite')
            self.set_target(np.r_[np.zeros(3), position])
            self.waypoint_active = True
            self.next_waypoint = following
            self.target_yaw = yaw
        except ValueError as error:
            self.get_logger().warning(str(error))

    def finished_callback(self, message):
        self.finished = message.data
        self.reset_response()
        if self.finished:
            self.publish_command(np.zeros(8))

    def odom_callback(self, message):
        p = message.pose.pose.position
        try:
            position = vector([p.x, p.y, p.z], 3, 'position')
            if self.value('state_source') == 'odom':
                q = message.pose.pose.orientation
                quat = vector([q.x, q.y, q.z, q.w], 4, 'orientation')
                norm = np.linalg.norm(quat)
                if norm < 1e-6:
                    raise ValueError('odometry must include a valid orientation')
                x, y, z, w = quat / norm
                yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
                v = message.twist.twist.linear
                velocity = vector([v.x, v.y, v.z], 3, 'velocity')
                self.set_yaw(yaw)
                self.body_velocity = velocity
            self.state[3:] = position
            self.received['odom'] = self.now_seconds()
        except ValueError as error:
            self.get_logger().warning(str(error))

    def dvl_callback(self, message):
        if self.value('state_source') != 'dvl':
            return
        v = message.twist.linear
        try:
            self.body_velocity = vector([v.x, v.y, v.z], 3, 'DVL velocity')
            self.received['dvl'] = self.now_seconds()
        except ValueError:
            return

    def set_yaw(self, yaw):
        self.yaw = yaw
        c, s = math.cos(yaw), math.sin(yaw)
        self.rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])

    def mag_callback(self, message):
        if self.value('state_source') != 'dvl':
            return
        m = message.magnetic_field
        if np.isfinite([m.x, m.y, m.z]).all() and math.hypot(m.x, m.y) > 1e-6:
            self.set_yaw(math.atan2(-m.y, m.x))
            self.received['mag'] = self.now_seconds()

    def tick(self):
        now = self.now_seconds()
        clock_reset = self.last_tick is not None and now < self.last_tick
        self.last_tick = now
        if clock_reset:
            self.received.clear()
            self.last_log = None
        required = ('odom', 'dvl', 'mag') if self.value('state_source') == 'dvl' else ('odom',)
        ready = all(key in self.received and
                    0 <= now - self.received[key] <= self.value('sensor_timeout')
                    for key in required)
        if self.finished or not ready:
            self.publish_command(np.zeros(8))
            if self.was_ready:
                self.get_logger().warning('Control interrupted; motors held at zero')
            self.was_ready = False
            self.reset_response()
            return
        self.was_ready = True
        self.state[:3] = self.rotation @ self.body_velocity
        mode = self.value('control_mode')
        if self.target_epoch is None:
            self.target_epoch = now
        if mode == 'state':
            reference = self.target.copy()
            reference[3:] += reference[:3] * (now - self.target_epoch)
            if np.all(reference[:3] == 0.):
                if self.monitor is None:
                    self.monitor = ResponseMonitor(self.state[3:], reference[3:])
                    self.response_epoch = now
                self.monitor.update(now - self.response_epoch, self.state[3:])
                if (not self.reported and
                        np.linalg.norm(reference[3:] - self.state[3:]) <=
                        self.value('stopping_distance')):
                    self.report_pub.publish(Float32MultiArray(data=reference[3:].tolist()))
                    self.reported = True
        else:
            reference = vector(self.value('target_velocity'), 3, 'target_velocity')
        cruise_reference = None
        if mode == 'state' and self.waypoint_active and self.value('waypoint_cruise_speed') > 0:
            desired = waypoint_velocity(
                self.state[3:], self.target[3:], self.next_waypoint,
                speed=self.value('waypoint_cruise_speed'),
                braking_acceleration=self.value('waypoint_braking_acceleration'),
                stopping_distance=self.value('stopping_distance'))
            # At the final point/corner, the existing position controller brakes
            # and holds. Otherwise use DLQR's velocity feedback and feedforward.
            if np.linalg.norm(desired) > 1e-9:
                cruise_reference = desired
        force = self.system.control_force(
            self.state, reference if cruise_reference is None else cruise_reference)
        yaw_error = math.atan2(math.sin(self.target_yaw - self.yaw),
                               math.cos(self.target_yaw - self.yaw))
        torque = np.clip(self.value('kp_yaw') * yaw_error,
                         -self.value('max_yaw_command'), self.value('max_yaw_command'))
        command = force_to_thrust(
            np.r_[self.rotation.T @ force, 0., 0., torque], perfect=self.value('perfect'))
        peak = float(np.max(np.abs(command)))
        scale = min(1., MAX_THRUST / peak) if peak > 0 else 1.
        command *= scale
        self.publish_command(command)
        applied = self.rotation @ thrust_to_force(command, perfect=self.value('perfect'))[:3]
        self.publish_force(self.force_pub, force)
        self.publish_force(self.applied_pub, applied)
        if self.last_log is None or now - self.last_log >= self.value('log_period'):
            self.last_log = now
            if self.monitor is not None:
                self.get_logger().info(self.monitor.summary())
            self.get_logger().info(f'Requested motor peak={peak:.3f}N; limit scale={scale:.3f}')
            if self.waypoint_active:
                requested_speed = 0. if cruise_reference is None else np.linalg.norm(cruise_reference)
                self.get_logger().info(
                    f'Waypoint speed: measured={np.linalg.norm(self.state[:3]):.3f}m/s; '
                    f'reference={requested_speed:.3f}m/s; '
                    f'distance={np.linalg.norm(self.target[3:] - self.state[3:]):.3f}m')

    def publish_force(self, publisher, force):
        message = Vector3Stamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'odom'
        message.vector.x, message.vector.y, message.vector.z = map(float, force)
        publisher.publish(message)

    def publish_command(self, command):
        command = vector(command, 8, 'motor command')
        self.command_pub.publish(Float32MultiArray(data=command.tolist()))


def main(args=None):
    """Run the controller and stop thrusters on a normal shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.publish_command(np.zeros(8))
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
