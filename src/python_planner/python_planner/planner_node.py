import math
import json
import time
from threading import Lock
from pathlib import Path

import numpy as np
import pyoctomap
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, MagneticField
from std_msgs.msg import Bool, Float32MultiArray
from collections import deque

from benchmark_evaluator.midrun_logger import MidrunLogger

from .get_NBV import DEFAULT_NBV_PARAMETERS, calculate_nbv_waypoints
from .route_planning import (
    generate_smooth_trajectory,
    inflate_occupied,
    path_is_collision_free,
    plan_multi_waypoint_route,
    resample_trajectory_by_distance,
    reconnect_trajectory,
)
from .trajectory_optimization import (
    continuity_cost, estimate_velocity, optimize_trajectory,
)


def body_rotation(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def vehicle_viewpoint(camera_position, target, camera_offset):
    """Place the vehicle so its mounted camera reaches the desired viewpoint."""
    yaw = math.atan2(target[1] - camera_position[1], target[0] - camera_position[0])
    for _ in range(12):
        vehicle = camera_position - body_rotation(0., 0., yaw) @ camera_offset
        next_yaw = math.atan2(target[1] - vehicle[1], target[0] - vehicle[0])
        if abs(math.atan2(math.sin(next_yaw - yaw), math.cos(next_yaw - yaw))) < 1e-8:
            return vehicle
        yaw = next_yaw
    return camera_position - body_rotation(0., 0., yaw) @ camera_offset


class NBVUtilityNode(Node):
    """Build a depth map online and publish NBV viewpoints."""

    def __init__(self):
        super().__init__('nbv_utility_node')

        self.resolution = self.declare_parameter('voxel_resolution', 0.1).value
        self.max_depth = self.declare_parameter('max_depth', 3.0).value
        self.min_depth = self.declare_parameter('min_depth', 0.5).value
        self.z_min_target = self.declare_parameter(
            'z_min_target', -3000.0).value
        self.depth_stride = self.declare_parameter('depth_stride', 8).value
        self.depth_scale = self.declare_parameter('depth_scale', 1.0).value
        self.frame_period = self.declare_parameter('planning_period', 10).value
        self.min_utility = self.declare_parameter(
            'min_utility_threshold', 0.005).value
        self.collision_radius = self.declare_parameter(
            'collision_radius', 0.6).value
        self.collision_tracking_margin = self.declare_parameter(
            'collision_tracking_margin', 0.4).value
        self.blacklist_cooldown_s = self.declare_parameter(
            'route_blacklist_cooldown_s', 30.0).value
        self.trajectory_points = self.declare_parameter(
            'trajectory_points', 300).value
        self.waypoint_spacing = self.declare_parameter(
            'waypoint_spacing', 0.5).value
        self.map_path = self.declare_parameter(
            'octomap_path', 'results/final_map_ros2.bt').value
        self.visit_radius = self.declare_parameter('visit_radius', 0.25).value
        self.results_dir = self.declare_parameter(
            'results_dir', 'results').value
        self.nbv_filter_logging = self.declare_parameter(
            'nbv_filter_logging', True).value
        self._nbv_empty_reason = None
        self._plan_failure_reason = None
        self.use_incremental_map_cache = self.declare_parameter(
            'use_incremental_map_cache', True).value
        self.nbv_parameters = {
            name: self.declare_parameter(
                f'nbv_{name}', default).value
            for name, default in DEFAULT_NBV_PARAMETERS.items()
            if name != 'z_min_target'
        }
        # The baseline NBV function filtered at -31.5 independently of mapping.
        self.nbv_parameters['z_min_target'] = self.declare_parameter(
            'nbv_z_min_target', DEFAULT_NBV_PARAMETERS['z_min_target']).value
        self.astar_max_steps = self.declare_parameter(
            'astar_max_steps', 50000).value
        self.spline_smoothing = self.declare_parameter(
            'spline_smoothing', 2.0).value
        self.collision_check_spacing_factor = self.declare_parameter(
            'collision_check_spacing_factor', 0.5).value
        self.arrival_target_radius = self.declare_parameter(
            'arrival_target_radius', 1.5).value
        self.replan_on_target_arrival = self.declare_parameter(
            'replan_on_target_arrival', False).value
        self.optimize_motion = self.declare_parameter(
            'trajectory_optimization_enabled', False).value
        self.velocity_history_window = self.declare_parameter(
            'velocity_history_window', 1.0).value
        self.trajectory_history_window = self.declare_parameter(
            'trajectory_history_window', 5.0).value
        self.motion_cost_parameters = {
            name: self.declare_parameter(f'trajectory_{name}', default).value
            for name, default in {
                'continuity_weight': 8.0, 'reversal_weight': 4.0,
                'history_weight': 2.0, 'history_radius': 0.5,
            }.items()
        }
        self.optimizer_max_deviation = self.declare_parameter(
            'trajectory_max_deviation', 0.5).value
        self.optimizer_max_iterations = self.declare_parameter(
            'trajectory_optimizer_max_iterations', 40).value

        self.state_lock = Lock()
        self.state_callbacks = MutuallyExclusiveCallbackGroup()
        self.bridge = CvBridge()
        self.position = np.zeros(3, dtype=np.float64)
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        # --- LATCHED VARIABLES ---
        self.latched_position = np.zeros(3, dtype=np.float64)
        self.latched_yaw = 0.0
        self.latched_pitch = 0.0
        self.latched_roll = 0.0
        self.odom_history = deque(maxlen=3000)

        self.odom_received = False
        self.rgb_received = False
        self.depth_frame = None
        self.depth_sequence = 0
        self.processed_depth_sequence = 0
        self.iteration = 0
        self.started_mapping = False
        self.finished_mapping = False
        self.octree = pyoctomap.ColorOcTree(self.resolution)
        self.active_trajectory = np.empty((0, 4), dtype=np.float64)
        self.camera_offset = np.array([0.35, 0.1125 + 0.275, -0.5])
        self.force_replan = False
        self.visited_targets = []
        self.selected_targets = []
        self.occupied_voxels = set()
        self.pending_occupied_voxels = set()
        self.collision_voxels = set()
        self.telemetry = MidrunLogger(self.results_dir, prefix='planner')
        self.failed_plan_count = 0
        self.observation_history = []
        self.blacklisted_targets = []
        self.blacklist_expiries = {}

        qos = rclpy.qos.QoSProfile(depth=10)
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            Image, 'holocean/depth/image_raw', self.depth_callback, qos)
        self.create_subscription(
            Image, 'holocean/cameraDown/image_raw', self.rgb_callback, qos)
        state_qos = rclpy.qos.QoSProfile(depth=1)
        state_qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            Odometry, 'holocean/odom', self.odom_callback, state_qos,
            callback_group=self.state_callbacks)
        self.create_subscription(
            MagneticField, 'holocean/mag', self.mag_callback, state_qos,
            callback_group=self.state_callbacks)
        self.create_subscription(
            Bool, '/planner/request_nbv', self.request_nbv_callback, 1)

        self.waypoint_pub = self.create_publisher(
            Float32MultiArray, 'object/areaWaypoints', 10)
        self.finished_pub = self.create_publisher(
            Bool, '/movement/finished_execution', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.evaluation_file = open(
            Path(self.results_dir) / 'evaluation_parameters.txt', 'a')

        self.get_logger().info(
            f'NBV utility node started: active NBV filter is Z > '
            f'{self.nbv_parameters["z_min_target"]}; '
            f'z_min_target={self.z_min_target} is not used by this filter')
        if self.nbv_filter_logging:
            self.get_logger().info(f'NBV filter diagnostics: {self.telemetry.filter_path}')

    def request_nbv_callback(self, msg):
        """Triggered instantly when C++ PathPlanner runs out of waypoints."""
        if msg.data:
            self.get_logger().info('Received manual NBV request from PathPlanner. Forcing immediate replan!')
            self.force_replan = True

    def _motion_snapshot(self):
        """Copy a consistent state without locking the expensive planning work."""
        with self.state_lock:
            return self.position.copy(), self.yaw, list(self.odom_history)

    def odom_callback(self, msg):
        """Keep odometry updating while the default group runs planning."""
        with self.state_lock:
            self._update_odometry(msg)

    def _update_odometry(self, msg):
        """Update the vehicle pose with state_lock held."""
        position = msg.pose.pose.position
        if abs(position.x) < 0.01 and abs(position.y) < 0.01:
            self._log_nbv_filter('odometry_origin_guard', 1, 0,
                                 {'position': [position.x, position.y, position.z],
                                  'xy_threshold': 0.01})
            return

        self.position = np.array(
            [position.x, position.y, position.z], dtype=np.float64)
        orientation = msg.pose.pose.orientation
        sin_roll = 2.0 * (orientation.w * orientation.x
                          + orientation.y * orientation.z)
        cos_roll = 1.0 - 2.0 * (orientation.x ** 2 + orientation.y ** 2)
        self.roll = math.atan2(sin_roll, cos_roll)
        sin_pitch = 2.0 * (orientation.w * orientation.y
                           - orientation.z * orientation.x)
        self.pitch = math.asin(np.clip(sin_pitch, -1.0, 1.0))
        # sin_yaw = 2.0 * (orientation.w * orientation.z
        #                  + orientation.x * orientation.y)
        # cos_yaw = 1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2)
        # self.yaw = math.atan2(sin_yaw, cos_yaw)
        self.odom_received = True
        
        # NEW: Save this exact state with its timestamp into the history buffer
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.odom_history and msg_time < self.odom_history[-1][0]:
            self.odom_history.clear()  # Simulator clock reset: discard old motion.
        if self.odom_history and msg_time == self.odom_history[-1][0]:
            return
        self.odom_history.append((
            msg_time, 
            np.copy(self.position), 
            self.roll, 
            self.pitch, 
            self.yaw
        ))

    def mag_callback(self, msg):
        """Use the magnetometer heading when it is available."""
        with self.state_lock:
            self.yaw = math.atan2(
                msg.magnetic_field.x, msg.magnetic_field.y) - math.pi / 2.0
            if self.yaw < -math.pi:
                self.yaw += 2.0 * math.pi

    def rgb_callback(self, msg):
        """Record that the camera stream is available."""
        del msg
        self.rgb_received = True

    def depth_callback(self, msg):
        """Find the historical odometry that exactly matches this depth frame."""
        self.depth_frame = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')
        self.depth_sequence += 1
        
        _, _, history = self._motion_snapshot()
        if len(history) == 0:
            return
            
        # Get the time the original image was taken (passed through by obstacleAvoidance)
        depth_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Search the history buffer for the odometry closest to the depth timestamp
        closest_odom = min(history, key=lambda x: abs(x[0] - depth_time))
        
        # If the closest match is within 0.1 seconds, use it! 
        if abs(closest_odom[0] - depth_time) < 0.1:
            self.latched_position = np.copy(closest_odom[1])
            self.latched_roll = closest_odom[2]
            self.latched_pitch = closest_odom[3]
            self.latched_yaw = closest_odom[4]
            self.telemetry.record_depth_pose(
                self.depth_sequence,
                depth_time,
                self.latched_position,
                self.latched_roll,
                self.latched_pitch,
                self.latched_yaw)
        else:
            self.get_logger().warning("Depth map timestamp is too far! Closest odometry is %.3f seconds away. Using the most recent odometry instead." % (abs(closest_odom[0] - depth_time)))
            latest = history[-1]
            self.latched_position = latest[1].copy()
            self.latched_roll, self.latched_pitch, self.latched_yaw = latest[2:]

    def _depth_points(self, depth_frame):
        """Project a depth image into the vehicle world frame."""
        depth = np.asarray(depth_frame)
        if depth.ndim != 2:
            self._log_nbv_filter('depth_image_shape', int(depth.size), 0,
                                 {'shape': list(depth.shape), 'required_dimensions': 2})
            return np.empty((0, 3), dtype=np.float64)

        scale = 0.001 if depth.dtype == np.uint16 else self.depth_scale
        depth = depth.astype(np.float64) * scale
        height, width = depth.shape
        stride = max(1, int(self.depth_stride))
        rows, cols = np.mgrid[0:height:stride, 0:width:stride]
        distances = depth[rows, cols]
        valid = np.isfinite(distances)
        valid &= (distances > self.min_depth)
        valid &= (distances < self.max_depth)
        finite_distances = distances[np.isfinite(distances)]
        self._log_nbv_filter('depth_range', int(distances.size), int(np.count_nonzero(valid)),
                             {'min_depth': self.min_depth, 'max_depth': self.max_depth,
                              'comparison': 'min_depth < distance < max_depth',
                              'nonfinite': int(np.count_nonzero(~np.isfinite(distances))),
                              'at_or_below_min': int(np.count_nonzero(
                                  np.isfinite(distances) & (distances <= self.min_depth))),
                              'at_or_above_max': int(np.count_nonzero(
                                  np.isfinite(distances) & (distances >= self.max_depth))),
                              'distance_min': float(finite_distances.min()) if finite_distances.size else None,
                              'distance_max': float(finite_distances.max()) if finite_distances.size else None,
                              'depth_scale': scale, 'stride': stride,
                              'horizontal_fov_deg': 90.0, 'camera_pitch_deg': 45.0,
                              'camera_offset': self.camera_offset})
        if not np.any(valid):
            return np.empty((0, 3), dtype=np.float64)

        rows = rows[valid].astype(np.float64)
        cols = cols[valid].astype(np.float64)
        distances = distances[valid]
        focal_length = (width / 2.0) / math.tan(math.radians(90.0) / 2.0)
        center_x, center_y = width / 2.0, height / 2.0
        camera_x = (cols - center_x) * distances / focal_length
        camera_y = (rows - center_y) * distances / focal_length

        pitch = math.radians(45.0)
        local_x = distances * math.cos(pitch) - camera_y * math.sin(pitch)
        local_y = -camera_x
        local_z = -distances * math.sin(pitch) - camera_y * math.cos(pitch)
        local_points = np.column_stack((local_x, local_y, local_z))

        rotation = body_rotation(self.latched_roll, self.latched_pitch, self.latched_yaw)
        self._depth_sensor_origin = self.latched_position + rotation @ self.camera_offset
        return local_points @ rotation.T + self._depth_sensor_origin

    def _accumulate_depth(self):
        points = self._depth_points(self.depth_frame)
        if len(points) == 0:
            # Warn if the camera only sees empty water outside max_depth
            self.get_logger().warning('Depth projection yielded 0 valid points (all out of range or invalid).', throttle_duration_sec=2.0)
            return

        before_point_cap = len(points)
        if len(points) > 4000:
            # Randomly sample exactly 4000 points to preserve the pipe's overall shape
            indices = np.random.choice(len(points), 4000, replace=False)
            points = points[indices]
        self._log_nbv_filter('depth_point_cap', before_point_cap,
                             len(points), {'limit': 4000})

        if self.use_incremental_map_cache:
            voxel_indices = np.floor(
                points / self.resolution).astype(np.int32)
            new_voxels = set(map(tuple, voxel_indices.tolist()))
            new_voxels.difference_update(self.occupied_voxels)
            self.occupied_voxels.update(new_voxels)
            self.pending_occupied_voxels.update(new_voxels)
            
        sensor_origin = self._depth_sensor_origin
        self.octree.insertPointCloud(
            points.astype(np.float64),
            sensor_origin=sensor_origin.astype(np.float64),
            max_range=float(self.max_depth),
        )
        self.get_logger().info(f'Inserted {len(points)} points into OctoTree.', throttle_duration_sec=1.0)

        save_obs = True
        if self.observation_history:
            last_pos, last_yaw = self.observation_history[-1]
            dist = np.linalg.norm(self.latched_position - last_pos)
            
            # Normalize yaw difference
            yaw_diff = abs(math.atan2(
                math.sin(self.latched_yaw - last_yaw), 
                math.cos(self.latched_yaw - last_yaw)
            ))
            
            # Only save if we moved 0.5m or turned 15 degrees to save memory
            if dist < 0.5 and yaw_diff < math.radians(15):
                save_obs = False
                
        if save_obs:
            self.observation_history.append((np.copy(self.latched_position), self.latched_yaw))
    

    def _occupied_points(self):
        """Extract occupied metric points from the persistent OctoMap."""
        if self.use_incremental_map_cache:
            if not self.occupied_voxels:
                return np.empty((0, 3), dtype=np.float64)
            return (
                (np.asarray(tuple(self.occupied_voxels), dtype=np.float64) + 0.5)
                * self.resolution)

        extracted = self.octree.extractPointCloud()
        points = np.asarray(extracted[0], dtype=np.float64)
        if points.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        return points.reshape((-1, 3))

    def _publish_trajectory(self, trajectory, occupied):
        """Validate the vehicle coordinates reconstructed from the wire message."""
        if len(trajectory) == 0:
            return False

        wire = np.asarray(trajectory, dtype=np.float64).copy()
        # The existing iterator subtracts this offset; compensate exactly once.
        wire[:, :3] += self.camera_offset
        wire = wire.astype(np.float32)
        executed = wire.astype(np.float64)
        executed[:, :3] -= self.camera_offset
        if not path_is_collision_free(executed[:, :3], occupied, self.resolution):
            self._log_nbv_filter('vehicle_wire_validation', len(trajectory), 0,
                                 {'reason': 'executed_vehicle_route_collision'})
            return False
        message = Float32MultiArray()
        message.data = wire.flatten().tolist()
        self.waypoint_pub.publish(message)
        self.active_trajectory = executed
        self.started_mapping = True
        self.get_logger().info(
            f'Published {len(self.active_trajectory)} trajectory points')
        return True

    def _plan_and_publish(self, candidates, occupied):
        """Run A*, smooth its route, and publish only a safe trajectory."""
        self._plan_failure_reason = None
        self._failed_route_target = None
        valid_candidates = [
            candidate for candidate in candidates
            if candidate['utility'] >= self.min_utility
        ]
        for candidate in candidates:
            self._log_nbv_filter('candidate_minimum_utility', 1,
                                 int(candidate['utility'] >= self.min_utility),
                                 {'cluster_id': candidate.get('cluster_id'),
                                  'target': candidate['target'],
                                  'position': candidate['position'],
                                  'utility': candidate['utility'],
                                  'threshold': self.min_utility})
        self._log_nbv_filter('minimum_utility', len(candidates), len(valid_candidates),
                             {'threshold': self.min_utility})
        if not valid_candidates:
            self._plan_failure_reason = ('minimum_utility_removed_all' if candidates
                                         else 'no_candidates_to_plan')
            return False
        valid_candidates.sort(key=lambda candidate: candidate['utility'], reverse=True)

        route_candidates = [
            (vehicle_viewpoint(np.asarray(candidate['position'], dtype=np.float64),
                               np.asarray(candidate['target'], dtype=np.float64), self.camera_offset),
             np.asarray(candidate['target'], dtype=np.float64))
            for candidate in valid_candidates
        ]
        route_started = time.perf_counter()
        position, yaw, history = self._motion_snapshot()
        velocity = estimate_velocity(
            history, float(self.velocity_history_window))
        latest_time = history[-1][0] if history else 0.0
        recent_positions = [sample[1]
                            for sample in history
                            if 0.0 <= latest_time - sample[0]
                            <= float(self.trajectory_history_window)]
        if len(recent_positions) > 64:
            indices = np.linspace(0, len(recent_positions) - 1, 64).astype(int)
            recent_positions = [recent_positions[index] for index in indices]
        route, targets = plan_multi_waypoint_route(
            position,
            route_candidates,
            occupied,
            self.resolution,
            astar_max_steps=int(self.astar_max_steps),
            current_velocity=velocity if self.optimize_motion else None,
            reversal_weight=(float(self.motion_cost_parameters['reversal_weight'])
                             if self.optimize_motion else 0.0),
        )
        self.telemetry.record_stage(
            'route_search', (time.perf_counter() - route_started) * 1000.0,
            iteration=self.iteration)
        route_safe = len(route) > 0 and path_is_collision_free(
            route,
            occupied,
            self.resolution,
            self.collision_check_spacing_factor)
        self._log_nbv_filter('route_validation', len(route), len(route) if route_safe else 0,
                             {'route_points': len(route), 'collision_free': route_safe,
                              'collision_voxels': len(occupied),
                              'resolution': self.resolution,
                              'spacing_factor': self.collision_check_spacing_factor})
        if not route_safe:
            self._plan_failure_reason = 'astar_removed_all' if len(route) == 0 else 'route_collision'
            self._failed_route_target = np.copy(valid_candidates[0]['target'])
            self.get_logger().warning('A* produced no collision-free route')
            return False

        trajectory_started = time.perf_counter()
        route_options = [('spline', route, targets)]
        target_changes = np.flatnonzero(np.any(targets != targets[0], axis=1))
        if len(target_changes) and target_changes[0] >= 4:
            first_end = int(target_changes[0])
            route_options.append(('first_target_spline', route[:first_end], targets[:first_end]))
        safe_options = []
        # A voxel polyline is not a dynamically validated trajectory for this controller.
        for option_name, option_route, option_targets in route_options:
            option = generate_smooth_trajectory(
                option_route, option_targets,
                num_points=int(self.trajectory_points),
                smoothing=float(self.spline_smoothing),
                sampling_spacing=float(self.waypoint_spacing))
            if len(option) == 0:
                self._log_nbv_filter('trajectory_option', 0, 0,
                                     {'option': option_name, 'reason': 'empty_trajectory',
                                      'route_points': len(option_route), 'spline_minimum_points': 4})
                continue
            option = option.copy()
            option[0, :3] = option_route[0]
            option[-1, :3] = option_route[-1]
            if self.optimize_motion:
                option[:, 3] = np.unwrap(np.r_[yaw, option[:, 3]])[1:]
                option[0, 3] = yaw
            option = resample_trajectory_by_distance(
                option, float(self.waypoint_spacing),
                max_points=int(self.trajectory_points))
            option_safe = path_is_collision_free(
                    option[:, :3], occupied, self.resolution,
                    self.collision_check_spacing_factor)
            self._log_nbv_filter('trajectory_option', len(option),
                                 len(option) if option_safe else 0,
                                 {'option': option_name, 'collision_free': option_safe,
                                  'waypoint_spacing': self.waypoint_spacing,
                                  'requested_max_points': int(self.trajectory_points),
                                  'mandatory_corners_exceed_cap': len(option) > int(self.trajectory_points)})
            if option_safe:
                safe_options.append((option, option_targets))
                if not self.optimize_motion:
                    break  # Baseline mode needs only the first validated route.
        if not safe_options:
            self._plan_failure_reason = 'all_trajectory_options_rejected'
            # This is the first target in travel order, which can differ from
            # the highest-utility candidate. Its prefix spline also failed.
            self._failed_route_target = np.copy(targets[0])
            self.get_logger().warning(
                'Resampled spline intersects the collision boundary; '
                'route rejected')
            self.telemetry.record_stage(
                'trajectory_generation_and_validation',
                (time.perf_counter() - trajectory_started) * 1000.0,
                iteration=self.iteration, details='rejected')
            return False
        trajectory, execution_targets = min(safe_options, key=lambda option: continuity_cost(
            option[0], velocity, recent_positions, **self.motion_cost_parameters)
        ) if self.optimize_motion else safe_options[0]
        if self.optimize_motion:
            before_cost = continuity_cost(
                trajectory, velocity, recent_positions, **self.motion_cost_parameters)
            trajectory = optimize_trajectory(
                trajectory, position, yaw, velocity,
                occupied, self.resolution, recent_positions,
                **self.motion_cost_parameters,
                max_deviation=float(self.optimizer_max_deviation),
                max_iterations=int(self.optimizer_max_iterations),
                collision_spacing_factor=float(self.collision_check_spacing_factor))
            after_cost = continuity_cost(
                trajectory, velocity, recent_positions, **self.motion_cost_parameters)
            self.get_logger().info(
                f'Motion optimization: speed={np.linalg.norm(velocity):.2f} m/s, '
                f'continuity cost={before_cost:.2f} -> {after_cost:.2f}')
        self.telemetry.record_stage(
            'trajectory_generation_and_validation',
            (time.perf_counter() - trajectory_started) * 1000.0,
            iteration=self.iteration)
            
        publish_started = time.perf_counter()
        latest_position, latest_yaw, _ = self._motion_snapshot()
        before_reconnect = len(trajectory)
        trajectory = reconnect_trajectory(
            trajectory, latest_position, latest_yaw,
            occupied, self.resolution, self.collision_check_spacing_factor)
        self._log_nbv_filter('trajectory_reconnection', before_reconnect, len(trajectory),
                             {'latest_vehicle_position': latest_position,
                              'latest_yaw': latest_yaw})
        if len(trajectory) == 0:
            self._plan_failure_reason = 'trajectory_reconnection_rejected'
            self.force_replan = True
            self.get_logger().warning(
                'Vehicle moved during planning; no safe entry to the new route. '
                'Requesting a fresh plan.')
        success = self._publish_trajectory(trajectory, occupied)
        self.telemetry.record_stage(
            'trajectory_publish', (time.perf_counter() - publish_started) * 1000.0,
            iteration=self.iteration,
            details='published' if success else 'rejected')
        if success:
            self.selected_targets = [np.copy(target) for target in execution_targets]
            # DELETED: We no longer add targets to memory here!
        elif self._plan_failure_reason is None:
            self._plan_failure_reason = 'trajectory_publish_rejected'
            self._failed_route_target = np.copy(execution_targets[0])
        return success

    def _expire_target_blacklist(self):
        now = time.perf_counter()
        retained = []
        for target in self.blacklisted_targets:
            expiry = self.blacklist_expiries.get(tuple(target), 0.0)
            if expiry > now:
                retained.append(target)
            else:
                self.blacklist_expiries.pop(tuple(target), None)
                self._log_nbv_filter('blacklist_expired', 1, 1, {'target': target})
        self.blacklisted_targets = retained

    def _blacklist_failed_target(self):
        """Suppress the failed target briefly so another structure can be tried."""
        if self._failed_route_target is None or self.blacklist_cooldown_s <= 0.0:
            return False
        target = np.asarray(self._failed_route_target, dtype=float).copy()
        existing = next((bt for bt in self.blacklisted_targets
                         if np.linalg.norm(target - bt) < 0.5), None)
        if existing is None:
            self.blacklisted_targets.append(target)
        else:
            target = existing
        self.blacklist_expiries[tuple(target)] = time.perf_counter() + self.blacklist_cooldown_s
        self._log_nbv_filter('blacklist_addition', 1, 0,
                             {'target': target, 'planning_failure': self._plan_failure_reason,
                              'radius': 0.5, 'expires': True,
                              'cooldown_s': self.blacklist_cooldown_s})
        self.get_logger().warning(
            f'Planning rejected ({self._plan_failure_reason}); '
            f'blacklisting failed target {target.tolist()} for {self.blacklist_cooldown_s:g}s.')
        self.force_replan = True
        return True

    def _log_nbv_filter(self, stage, input_count, output_count, details):
        """Log every filter transition to ROS and a per-frame JSONL event stream."""
        if not self.nbv_filter_logging:
            return
        if stage == 'nbv_empty':
            self._nbv_empty_reason = details.get('reason')
        safe_details = MidrunLogger._json_safe(details)
        self.get_logger().info(
            f'[NBV_FILTER iteration={self.iteration} frame={self.depth_sequence}] '
            f'{stage}: {input_count} -> {output_count} '
            f'{json.dumps(safe_details, allow_nan=False)}')
        self.telemetry.record_filter(stage, input_count, output_count, safe_details,
                                     iteration=self.iteration, frame_id=self.depth_sequence)

    def _active_trajectory_is_safe(self, occupied):
        """Check the currently published route against the updated map."""
        if len(self.active_trajectory) == 0:
            return True
        return path_is_collision_free(
            self.active_trajectory[:, :3],
            occupied,
            self.resolution,
            self.collision_check_spacing_factor)

    def _cancel_active_trajectory(self):
        """Cancel the downstream queue as well as the planner's cached route."""
        previous_count = len(self.active_trajectory)
        self.waypoint_pub.publish(Float32MultiArray())
        self.active_trajectory = np.empty((0, 4), dtype=np.float64)
        self.selected_targets = []
        self._log_nbv_filter('trajectory_cancelled', previous_count, 0,
                             {'reason': 'new_obstacle_on_active_route',
                              'downstream_queue_cleared': True})

    def timer_callback(self):
        """Update the map and periodically publish the next viewpoints."""
        if self.finished_mapping:
            return
            
        # 1. DIAGNOSTIC LOGS: Find out exactly what is blocking the pipeline
        if not self.odom_received:
            self.get_logger().warning('Pipeline blocked: Waiting for Odometry...', throttle_duration_sec=2.0)
            return
        if not self.rgb_received:
            self.get_logger().warning('Pipeline blocked: Waiting for RGB camera...', throttle_duration_sec=2.0)
            return
        if self.depth_frame is None:
            self.get_logger().warning('Pipeline blocked: Waiting for Depth frame...', throttle_duration_sec=2.0)
            return
        if self.depth_sequence == self.processed_depth_sequence:
            self.get_logger().warning('Pipeline blocked: Waiting for NEW Depth frame (Sequence unchanged)...', throttle_duration_sec=2.0)
            return

        # 2. ACCUMULATION PHASE
        started = time.perf_counter()
        stage_started = time.perf_counter()
        self._accumulate_depth()
        self.telemetry.record_stage(
            'depth_to_octomap', (time.perf_counter() - stage_started) * 1000.0,
            iteration=self.iteration, frame_id=self.depth_sequence)
        self.processed_depth_sequence = self.depth_sequence
        
        stage_started = time.perf_counter()
        occupied_points = self._occupied_points()
        self.telemetry.record_stage(
            'octomap_extract', (time.perf_counter() - stage_started) * 1000.0,
            iteration=self.iteration, frame_id=self.depth_sequence)
        self.get_logger().info(f'Iteration {self.iteration}: OctoMap total size = {len(occupied_points)} voxels.', throttle_duration_sec=2.0)
        
        stage_started = time.perf_counter()
        if self.use_incremental_map_cache:
            new_voxels = self.pending_occupied_voxels
            self.pending_occupied_voxels = set()
            if new_voxels:
                self.collision_voxels.update(
                    inflate_occupied(
                        new_voxels,
                        self.collision_radius + self.collision_tracking_margin,
                        self.resolution))
            collision_voxels = self.collision_voxels
        else:
            occupied_indices = set(map(
                tuple, np.floor(occupied_points / self.resolution).astype(int)))
            collision_voxels = inflate_occupied(
                occupied_indices, self.collision_radius + self.collision_tracking_margin, self.resolution)
        self.telemetry.record_stage(
            'collision_inflation', (time.perf_counter() - stage_started) * 1000.0,
            iteration=self.iteration, frame_id=self.depth_sequence)
            
        route_blocked = not self._active_trajectory_is_safe(collision_voxels)
        if route_blocked:
            self._cancel_active_trajectory()
            self.get_logger().warning('🚨 Active trajectory collided with newly mapped space; forcing replan!')


        arrived_at_target = False
        if self.replan_on_target_arrival and self.selected_targets:
            for target in self.selected_targets:
                # Calculate absolute distance to the NBV centroid
                dist = np.linalg.norm(self.position - target)
                
                # Use a 1.5m radius because B-splines smooth out corners, 
                # so the AUV might not hit the exact dead-center millimeter.
                if dist < self.arrival_target_radius:
                    self.get_logger().info(f'📍 Arrived at NBV Target (Distance: {dist:.2f}m). Forcing instant replan!')
                    arrived_at_target = True
                    break

        # 3. PLANNING PHASE
        should_plan = (
            self.iteration % max(1, int(self.frame_period)) == 0
            or route_blocked
            or self.force_replan
            or arrived_at_target
        )
        
        if should_plan:
            self._nbv_empty_reason = None
            self._log_nbv_filter('planning_begin', len(occupied_points), len(occupied_points),
                                 {'periodic': self.iteration % max(1, int(self.frame_period)) == 0,
                                  'route_blocked': bool(route_blocked),
                                  'forced': bool(self.force_replan),
                                  'arrived_at_target': bool(arrived_at_target),
                                  'collision_voxels': len(collision_voxels),
                                  'blacklisted_targets': self.blacklisted_targets,
                                  'minimum_utility': self.min_utility})
            self.force_replan = False # Reset the flag
            self.get_logger().info(f'--- PLANNING TRIGGERED (Iteration {self.iteration}) ---')
            self.get_logger().info(f'Saving OctoMap to: {self.map_path}')
            stage_started = time.perf_counter()
            self.octree.write(self.map_path)
            self.telemetry.record_stage(
                'octomap_save', (time.perf_counter() - stage_started) * 1000.0,
                iteration=self.iteration)
            
            nbv_position, nbv_yaw, nbv_history = self._motion_snapshot()
            roll, pitch = nbv_history[-1][2:4] if nbv_history else (0., 0.)
            camera_position = nbv_position + body_rotation(roll, pitch, nbv_yaw) @ self.camera_offset
            heading = np.array([math.cos(nbv_yaw), math.sin(nbv_yaw), 0.0])
            
            nbv_started = time.perf_counter()
            try:
                self.get_logger().info('Calculating NBV waypoints...')
                candidates = calculate_nbv_waypoints(
                    occupied_points,
                    camera_position,
                    heading,
                    self.observation_history,
                    self.nbv_parameters,
                    timing_callback=lambda stage, duration_ms:
                        self.telemetry.record_stage(
                            stage, duration_ms, iteration=self.iteration),
                )
                self.get_logger().info(f'Found {len(candidates)} valid NBV targets.')
            except (ValueError, RuntimeError) as error:
                self.get_logger().error(f'NBV calculation failed: {error}')
                self._nbv_empty_reason = 'nbv_exception'
                self._log_nbv_filter('nbv_exception', len(occupied_points), 0,
                                     {'exception_type': type(error).__name__, 'error': str(error)})
                candidates = []
            self.telemetry.record_stage(
                'nbv_generation', (time.perf_counter() - nbv_started) * 1000.0,
                iteration=self.iteration,
                details=f'candidates={len(candidates)}')

            self._expire_target_blacklist()
            safe_candidates = []
            for cand in candidates:
                # If this target is within 0.5m of a known bad coordinate, drop it
                is_banned = any(np.linalg.norm(cand['target'] - bt) < 0.5 for bt in self.blacklisted_targets)
                self._log_nbv_filter('candidate_blacklist', 1, int(not is_banned),
                                     {'cluster_id': cand.get('cluster_id'), 'target': cand['target'],
                                      'radius': 0.5,
                                      'matching_bans': [
                                          {'target': bt, 'distance': float(np.linalg.norm(cand['target'] - bt)),
                                           'remaining_s': max(0.0, self.blacklist_expiries[tuple(bt)] - time.perf_counter())}
                                          for bt in self.blacklisted_targets
                                          if np.linalg.norm(cand['target'] - bt) < 0.5]})
                if not is_banned:
                    safe_candidates.append(cand)

            if len(safe_candidates) < len(candidates):
                self.get_logger().info(f"Dropped {len(candidates) - len(safe_candidates)} blacklisted targets.")
            self._log_nbv_filter('blacklist', len(candidates), len(safe_candidates),
                                 {'radius': 0.5, 'blacklisted_target_count': len(self.blacklisted_targets)})

            # Pass the SAFE candidates to the planner
            published = self._plan_and_publish(safe_candidates, collision_voxels)
            failure_reason = ((self._nbv_empty_reason or 'nbv_generation_returned_no_candidates')
                              if not candidates else
                              'blacklist_removed_all' if not safe_candidates else
                              self._plan_failure_reason)
            self._log_nbv_filter('planning_result', len(candidates), int(published),
                                 {'published': bool(published),
                                  'reason': 'published' if published else failure_reason,
                                  'generated_candidates': len(candidates),
                                  'after_blacklist': len(safe_candidates),
                                  'active_trajectory_points': len(self.active_trajectory)})
            if not published:
                self.get_logger().warning(f'[NBV_EMPTY] No trajectory published: {failure_reason}')


            if published:
                self.failed_plan_count = 0  # Reset counter on success
            else:
                if self.force_replan:
                    pass  # Vehicle simply moved during planning; not a failure of the candidates.
                elif len(safe_candidates) > 0:
                    # We had candidates, but failed to publish. 
                    # Did we identify a specific bad target to blacklist?
                    if not self._blacklist_failed_target():
                        self._log_nbv_filter('blacklist_skipped', len(safe_candidates), len(safe_candidates),
                                             {'reason': 'no_individual_failed_target',
                                              'planning_failure': self._plan_failure_reason})
                        
                        # CRITICAL FIX: If we didn't blacklist anything but still failed 
                        # (e.g., minimum_utility_removed_all), the entire batch is unusable. Strike out!
                        self.failed_plan_count += 1
                        self.get_logger().warning(
                            f'All safe candidates failed ({self._plan_failure_reason}). Strike {self.failed_plan_count}/3'
                        )
                else:
                    # CRITICAL FIX: Increment if NO safe candidates exist (all banned or none found).
                    self.failed_plan_count += 1
                    self.get_logger().warning(
                        f'No valid candidates remain ({failure_reason}). Strike {self.failed_plan_count}/3'
                    )
            
            if self.failed_plan_count >= 3:
                self._log_nbv_filter('mission_completion', self.failed_plan_count, 0,
                                     {'reason': 'three_empty_candidate_passes',
                                      'last_empty_reason': failure_reason,
                                      'blacklisted_target_count': len(self.blacklisted_targets)})
                self.finished_mapping = True
                self.finished_pub.publish(Bool(data=True))
                
                self.get_logger().info('Flushing final RAM state to OctoMap file...')
                self.octree.write(self.map_path)
                self.get_logger().info('✅ Mapping finished. Explored all accessible frontiers.')
                
            # CRITICAL FIX: Only end the mission if there are literally zero frontiers left.
            # if len(candidates) == 0 and self.started_mapping:
            #     self.finished_mapping = True
            #     self.finished_pub.publish(Bool(data=True))
                
            #     # FORCE FINAL SAVE TO DISK BEFORE EXITING
            #     self.get_logger().info('Flushing final RAM state to OctoMap file...')
            #     self.octree.write(self.map_path)
                
            #     self.get_logger().info('✅ Mapping finished. Explored all accessible frontiers.')
            # elif not published and len(candidates) > 0:
            #     self.get_logger().warning('Failed to generate a collision-free spline to the targets. Retrying next cycle...')

            elapsed_ms = (time.perf_counter() - started) * 1000.0
            self.get_logger().info(f'Planning cycle completed in {elapsed_ms:.2f} ms.')
            self.telemetry.record_iteration(
                self.iteration,
                elapsed_ms,
                self.position,
                candidates,
                self.active_trajectory,
                published,
                route_blocked,
                self.selected_targets)
            
            self.evaluation_file.write(
                f'Process Time (ms): {elapsed_ms}, '
                f'Total Leaves: {self.octree.getNumLeafNodes()}\n')
            self.evaluation_file.flush()
        self.telemetry.record_stage(
            'planning_cycle_total', (time.perf_counter() - started) * 1000.0,
            iteration=self.iteration, frame_id=self.depth_sequence)

        self.iteration += 1

    def destroy_node(self):
        """Close the evaluation log and save the final map before destroying the ROS node."""
        self.get_logger().info('Node shutting down. Saving final OctoMap state...')
        try:
            self.octree.write(self.map_path)
        except Exception as e:
            self.get_logger().error(f'Failed to save final map on exit: {e}')
            
        self.evaluation_file.close()
        super().destroy_node()


def main(args=None):
    """Run the Python NBV utility node."""
    rclpy.init(args=args)
    node = NBVUtilityNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
