import math
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


class NBVUtilityNode(Node):
    """Build a depth map online and publish NBV viewpoints."""

    def __init__(self):
        super().__init__('nbv_utility_node')

        self.resolution = self.declare_parameter('voxel_resolution', 0.1).value
        self.max_depth = self.declare_parameter('max_depth', 3.0).value
        self.min_depth = self.declare_parameter('min_depth', 0.5).value
        self.z_min_target = self.declare_parameter(
            'z_min_target', -31.5).value
        self.depth_stride = self.declare_parameter('depth_stride', 8).value
        self.depth_scale = self.declare_parameter('depth_scale', 1.0).value
        self.frame_period = self.declare_parameter('planning_period', 10).value
        self.min_utility = self.declare_parameter(
            'min_utility_threshold', 0.005).value
        self.collision_radius = self.declare_parameter(
            'collision_radius', 0.6).value
        self.trajectory_points = self.declare_parameter(
            'trajectory_points', 300).value
        self.waypoint_spacing = self.declare_parameter(
            'waypoint_spacing', 0.5).value
        self.map_path = self.declare_parameter(
            'octomap_path', 'results/final_map_ros2.bt').value
        self.visit_radius = self.declare_parameter('visit_radius', 0.25).value
        self.results_dir = self.declare_parameter(
            'results_dir', 'results').value
        self.use_incremental_map_cache = self.declare_parameter(
            'use_incremental_map_cache', True).value
        self.nbv_parameters = {
            name: self.declare_parameter(
                f'nbv_{name}', default).value
            for name, default in DEFAULT_NBV_PARAMETERS.items()
            if name != 'z_min_target'
        }
        self.nbv_parameters['z_min_target'] = self.z_min_target
        self.astar_max_steps = self.declare_parameter(
            'astar_max_steps', 50000).value
        self.spline_smoothing = self.declare_parameter(
            'spline_smoothing', 2.0).value
        self.collision_check_spacing_factor = self.declare_parameter(
            'collision_check_spacing_factor', 0.5).value
        self.arrival_target_radius = self.declare_parameter(
            'arrival_target_radius', 1.5).value
        self.optimize_motion = self.declare_parameter(
            'trajectory_optimization_enabled', True).value
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

        self.get_logger().info('NBV utility node started in Python')

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

        cr, sr = math.cos(self.latched_roll), math.sin(self.latched_roll)
        cp, sp = math.cos(self.latched_pitch), math.sin(self.latched_pitch)
        cy, sy = math.cos(self.latched_yaw), math.sin(self.latched_yaw)
        rotation = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ])
        return local_points @ rotation.T + self.latched_position + self.camera_offset

    def _accumulate_depth(self):
        points = self._depth_points(self.depth_frame)
        if len(points) == 0:
            # Warn if the camera only sees empty water outside max_depth
            self.get_logger().warning('Depth projection yielded 0 valid points (all out of range or invalid).', throttle_duration_sec=2.0)
            return

        if len(points) > 4000:
            # Randomly sample exactly 4000 points to preserve the pipe's overall shape
            indices = np.random.choice(len(points), 4000, replace=False)
            points = points[indices]

        if self.use_incremental_map_cache:
            voxel_indices = np.floor(
                points / self.resolution).astype(np.int32)
            new_voxels = set(map(tuple, voxel_indices.tolist()))
            new_voxels.difference_update(self.occupied_voxels)
            self.occupied_voxels.update(new_voxels)
            self.pending_occupied_voxels.update(new_voxels)
            
        sensor_origin = self.latched_position + self.camera_offset
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

    def _publish_trajectory(self, trajectory):
        """Publish a collision-checked Nx4 trajectory."""
        if len(trajectory) == 0:
            return False

        message = Float32MultiArray()
        message.data = np.asarray(
            trajectory, dtype=np.float32).flatten().tolist()
        self.waypoint_pub.publish(message)
        self.active_trajectory = np.asarray(trajectory, dtype=np.float64)
        self.started_mapping = True
        self.get_logger().info(
            f'Published {len(self.active_trajectory)} trajectory points')
        return True

    def _plan_and_publish(self, candidates, occupied):
        """Run A*, smooth its route, and publish only a safe trajectory."""
        valid_candidates = [
            candidate for candidate in candidates
            if candidate['utility'] >= self.min_utility
        ]
        if not valid_candidates:
            return False

        route_candidates = [
            (np.asarray(candidate['position'], dtype=np.float64),
             np.asarray(candidate['target'], dtype=np.float64))
            for candidate in valid_candidates
        ]
        route_started = time.perf_counter()
        position, yaw, history = self._motion_snapshot()
        velocity = estimate_velocity(
            history, float(self.velocity_history_window))
        latest_time = history[-1][0] if history else 0.0
        recent_positions = [sample[1] + self.camera_offset
                            for sample in history
                            if 0.0 <= latest_time - sample[0]
                            <= float(self.trajectory_history_window)]
        if len(recent_positions) > 64:
            indices = np.linspace(0, len(recent_positions) - 1, 64).astype(int)
            recent_positions = [recent_positions[index] for index in indices]
        route, targets = plan_multi_waypoint_route(
            position + self.camera_offset,
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
        if len(route) == 0 or not path_is_collision_free(
            route,
            occupied,
            self.resolution,
            self.collision_check_spacing_factor):
            self.get_logger().warning('A* produced no collision-free route')
            return False

        trajectory_started = time.perf_counter()
        trajectory = generate_smooth_trajectory(
            route,
            targets,
            num_points=int(self.trajectory_points),
            smoothing=float(self.spline_smoothing),
            sampling_spacing=float(self.waypoint_spacing),
        )
        raw_trajectory = np.column_stack((route, np.arctan2(
            targets[:, 1] - route[:, 1], targets[:, 0] - route[:, 0])))
        safe_options = []
        for option in (trajectory, raw_trajectory):
            if len(option) == 0:
                continue
            option = option.copy()
            option[0, :3] = route[0]
            option[-1, :3] = route[-1]
            if self.optimize_motion:
                option[:, 3] = np.unwrap(np.r_[yaw, option[:, 3]])[1:]
                option[0, 3] = yaw
            option = resample_trajectory_by_distance(
                option, float(self.waypoint_spacing),
                max_points=int(self.trajectory_points))
            if path_is_collision_free(
                    option[:, :3], occupied, self.resolution,
                    self.collision_check_spacing_factor):
                safe_options.append(option)
        if not safe_options:
            self.get_logger().warning(
                'Resampled spline intersects the collision boundary; '
                'route rejected')
            self.telemetry.record_stage(
                'trajectory_generation_and_validation',
                (time.perf_counter() - trajectory_started) * 1000.0,
                iteration=self.iteration, details='rejected')
            return False
        trajectory = min(safe_options, key=lambda option: continuity_cost(
            option, velocity, recent_positions, **self.motion_cost_parameters)
        ) if self.optimize_motion else safe_options[0]
        if self.optimize_motion:
            before_cost = continuity_cost(
                trajectory, velocity, recent_positions, **self.motion_cost_parameters)
            trajectory = optimize_trajectory(
                trajectory, position + self.camera_offset, yaw, velocity,
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
        trajectory = reconnect_trajectory(
            trajectory, latest_position + self.camera_offset, latest_yaw,
            occupied, self.resolution, self.collision_check_spacing_factor)
        if len(trajectory) == 0:
            self.force_replan = True
            self.get_logger().warning(
                'Vehicle moved during planning; no safe entry to the new route. '
                'Requesting a fresh plan.')
        success = self._publish_trajectory(trajectory)
        self.telemetry.record_stage(
            'trajectory_publish', (time.perf_counter() - publish_started) * 1000.0,
            iteration=self.iteration,
            details='published' if success else 'rejected')
        if success:
            self.selected_targets = [np.copy(target) for target in targets]
            # DELETED: We no longer add targets to memory here!
        return success

    def _active_trajectory_is_safe(self, occupied):
        """Check the currently published route against the updated map."""
        if len(self.active_trajectory) == 0:
            return True
        return path_is_collision_free(
            self.active_trajectory[:, :3],
            occupied,
            self.resolution,
            self.collision_check_spacing_factor)

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
                        self.collision_radius,
                        self.resolution))
            collision_voxels = self.collision_voxels
        else:
            occupied_indices = set(map(
                tuple, np.floor(occupied_points / self.resolution).astype(int)))
            collision_voxels = inflate_occupied(
                occupied_indices, self.collision_radius, self.resolution)
        self.telemetry.record_stage(
            'collision_inflation', (time.perf_counter() - stage_started) * 1000.0,
            iteration=self.iteration, frame_id=self.depth_sequence)
            
        route_blocked = not self._active_trajectory_is_safe(collision_voxels)
        if route_blocked:
            self.active_trajectory = np.empty((0, 4), dtype=np.float64)
            self.get_logger().warning('🚨 Active trajectory collided with newly mapped space; forcing replan!')


        arrived_at_target = False
        if self.selected_targets:
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
            self.force_replan = False # Reset the flag
            self.get_logger().info(f'--- PLANNING TRIGGERED (Iteration {self.iteration}) ---')
            self.get_logger().info(f'Saving OctoMap to: {self.map_path}')
            stage_started = time.perf_counter()
            self.octree.write(self.map_path)
            self.telemetry.record_stage(
                'octomap_save', (time.perf_counter() - stage_started) * 1000.0,
                iteration=self.iteration)
            
            nbv_position, nbv_yaw, _ = self._motion_snapshot()
            heading = np.array([math.cos(nbv_yaw), math.sin(nbv_yaw), 0.0])
            
            nbv_started = time.perf_counter()
            try:
                self.get_logger().info('Calculating NBV waypoints...')
                candidates = calculate_nbv_waypoints(
                    occupied_points,
                    nbv_position + self.camera_offset,
                    heading,
                    self.observation_history,
                    self.nbv_parameters,
                )
                self.get_logger().info(f'Found {len(candidates)} valid NBV targets.')
            except (ValueError, RuntimeError) as error:
                self.get_logger().error(f'NBV calculation failed: {error}')
                candidates = []
            self.telemetry.record_stage(
                'nbv_generation', (time.perf_counter() - nbv_started) * 1000.0,
                iteration=self.iteration,
                details=f'candidates={len(candidates)}')

            safe_candidates = []
            for cand in candidates:
                # If this target is within 0.5m of a known bad coordinate, drop it
                is_banned = any(np.linalg.norm(cand['target'] - bt) < 0.5 for bt in self.blacklisted_targets)
                if not is_banned:
                    safe_candidates.append(cand)

            if len(safe_candidates) < len(candidates):
                self.get_logger().info(f"Dropped {len(candidates) - len(safe_candidates)} blacklisted targets.")

            # Pass the SAFE candidates to the planner
            published = self._plan_and_publish(safe_candidates, collision_voxels)


            if published:
                self.failed_plan_count = 0  # Reset counter on success
            else:
                if self.force_replan:
                    pass  # A stale entry is not evidence that the target is bad.
                elif len(safe_candidates) > 0:
                    self.get_logger().warning('Spline rejected. Blacklisting top target to prevent infinite loop.')
                    # Hard-ban the physical coordinate of the culprit so it never gets passed to A* again
                    self.blacklisted_targets.append(np.copy(safe_candidates[0]['target']))
                else:
                    self.failed_plan_count += 1
                    self.get_logger().warning(f'No frontiers found. Strike {self.failed_plan_count}/3')

            if self.failed_plan_count >= 3:
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
