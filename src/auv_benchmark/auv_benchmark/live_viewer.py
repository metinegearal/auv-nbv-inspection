"""Independent ROS/Open3D viewer for map snapshots and live planned routes."""

from concurrent.futures import ThreadPoolExecutor
import math
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray

from .adapters.live_map import SnapshotUpdates, trajectory_points
from .adapters.frontier_targets import FrontierTargets


class LiveViewer(Node):
    def __init__(self):
        super().__init__('live_viewer')
        defaults = {
            'map_path': 'results/final_map_ros2.bt',
            'map_poll_period_s': 1.0,
            'render_hz': 20.0,
            'voxel_size_m': 0.2,
            'filter_outliers': True,
            'outlier_neighbors': 15,
            'outlier_std_ratio': 1.5,
            'chunk_size_m': 4.0,
            'max_chunks_per_frame': 8,
            'point_size': 3.0,
            'window_width': 1280,
            'window_height': 800,
            'trajectory_topic': 'object/areaWaypoints',
            'odometry_topic': 'holocean/odom',
            'completion_topic': '/movement/finished_execution',
            'show_target_frontiers': True,
            'frontier_log_path': '',
            'frontier_poll_period_s': 0.5,
            'frontier_marker_radius_m': 0.35,
        }
        self.config = {key: self.declare_parameter(key, value).value
                       for key, value in defaults.items()}
        for key in ('map_poll_period_s', 'render_hz', 'voxel_size_m', 'chunk_size_m',
                    'max_chunks_per_frame', 'point_size', 'window_width', 'window_height',
                    'frontier_poll_period_s', 'frontier_marker_radius_m'):
            if not math.isfinite(self.config[key]) or self.config[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        self.snapshots = SnapshotUpdates(self.config['map_path'],
                                         self.config['voxel_size_m'], self.config['chunk_size_m'],
                                         filter_outliers=self.config['filter_outliers'],
                                         outlier_neighbors=self.config['outlier_neighbors'],
                                         outlier_std_ratio=self.config['outlier_std_ratio'])
        self.frontiers = FrontierTargets(self.config['frontier_log_path'] or
                                        Path(self.config['map_path']).parent / 'planner_nbv_filters.jsonl')
        self.target_frontiers = np.empty((0, 3))
        self.frontiers_dirty = False
        self.trajectory = np.empty((0, 3))
        self.trajectory_dirty = False
        self.vehicle = None
        self.vehicle_dirty = False
        self.create_subscription(Float32MultiArray, self.config['trajectory_topic'],
                                 self.on_trajectory, 10)
        self.create_subscription(Odometry, self.config['odometry_topic'],
                                 self.on_odometry, qos_profile_sensor_data)
        self.create_subscription(Bool, self.config['completion_topic'], self.on_completion, 10)

    def on_trajectory(self, message):
        try:
            points = trajectory_points(message.data)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        if not np.array_equal(points, self.trajectory):
            self.trajectory = points
            self.trajectory_dirty = True
            self.frontiers_dirty = True

    def on_odometry(self, message):
        p = message.pose.pose.position
        position = np.array([p.x, p.y, p.z])
        if np.isfinite(position).all() and not np.array_equal(position, self.vehicle):
            self.vehicle = position
            self.vehicle_dirty = True

    def on_completion(self, message):
        if message.data:
            self.trajectory = np.empty((0, 3))
            self.trajectory_dirty = True
            self.frontiers_dirty = True

    def run(self):
        # Match the existing desktop viewer's XWayland compatibility setup.
        if os.environ.get('DISPLAY') and (os.environ.get('WAYLAND_DISPLAY')
                                         or os.environ.get('XDG_SESSION_TYPE') == 'wayland'):
            os.environ['GDK_BACKEND'] = 'x11'
            os.environ['XDG_SESSION_TYPE'] = 'x11'
            os.environ.pop('WAYLAND_DISPLAY', None)
        import open3d as o3d

        view = o3d.visualization.VisualizerWithKeyCallback()
        if not view.create_window('Live map | cyan: route | yellow: frontier target | orange: vehicle | F: fit',
                                  self.config['window_width'], self.config['window_height']):
            raise RuntimeError('Could not open viewer window; check the desktop display')
        worker = ThreadPoolExecutor(max_workers=2, thread_name_prefix='viewer-data')
        future = None
        frontier_future = None
        geometries, pending = {}, {}
        route = o3d.geometry.LineSet()
        route_points = o3d.geometry.PointCloud()
        frontier_geometry = o3d.geometry.LineSet()
        unit_frontier = o3d.geometry.LineSet.create_from_triangle_mesh(
            o3d.geometry.TriangleMesh.create_sphere(
                radius=self.config['frontier_marker_radius_m'], resolution=8))
        frontier_vertices = np.asarray(unit_frontier.points)
        frontier_edges = np.asarray(unit_frontier.lines)
        frontiers_added = False
        vehicle = o3d.geometry.TriangleMesh.create_sphere(radius=0.18)
        vehicle.paint_uniform_color([1.0, 0.55, 0.05])
        route_added = vehicle_added = fitted = False
        last_vehicle = np.zeros(3)
        options = view.get_render_option()
        options.background_color = np.array([0.06, 0.07, 0.09])
        options.point_size = self.config['point_size']
        options.line_width = 3.0
        view.register_key_callback(ord('F'), lambda vis: vis.reset_view_point(True) or False)
        next_poll = 0.0
        next_frontier_poll = 0.0
        self.get_logger().info(f"Viewing {self.config['map_path']}; close the window to stop only the viewer")
        try:
            while rclpy.ok():
                started = time.monotonic()
                # Drain short bursts without letting high-rate odometry starve
                # the desktop event loop or delay a route-clear message.
                for _ in range(8):
                    rclpy.spin_once(self, timeout_sec=0.0)
                    if time.monotonic() - started >= 0.005:
                        break
                if future is not None and future.done():
                    try:
                        pending.update(future.result())
                    except Exception as error:
                        self.get_logger().warning(f'Map snapshot postponed: {error}',
                                                  throttle_duration_sec=10.0)
                    future = None
                if future is None and started >= next_poll:
                    future = worker.submit(self.snapshots.poll)
                    next_poll = started + self.config['map_poll_period_s']
                if frontier_future is not None and frontier_future.done():
                    try:
                        targets = frontier_future.result()
                        if not np.array_equal(targets, self.target_frontiers):
                            self.target_frontiers = targets
                            self.frontiers_dirty = True
                    except Exception as error:
                        self.get_logger().warning(f'Frontier overlay postponed: {error}',
                                                  throttle_duration_sec=10.0)
                    frontier_future = None
                if (self.config['show_target_frontiers'] and frontier_future is None
                        and started >= next_frontier_poll):
                    frontier_future = worker.submit(self.frontiers.poll)
                    next_frontier_poll = started + self.config['frontier_poll_period_s']
                for key in list(pending)[:self.config['max_chunks_per_frame']]:
                    points = pending.pop(key)
                    if points is None:
                        old = geometries.pop(key, None)
                        if old is not None:
                            view.remove_geometry(old, reset_bounding_box=False)
                        continue
                    cloud = geometries.get(key)
                    new = cloud is None
                    if new:
                        cloud = o3d.geometry.PointCloud()
                        geometries[key] = cloud
                    cloud.points = o3d.utility.Vector3dVector(points[:, :3])
                    colors = np.clip(points[:, 3:] / 255., 0, 1)
                    # Uncolored occupancy defaults to white; soften it so the
                    # cyan route remains visible against dense surfaces.
                    colors[np.all(points[:, 3:] == 255, axis=1)] = [0.55, 0.65, 0.75]
                    cloud.colors = o3d.utility.Vector3dVector(colors)
                    if new:
                        view.add_geometry(cloud, reset_bounding_box=False)
                    else:
                        view.update_geometry(cloud)
                if self.trajectory_dirty:
                    if route_added:
                        view.remove_geometry(route, reset_bounding_box=False)
                        view.remove_geometry(route_points, reset_bounding_box=False)
                        route_added = False
                    count = len(self.trajectory)
                    if count:
                        route.points = o3d.utility.Vector3dVector(self.trajectory)
                        route.lines = o3d.utility.Vector2iVector(
                            np.column_stack((np.arange(count - 1), np.arange(1, count))))
                        route.paint_uniform_color([0.05, 1.0, 1.0])
                        route_points.points = route.points
                        route_points.paint_uniform_color([0.05, 1.0, 1.0])
                        view.add_geometry(route, reset_bounding_box=False)
                        view.add_geometry(route_points, reset_bounding_box=False)
                        route_added = True
                    self.trajectory_dirty = False
                if self.frontiers_dirty:
                    if frontiers_added:
                        view.remove_geometry(frontier_geometry, reset_bounding_box=False)
                        frontiers_added = False
                    # A delayed log read must never revive a target after a
                    # live ROS route clear or mission completion.
                    if (self.config['show_target_frontiers'] and len(self.trajectory)
                            and len(self.target_frontiers)):
                        target = self.target_frontiers[0]
                        vertices = np.vstack((frontier_vertices + target,
                                              self.trajectory[-1], target))
                        edges = np.vstack((frontier_edges,
                                           [len(frontier_vertices), len(frontier_vertices) + 1]))
                        frontier_geometry.points = o3d.utility.Vector3dVector(vertices)
                        frontier_geometry.lines = o3d.utility.Vector2iVector(edges)
                        frontier_geometry.paint_uniform_color([1.0, 0.85, 0.05])
                        view.add_geometry(frontier_geometry, reset_bounding_box=False)
                        frontiers_added = True
                    self.frontiers_dirty = False
                if self.vehicle_dirty:
                    vehicle.translate(self.vehicle - last_vehicle)
                    last_vehicle = self.vehicle.copy()
                    if vehicle_added:
                        view.update_geometry(vehicle)
                    else:
                        view.add_geometry(vehicle, reset_bounding_box=False)
                        vehicle_added = True
                    self.vehicle_dirty = False
                if not fitted and geometries and not pending:
                    view.reset_view_point(True)
                    control = view.get_view_control()
                    control.set_front([0.7, -0.7, 0.55])
                    control.set_up([0.0, 0.0, 1.0])
                    fitted = True
                if not view.poll_events():
                    break
                view.update_renderer()
                time.sleep(max(0, 1.0 / self.config['render_hz'] - (time.monotonic() - started)))
        finally:
            worker.shutdown(wait=True, cancel_futures=True)
            view.destroy_window()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LiveViewer()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
