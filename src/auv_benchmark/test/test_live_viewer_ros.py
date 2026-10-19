"""Verify real viewer subscriptions independently of a desktop window."""

import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')


def test_viewer_receives_route_replacement_clear_odometry_and_completion(tmp_path):
    from auv_benchmark.live_viewer import LiveViewer
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Bool, Float32MultiArray

    rclpy.init(args=['--ros-args', '-p', f'map_path:={tmp_path / "missing.bt"}'])
    viewer = LiveViewer()
    assert viewer.frontiers.path == tmp_path / 'planner_nbv_filters.jsonl'
    assert viewer.snapshots.map.filter_outliers
    source = Node('viewer_test_source')
    executor = SingleThreadedExecutor()
    executor.add_node(viewer)
    executor.add_node(source)
    route = source.create_publisher(Float32MultiArray, 'object/areaWaypoints', 10)
    odom = source.create_publisher(Odometry, 'holocean/odom', 10)
    completion = source.create_publisher(Bool, '/movement/finished_execution', 10)

    def wait(predicate):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        pytest.fail('Viewer subscription timed out')

    try:
        wait(lambda: all(p.get_subscription_count() for p in (route, odom, completion)))
        route.publish(Float32MultiArray(data=[1., 2., 3., 0., 4., 5., 6., 0.]))
        wait(lambda: len(viewer.trajectory) == 2)
        route.publish(Float32MultiArray(data=[7., 8., 9., 0.]))
        wait(lambda: len(viewer.trajectory) == 1)
        np.testing.assert_array_equal(viewer.trajectory, [[7, 8, 9]])
        message = Odometry()
        message.pose.pose.position.x = 4.
        odom.publish(message)
        wait(lambda: viewer.vehicle is not None)
        np.testing.assert_array_equal(viewer.vehicle, [4, 0, 0])
        viewer.frontiers_dirty = False
        route.publish(Float32MultiArray(data=[]))
        wait(lambda: not len(viewer.trajectory))
        assert viewer.frontiers_dirty
        route.publish(Float32MultiArray(data=[1., 2., 3., 0.]))
        wait(lambda: len(viewer.trajectory) == 1)
        viewer.frontiers_dirty = False
        completion.publish(Bool(data=True))
        wait(lambda: not len(viewer.trajectory))
        assert viewer.frontiers_dirty
    finally:
        executor.shutdown()
        viewer.destroy_node()
        source.destroy_node()
        rclpy.shutdown()


def test_viewer_display_filter_overrides():
    from auv_benchmark.live_viewer import LiveViewer

    rclpy.init(args=['--ros-args', '-p', 'filter_outliers:=false',
                    '-p', 'outlier_neighbors:=8', '-p', 'outlier_std_ratio:=2.0'])
    viewer = None
    try:
        viewer = LiveViewer()
        assert not viewer.snapshots.map.filter_outliers
        assert viewer.snapshots.map.outlier_neighbors == 8
        assert viewer.snapshots.map.outlier_std_ratio == 2.0
    finally:
        if viewer is not None:
            viewer.destroy_node()
        rclpy.shutdown()
