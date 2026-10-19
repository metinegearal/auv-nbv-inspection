"""Exercise production evaluator subscriptions and completion reports over ROS."""
import csv
import json
import time

import pytest

rclpy = pytest.importorskip('rclpy')


def test_waypoint_and_odometry_topics_save_response_on_completion(tmp_path, monkeypatch):
    from benchmark_evaluator.run_baselines import BenchmarkLogger
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Bool, Float32MultiArray

    monkeypatch.setenv('MPLBACKEND', 'Agg')
    (tmp_path / 'planner_nbv_filters.jsonl').write_text(json.dumps({
        'stage': 'blacklist_addition', 'iteration': 10, 'frame_id': 20,
        'details': {'target': [10., 20., -30.], 'radius': 1.,
                    'expires': False, 'reason': 'all_nbv_rays_collide'}}) + '\n')
    rclpy.init(args=['--ros-args', '-p', f'results_dir:={tmp_path}',
                     '-p', 'show_reference_objects:=false'])
    evaluator = BenchmarkLogger()
    publisher = Node('motion_response_test_source')
    executor = SingleThreadedExecutor()
    executor.add_node(evaluator)
    executor.add_node(publisher)
    odometry = publisher.create_publisher(Odometry, 'holocean/odom', 10)
    waypoint = publisher.create_publisher(Float32MultiArray, '/trajectory/waypoint', 10)
    completion = publisher.create_publisher(Bool, '/movement/finished_execution', 10)

    def wait(condition):
        end = time.monotonic() + 10.
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.02)
            if condition():
                return
        raise AssertionError('Evaluator transport timed out')

    try:
        wait(lambda: all(pub.get_subscription_count() for pub in (odometry, waypoint, completion)))
        wait(lambda: (tmp_path / 'permanent_ban_spheres.json').exists())
        bans = json.loads((tmp_path / 'permanent_ban_spheres.json').read_text())
        assert bans['spheres'][0]['center'] == [10., 20., -30.]
        assert bans['spheres'][0]['radius_m'] == 1.
        odom = Odometry()
        odom.pose.pose.orientation.w = 1.
        odom.pose.pose.position.x = 10.
        odometry.publish(odom)
        wait(lambda: evaluator.evaluation.motion_response.position is not None)
        command = Float32MultiArray(data=[11., 0., 0., 0., 99., 99., 99., 0.])
        waypoint.publish(command)
        wait(lambda: evaluator.evaluation.motion_response.monitor is not None)
        for index, position in enumerate((10.1, 10.5, 10.9, 11.), start=2):
            odom.pose.pose.position.x = position
            odometry.publish(odom)
            wait(lambda: evaluator.evaluation.motion_response.monitor.sample_count >= index)
        waypoint.publish(command)
        completion.publish(Bool(data=True))
        wait(lambda: evaluator.evaluation.data_saved)
        assert evaluator.evaluation.motion_response.episode_id == 1
        with (tmp_path / 'motion_response_metrics.csv').open() as stream:
            records = list(csv.DictReader(stream))
        assert len(records) == 1
        assert records[0]['target_x_m'] == '11.0'
        assert float(records[0]['rise_time_s']) >= 0.
        assert records[0]['final_error_m'] == '0.0'
        assert records[0]['end_reason'] == 'mission_finished'
        assert (tmp_path / 'motion_control_response.png').stat().st_size > 0
        assert len(list(tmp_path.glob('permanent_ban_spheres_*.json'))) == 1
    finally:
        executor.shutdown()
        evaluator.destroy_node()
        publisher.destroy_node()
        rclpy.shutdown()
