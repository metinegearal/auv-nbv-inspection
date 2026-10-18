"""Exercise real ROS discovery, parameter defaults, QoS, and mission completion."""
import os
import signal
import subprocess
import sys
import time

import pytest


@pytest.mark.parametrize('implementation', ['python', 'cpp'])
@pytest.mark.parametrize('profile', ['defaults', 'configured'])
def test_ros_topics_qos_and_completion(implementation, profile, tmp_path):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, MagneticField
    from std_msgs.msg import Bool, Float32MultiArray
    from rclpy.parameter_client import AsyncParameterClient
    from reference import ROOT, run

    executable = os.environ.get('PLANNER_NODE', '/tmp/planner-cpp-build/inspection_planner_node')
    env = dict(os.environ, ROS_LOG_DIR=str(tmp_path / 'ros_logs'), PYTHONDONTWRITEBYTECODE='1')
    env['PYTHONPATH'] = ':'.join(str(ROOT / p) for p in ('python_planner', 'auv_contracts', 'telemetry_adapters')) + ':' + env.get('PYTHONPATH', '')
    command = [executable] if implementation == 'cpp' else [sys.executable, '-c', 'from python_planner.planner_node import main; main()']
    command += ['--ros-args', '-r', '__node:=planner_node']
    overrides = {}
    if profile == 'configured':
        import yaml
        for name in ('nbv_params.yaml', 'params.yaml'):
            path = ROOT / 'bringup/config' / name
            command += ['--params-file', str(path)]
            overrides.update(yaml.safe_load(path.read_text())['planner_node']['ros__parameters'])
    command += ['-p', f'results_dir:={tmp_path}',
                '-p', f'octomap_path:={tmp_path / "map.bt"}', '-p', 'planning_period:=1',
                '-p', 'nbv_filter_logging:=false']
    with (tmp_path / 'node.log').open('w') as log:
        process = subprocess.Popen(command, env=env, stdout=log, stderr=log)
        rclpy.init()
        node = rclpy.create_node('migration_contract_probe')
        try:
            received = []
            subscription = node.create_subscription(Bool, '/movement/finished_execution', lambda msg: received.append(msg.data), 10)
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            odom = node.create_publisher(Odometry, 'holocean/odom', qos)
            mag = node.create_publisher(MagneticField, 'holocean/mag', qos)
            rgb = node.create_publisher(Image, 'holocean/cameraDown/image_raw', qos)
            depth = node.create_publisher(Image, 'holocean/depth/image_raw', qos)
            client = AsyncParameterClient(node, '/planner_node')
            assert client.wait_for_services(timeout_sec=15), (tmp_path / 'node.log').read_text()
            defaults = run(dict(op='defaults'))
            future = client.get_parameters(list(defaults))
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            assert future.result() is not None
            from rclpy.parameter import parameter_value_to_python
            actual = {k: parameter_value_to_python(v) for k, v in zip(defaults, future.result().values)}
            expected = {**defaults, **{k: v for k, v in overrides.items() if k in defaults}, 'results_dir': str(tmp_path), 'octomap_path': str(tmp_path / 'map.bt'),
                        'planning_period': 1, 'nbv_filter_logging': False}
            assert actual == expected
            expected_topics = {
                '/holocean/odom': ('nav_msgs/msg/Odometry', 1, ReliabilityPolicy.BEST_EFFORT),
                '/holocean/mag': ('sensor_msgs/msg/MagneticField', 1, ReliabilityPolicy.BEST_EFFORT),
                '/holocean/depth/image_raw': ('sensor_msgs/msg/Image', 10, ReliabilityPolicy.BEST_EFFORT),
                '/holocean/cameraDown/image_raw': ('sensor_msgs/msg/Image', 10, ReliabilityPolicy.BEST_EFFORT),
                '/planner/request_nbv': ('std_msgs/msg/Bool', 1, ReliabilityPolicy.RELIABLE),
            }
            deadline = time.monotonic() + 30
            while not received and time.monotonic() < deadline:
                assert process.poll() is None, (tmp_path / 'node.log').read_text()
                stamp = node.get_clock().now().to_msg()
                o = Odometry(); o.header.stamp = stamp; o.pose.pose.position.x = 1.; o.pose.pose.orientation.w = 1.
                m = MagneticField(); m.magnetic_field.y = 1.
                image = Image(); image.header.stamp = stamp; image.height = 2; image.width = 2
                image.encoding = '32FC1'; image.step = 8; image.data = bytes(16)
                odom.publish(o); mag.publish(m); rgb.publish(image); depth.publish(image)
                rclpy.spin_once(node, timeout_sec=.1)
                time.sleep(.05)  # spin_once can return immediately; bound publisher rate.
            assert received == [True], (tmp_path / 'node.log').read_text()
            for topic, (msgtype, queue_depth, reliability) in expected_topics.items():
                infos = [x for x in node.get_subscriptions_info_by_topic(topic) if x.node_name == 'planner_node']
                assert len(infos) == 1, topic
                assert infos[0].topic_type == msgtype
                # Fast DDS graph discovery does not transmit history depth (0).
                assert infos[0].qos_profile.depth in (0, queue_depth)
                assert infos[0].qos_profile.reliability == reliability
            for topic, msgtype in [('/object/areaWaypoints', 'std_msgs/msg/Float32MultiArray'),
                                   ('/movement/finished_execution', 'std_msgs/msg/Bool')]:
                infos = [x for x in node.get_publishers_info_by_topic(topic) if x.node_name == 'planner_node']
                assert len(infos) == 1 and infos[0].topic_type == msgtype
                assert infos[0].qos_profile.depth in (0, 10)
                assert infos[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
        finally:
            node.destroy_node()
            rclpy.shutdown()
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        assert (tmp_path / 'map.bt').exists()
        assert len((tmp_path / 'evaluation_parameters.txt').read_text().splitlines()) == 3
