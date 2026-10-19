import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data

import numpy as np
import time
import traceback

from sensor_msgs.msg import Image, CameraInfo, Imu, MagneticField
from std_msgs.msg import Float32, Header, Float32MultiArray
from geometry_msgs.msg import TwistStamped, Vector3, Quaternion, Pose, Point
from nav_msgs.msg import Odometry

# cv_bridge for numpy <-> ROS Image
from cv_bridge import CvBridge

# Helper quaternion conversion
import math
import json

from .domain.simulation import Command
from .use_cases.current_field import vortex_field
from .use_cases.simulation import SimulationRunner
from .adapters.holoocean import HoloOceanSessions, load_scenario
from .adapters.sensor_dispatch import SensorDispatch
from .adapters.ros_sensors import RosSensorOutputs

def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) - math.cos(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
    qy = math.cos(roll/2)*math.sin(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.cos(pitch/2)*math.sin(yaw/2)
    qz = math.cos(roll/2)*math.cos(pitch/2)*math.sin(yaw/2) - math.sin(roll/2)*math.sin(pitch/2)*math.cos(yaw/2)
    qw = math.cos(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
    return qx, qy, qz, qw


class HoloOceanROS2Bridge(Node):
    def __init__(self, holoocean_make_fn, step_limit=None):
        """
        holoocean_make_fn: function holoocean.make
        scenario_cfg: config object/str passed to holoocean.make
        command: default command to pass to env.step(command). If None, pass {}.
        step_limit: optional max steps to run (None => run until env finishes or KeyboardInterrupt)
        """
        super().__init__('holocean_ros2_bridge')

        # 1. Declare and get the parameter passed from the launch file
        self.runner = None
        self.declare_parameter('holoocean_config_path', '')
        config_path = self.get_parameter('holoocean_config_path').value

        # 2. Read the JSON file into a dictionary
        if not config_path:
            self.get_logger().error("No holoocean config path provided!")
            return

        scenario_cfg = load_scenario(config_path)

        self.get_logger().info(f"Loaded HoloOcean Config for world: {scenario_cfg['world']}")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        # qos = qos_profile_sensor_data

        # Publishers
        self.camera_pub = self.create_publisher(Image, 'holocean/camera/image_raw', qos)
        self.cameraDown_pub = self.create_publisher(Image, 'holocean/cameraDown/image_raw', qos)
        self.cam_info_pub = self.create_publisher(CameraInfo, 'holocean/camera/camera_info', qos)
        self.depth_image_pub = self.create_publisher(Image, 'holocean/depth/image_raw', qos)
        self.depth_scalar_pub = self.create_publisher(Float32, 'holocean/depth/scalar', qos)
        self.range_finder_pub = self.create_publisher(Float32, 'holocean/distance/scalar', qos)
        self.depth_distance_pub = self.create_publisher(Float32, 'holocean/depth/distance', qos)
        self.imu_pub = self.create_publisher(Imu, 'holocean/imu', qos)
        self.mag_pub = self.create_publisher(MagneticField, 'holocean/mag', qos)
        self.dvl_pub = self.create_publisher(TwistStamped, 'holocean/dvl', qos)
        self.odom_pub = self.create_publisher(Odometry, 'holocean/odom', qos)

        # Bridge and config
        self.bridge = CvBridge()
        self.holoocean_make_fn = holoocean_make_fn
        self.scenario_cfg = scenario_cfg
        self.step_limit = step_limit

        # CameraInfo default (override if you have calibration YAML)
        self.default_cam_info = CameraInfo()
        # fill some reasonable defaults (width/height will be set per-image)
        self.default_cam_info.distortion_model = 'plumb_bob'
        self.default_cam_info.k = [0.0]*9
        self.default_cam_info.d = [0.0]*5
        self.default_cam_info.r = [0.0]*9
        self.default_cam_info.p = [0.0]*12
        self.default_cam_info.binning_x = 0
        self.default_cam_info.binning_y = 0



        self.command = [10,10,10,10,0,0,0,0]  # default command
        self.sub_cmd = self.create_subscription(
            Float32MultiArray,
            '/holo/cmd',
            self.cmd_callback,
            10
        )


        outputs = RosSensorOutputs(
            {name: getattr(self, name) for name in (
                'camera_pub', 'cameraDown_pub', 'cam_info_pub', 'depth_image_pub',
                'depth_scalar_pub', 'range_finder_pub', 'depth_distance_pub',
                'imu_pub', 'mag_pub', 'dvl_pub', 'odom_pub')},
            self.bridge, self.get_clock(), self.get_logger(), self.default_cam_info)
        self.runner = SimulationRunner(
            HoloOceanSessions(holoocean_make_fn, scenario_cfg),
            SensorDispatch(outputs, self.get_logger(), vortex_field, agent='auv0'),
            lambda: Command(tuple(self.command)),
            lambda: rclpy.spin_once(self, timeout_sec=0.0), self.get_logger(), step_limit)

    def run(self):
        if self.runner is None:
            return
        self.get_logger().info('Starting HoloOcean bridge loop...')
        try:
            self.runner.run()
        except Exception as e:
            self.get_logger().error('Exception in holoocean loop: %s' % str(e))
            self.get_logger().error(traceback.format_exc())
            raise

    def cmd_callback(self, msg):
        self.command = list(msg.data)
        print("Received command:", self.command)



def main(args=None):
    import holoocean  # ensure holoocean is available
    rclpy.init(args=args)

    bridge_node = HoloOceanROS2Bridge(holoocean.make,  step_limit=None)

    bridge_node.run()

    try:
        # Keep the node alive after the explicit simulation run returns.
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        bridge_node.get_logger().info("Shutting down via KeyboardInterrupt")
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
