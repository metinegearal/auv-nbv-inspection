import time
from functools import partial
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, MagneticField
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray
from auv_contracts.values import PoseSample, Vector3, Quaternion, Stamp, Flag
from telemetry_adapters.file_sinks import PlannerFileSink
from .domain.configuration import load_configuration
from .use_cases.mission import InspectionMission, body_rotation, vehicle_viewpoint
from .use_cases.nbv import calculate_nbv_waypoints
from .use_cases.trajectory_optimization import optimize_trajectory
from .adapters.point_cloud import Open3DPreprocessor
from .adapters.optimization import ScipySolver
from .adapters.octomap import OctomapAdapter
from .adapters.files import open_evaluation_log
from .adapters.ros_output import RosTrajectoryOutput, RosCompletionOutput

class NBVUtilityNode(Node):
    """ROS input/output adapter; mission decisions belong to InspectionMission."""
    def __init__(self):
        super().__init__('nbv_utility_node')
        config = load_configuration(lambda name, default:
                                    self.declare_parameter(name, default).value)
        self.bridge = CvBridge()
        self.state_callbacks = MutuallyExclusiveCallbackGroup()
        occupancy = OctomapAdapter(config.resolution)
        telemetry = PlannerFileSink(config.results_dir, prefix='planner')
        output = RosTrajectoryOutput(self.create_publisher(
            Float32MultiArray, 'object/areaWaypoints', 10))
        completion = RosCompletionOutput(self.create_publisher(
            Bool, '/movement/finished_execution', 10))
        self.mission = InspectionMission(
            config, occupancy, telemetry, open_evaluation_log(config.results_dir),
            self.get_logger(), time, output, completion,
            partial(calculate_nbv_waypoints, preprocessor=Open3DPreprocessor(),
                    diagnostics=rclpy.logging.get_logger('nbv_waypoint_calculator'), clock=time),
            partial(optimize_trajectory, solver=ScipySolver()), telemetry.description)
        qos = rclpy.qos.QoSProfile(depth=10)
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Image, 'holocean/depth/image_raw', self.depth_callback, qos)
        self.create_subscription(Image, 'holocean/cameraDown/image_raw', self.rgb_callback, qos)
        state_qos = rclpy.qos.QoSProfile(depth=1)
        state_qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Odometry, 'holocean/odom', self.odom_callback,
                                 state_qos, callback_group=self.state_callbacks)
        self.create_subscription(MagneticField, 'holocean/mag', self.mag_callback,
                                 state_qos, callback_group=self.state_callbacks)
        self.create_subscription(Bool, '/planner/request_nbv', self.request_nbv_callback, 1)
        self.timer = self.create_timer(0.5, self.mission.timer_callback)

    def odom_callback(self, msg):
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.header.stamp
        self.mission.odom_callback(PoseSample(
            Vector3(p.x, p.y, p.z), Quaternion(q.x, q.y, q.z, q.w), Stamp(t.sec, t.nanosec)))
    def mag_callback(self, msg):
        v = msg.magnetic_field
        self.mission.mag_callback(Vector3(v.x, v.y, v.z))
    def rgb_callback(self, msg):
        self.mission.rgb_callback(None)
    def request_nbv_callback(self, msg):
        self.mission.request_nbv_callback(Flag(msg.data))
    def depth_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        t = msg.header.stamp
        self.mission.depth_callback(frame, Stamp(t.sec, t.nanosec))
    def destroy_node(self):
        self.mission.close()
        return super().destroy_node()

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
