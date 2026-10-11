import time
from threading import Lock
from functools import partial
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.task import Future
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion as RosQuaternion
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from nbv_interfaces.action import ComputeNBV
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, MagneticField
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from auv_contracts.values import PoseSample, Vector3, Quaternion, Stamp
from telemetry_adapters.file_sinks import PlannerFileSink
from .domain.configuration import load_configuration
from .use_cases.mission import InspectionMission
from .use_cases.nbv import calculate_nbv_waypoints
from .use_cases.trajectory_optimization import optimize_trajectory
from .adapters.point_cloud import Open3DPreprocessor
from .adapters.optimization import ScipySolver
from .adapters.octomap import OctomapAdapter
from .adapters.files import open_evaluation_log
from .adapters.ros_output import RosTrajectoryOutput
from .adapters.transforms import TfCameraTransforms

class NBVUtilityNode(Node):
    """ROS input/output adapter; mission decisions belong to InspectionMission."""
    def __init__(self):
        super().__init__('nbv_utility_node')
        config = load_configuration(lambda name, default:
                                    self.declare_parameter(name, default).value)
        self.bridge = CvBridge()
        self.state_callbacks = MutuallyExclusiveCallbackGroup()
        self.world_frame = self.declare_parameter('world_frame', 'map').value
        self.body_frame = self.declare_parameter('body_frame', 'base_link').value
        self.camera_frame = self.declare_parameter('camera_frame', 'camera_link').value
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # HoloOcean reports global poses but does not publish TF. Disable this
        # adapter when localization/robot_state_publisher owns the vehicle TF.
        self.tf_broadcaster = (TransformBroadcaster(self) if self.declare_parameter(
            'publish_vehicle_tf', False).value else None)
        transforms = TfCameraTransforms(
            self.tf_buffer, self.world_frame, self.camera_frame, self.body_frame)
        self._goal_lock = Lock()
        self._goal_reserved = False
        self._goal_handle = None
        self._goal_result = None
        self._sim_heading = None
        occupancy = OctomapAdapter(config.resolution)
        telemetry = PlannerFileSink(config.results_dir, prefix='planner')
        output = RosTrajectoryOutput(self.create_publisher(
            Float32MultiArray, 'object/areaWaypoints', 10))
        self.mission = InspectionMission(
            config, occupancy, telemetry, open_evaluation_log(config.results_dir),
            self.get_logger(), time, output,
            partial(calculate_nbv_waypoints, preprocessor=Open3DPreprocessor(),
                    diagnostics=rclpy.logging.get_logger('nbv_waypoint_calculator'), clock=time),
            partial(optimize_trajectory, solver=ScipySolver()), telemetry.description,
            transforms, self.publish_feedback)
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
        self.action_server = ActionServer(
            self, ComputeNBV, '/planner/compute_nbv', self.execute_nbv,
            goal_callback=self.goal_callback, cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.accepted_callback)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def goal_callback(self, request):
        with self._goal_lock:
            if not request.trigger or self._goal_reserved:
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def accepted_callback(self, goal_handle):
        self._goal_handle = goal_handle
        self._goal_result = Future(executor=self.executor)
        goal_handle.nbv_result = self._goal_result
        self._goal_revision = self.mission.trajectory_revision
        self.mission.planning_enabled = True
        self.mission.planning_state = 'WaitingForSensors'
        self.mission.force_replan = goal_handle.request.force_replan
        goal_handle.execute()

    async def execute_nbv(self, goal_handle):
        # Yield to the executor: depth, TF, cancellation and timer callbacks
        # remain live while this action waits for a new collision-checked route.
        return await goal_handle.nbv_result

    def _finish_goal(self, status, message):
        goal = self._goal_handle
        result = ComputeNBV.Result()
        result.frame_id = self.world_frame
        result.message = message
        result.strikes = self.mission.failed_plan_count
        result.strikeout = self.mission.finished_mapping
        if status == 'succeeded':
            result.trajectory = self.mission.active_trajectory.flatten().tolist()
            goal.succeed()
        elif status == 'canceled':
            goal.canceled()
        else:
            goal.abort()
        self.mission.planning_enabled = False
        self.mission.force_replan = False
        future = self._goal_result
        self._goal_handle = None
        with self._goal_lock:
            self._goal_reserved = False
        future.set_result(result)

    def publish_feedback(self, state=None):
        if self._goal_handle is None:
            return
        feedback = ComputeNBV.Feedback()
        feedback.state = state or self.mission.planning_state
        feedback.message = self.mission.failure_reason
        feedback.strikes = self.mission.failed_plan_count
        feedback.blacklisted_targets = len(self.mission.blacklisted_targets)
        self._goal_handle.publish_feedback(feedback)

    def timer_callback(self):
        if self._goal_handle is not None:
            if self._goal_handle.is_cancel_requested:
                self._finish_goal('canceled', 'NBV computation canceled')
                return
            if self.mission.finished_mapping:
                self._finish_goal('aborted', 'Strike 3: ' + self.mission.failure_reason)
                return
            self.publish_feedback()
        try:
            self.mission.timer_callback()
        except TransformException as error:
            self.mission.planning_state = 'WaitingForTF'
            if self._goal_handle is not None:
                self.mission.force_replan = True
            self.publish_feedback()
            self.get_logger().warning(
                f'Waiting for camera TF at depth timestamp: {error}', throttle_duration_sec=2.)
            return
        except Exception as error:
            self.get_logger().error(f'NBV pipeline failed: {error}')
            if self._goal_handle is not None:
                self._finish_goal('aborted', f'NBV pipeline failed: {error}')
            return
        if self._goal_handle is not None:
            if self.mission.finished_mapping:
                self._finish_goal('aborted', 'Strike 3: ' + self.mission.failure_reason)
            elif self.mission.trajectory_revision > self._goal_revision:
                self._finish_goal('succeeded', 'Collision-checked vehicle trajectory')

    def odom_callback(self, msg):
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.header.stamp
        # LocationSensor supplies position only. Use the simulator's heading
        # stream until a full odometry quaternion is available.
        valid_orientation = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w > 1e-12
        if not valid_orientation and self._sim_heading is not None:
            x, y, z, w = Rotation.from_euler('z', self._sim_heading).as_quat()
            q = RosQuaternion(x=float(x), y=float(y), z=float(z), w=float(w))
            valid_orientation = True
        if self.tf_broadcaster is not None and valid_orientation:
            transform = TransformStamped()
            transform.header.stamp = t
            transform.header.frame_id = msg.header.frame_id
            transform.child_frame_id = self.body_frame
            transform.transform.translation.x = p.x
            transform.transform.translation.y = p.y
            transform.transform.translation.z = p.z
            transform.transform.rotation = q
            self.tf_broadcaster.sendTransform(transform)
        self.mission.odom_callback(PoseSample(
            Vector3(p.x, p.y, p.z), Quaternion(q.x, q.y, q.z, q.w), Stamp(t.sec, t.nanosec)))
    def mag_callback(self, msg):
        v = msg.magnetic_field
        self.mission.mag_callback(Vector3(v.x, v.y, v.z))
        self._sim_heading = self.mission._motion_snapshot()[1]
    def rgb_callback(self, msg):
        self.mission.rgb_callback(None)
    def depth_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        t = msg.header.stamp
        self.mission.depth_callback(frame, Stamp(t.sec, t.nanosec))
    def destroy_node(self):
        self.action_server.destroy()
        self.tf_listener.unregister()
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
