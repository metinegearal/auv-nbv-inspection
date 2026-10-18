import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray
from auv_contracts.values import PoseSample, Vector3, Quaternion, Stamp, ArrayEvent, Flag
from telemetry_adapters.file_sinks import MidrunLogger
from .adapters.reports import FileReportWriter
from .use_cases.trajectory_evaluation import TrajectoryEvaluation

class BenchmarkLogger(Node):
    def __init__(self):
        super().__init__('benchmark_logger')
        results_dir = self.declare_parameter('results_dir', 'results').value
        show = self.declare_parameter('show_reference_objects', True).value
        response_options = {
            'band_fraction': self.declare_parameter('response_band_fraction', 0.02).value,
            'dwell_s': self.declare_parameter('response_settling_dwell_s', 1.0).value,
            'target_tolerance_m': self.declare_parameter('response_target_tolerance_m', 1e-5).value,
            'max_samples': self.declare_parameter('response_max_samples', 3000).value,
            'log_period_s': self.declare_parameter('response_log_period_s', 1.0).value,
        }
        reports = FileReportWriter(results_dir, self.get_logger())
        self.permanent_bans = reports.permanent_bans
        self.evaluation = TrajectoryEvaluation(
            MidrunLogger(results_dir, trajectory_only=True),
            reports, time, self.get_logger(), show,
            response_options)
        self.odom_sub = self.create_subscription(Odometry, 'holocean/odom', self.odom_callback, 10)
        self.planned_trajectory_sub = self.create_subscription(
            Float32MultiArray, 'object/areaWaypoints', self.planned_trajectory_callback, 10)
        self.finish_sub = self.create_subscription(
            Bool, '/movement/finished_execution', self.finish_callback, 10)
        self.waypoint_sub = self.create_subscription(
            Float32MultiArray, '/trajectory/waypoint', self.waypoint_callback, 10)
        self.timer = self.create_timer(0.5, self.evaluation.log_trajectory_callback)
        self.ban_timer = self.create_timer(1.0, self.save_permanent_bans)
    def save_permanent_bans(self):
        try:
            self.permanent_bans.poll()
        except OSError as error:
            self.get_logger().warning(f'Could not save permanent ban spheres: {error}')
    def odom_callback(self, msg):
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.header.stamp
        self.evaluation.odom_callback(PoseSample(
            Vector3(p.x, p.y, p.z), Quaternion(q.x, q.y, q.z, q.w), Stamp(t.sec, t.nanosec)))
    def planned_trajectory_callback(self, msg):
        self.evaluation.planned_trajectory_callback(ArrayEvent(tuple(msg.data)))
    def finish_callback(self, msg):
        self.evaluation.finish_callback(Flag(msg.data))
    def waypoint_callback(self, msg):
        self.evaluation.motion_response.waypoint_callback(ArrayEvent(tuple(msg.data)))
    def destroy_node(self):
        self.save_permanent_bans()
        self.evaluation.close()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BenchmarkLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
