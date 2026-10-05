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
        self.evaluation = TrajectoryEvaluation(
            MidrunLogger(results_dir, trajectory_only=True),
            FileReportWriter(results_dir, self.get_logger()), time, self.get_logger(), show)
        self.odom_sub = self.create_subscription(Odometry, 'holocean/odom', self.odom_callback, 10)
        self.planned_trajectory_sub = self.create_subscription(
            Float32MultiArray, 'object/areaWaypoints', self.planned_trajectory_callback, 10)
        self.finish_sub = self.create_subscription(
            Bool, '/movement/finished_execution', self.finish_callback, 10)
        self.timer = self.create_timer(0.5, self.evaluation.log_trajectory_callback)
    def odom_callback(self, msg):
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.header.stamp
        self.evaluation.odom_callback(PoseSample(
            Vector3(p.x, p.y, p.z), Quaternion(q.x, q.y, q.z, q.w), Stamp(t.sec, t.nanosec)))
    def planned_trajectory_callback(self, msg):
        self.evaluation.planned_trajectory_callback(ArrayEvent(tuple(msg.data)))
    def finish_callback(self, msg):
        self.evaluation.finish_callback(Flag(msg.data))
    def destroy_node(self):
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
