import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
import csv
import time
from datetime import datetime

from .midrun_logger import MidrunLogger, generate_results_plot

class BenchmarkLogger(Node):
    def __init__(self):
        super().__init__('benchmark_logger')
        
        # State variables
        self.mission_started = False
        self.mission_finished = False
        self.start_time = 0.0
        self.end_time = 0.0
        self.total_distance = 0.0
        self.results_dir = self.declare_parameter(
            'results_dir', 'results').value
        self.telemetry = MidrunLogger(
            self.results_dir, trajectory_only=True)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        self.trajectory_log = [] # Stores (timestamp, x, y, z, distance_from_start)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'holocean/odom', self.odom_callback, 10)
        
        self.finish_sub = self.create_subscription(
            Bool, '/movement/finished_execution', self.finish_callback, 10)
            
        # 0.5 second logging timer
        self.timer = self.create_timer(0.5, self.log_trajectory_callback)
        
        self.get_logger().info("📊 Benchmark Logger Initialized. Waiting for Odometry...")

    def odom_callback(self, msg):
        if self.mission_finished:
            return
            
        # Check if this is the first valid odometry message (not 0,0,0)
        if not self.mission_started and (abs(msg.pose.pose.position.x) > 0.01):
            self.mission_started = True
            self.start_time = time.time()
            self.get_logger().info("🚀 Mission Started! Timer running.")
            
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z

    def log_trajectory_callback(self):
        if not self.mission_started or self.mission_finished:
            return
            
        current_time = time.time() - self.start_time
        
        # Calculate distance traveled since last log
        if len(self.trajectory_log) > 0:
            last_pt = self.trajectory_log[-1]
            dx = self.current_x - last_pt[1]
            dy = self.current_y - last_pt[2]
            dz = self.current_z - last_pt[3]
            segment_dist = math.sqrt(dx**2 + dy**2 + dz**2)
            self.total_distance += segment_dist
            
        self.trajectory_log.append((
            current_time, self.current_x, self.current_y, self.current_z, self.total_distance
        ))
        self.telemetry.record_trajectory(
            current_time,
            (self.current_x, self.current_y, self.current_z),
            self.total_distance)

    def finish_callback(self, msg):
        if msg.data and not self.mission_finished:
            self.mission_finished = True
            if self.start_time == 0.0:
                self.start_time = time.time() # Handle immediate aborts
                
            self.end_time = time.time()
            total_time = self.end_time - self.start_time
            
            self.get_logger().info(f"🛑 Mission Finished! Total Time: {total_time:.2f} s")
            self.save_benchmark_data(total_time)

    def save_benchmark_data(self, total_time):
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        results_path = self.telemetry.results_dir
        csv_filename = results_path / f"trajectory_benchmark_{timestamp_str}.csv"
        # txt_filename = f"mission_stats_{timestamp_str}.txt"
        txt_filename = results_path / "optuna_mission_stats.txt"
        
        # 1. Save Trajectory CSV
        with open(csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Time_s', 'X_m', 'Y_m', 'Z_m', 'Cumulative_Distance_m'])
            for row in self.trajectory_log:
                writer.writerow([f"{row[0]:.2f}", f"{row[1]:.3f}", f"{row[2]:.3f}", f"{row[3]:.3f}", f"{row[4]:.3f}"])
                
        # 2. Save Academic Stats Summary
        with open(txt_filename, 'w') as f:
            f.write("=== AUV Exploration Benchmark ===\n")
            f.write(f"Total Mission Time (Makespan): {total_time:.2f} seconds\n")
            f.write(f"Total Distance Traveled: {self.total_distance:.2f} meters\n")
            if total_time > 0:
                f.write(f"Average Flight Speed: {(self.total_distance / total_time):.3f} m/s\n")
            f.write(f"Total Trajectory Waypoints Logged: {len(self.trajectory_log)}\n")
            
        self.get_logger().info(f"💾 Saved {csv_filename} and {txt_filename}")
        plot_path = generate_results_plot(self.results_dir)
        if plot_path is not None:
            self.get_logger().info(f'Saved mid-run overview plot to {plot_path}')

def main(args=None):
    rclpy.init(args=args)
    node = BenchmarkLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()