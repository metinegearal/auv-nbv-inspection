import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray
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
        self.data_saved = False
        self.results_dir = self.declare_parameter(
            'results_dir', 'results').value
        self.telemetry = MidrunLogger(
            self.results_dir, trajectory_only=True)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        
        self.trajectory_log = [] # Stores (timestamp, x, y, z, distance_from_start)
        self.diagnostic_log = []
        self.planned_trajectory = []
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'holocean/odom', self.odom_callback, 10)
        self.planned_trajectory_sub = self.create_subscription(
            Float32MultiArray,
            'object/areaWaypoints',
            self.planned_trajectory_callback,
            10)
        
        self.finish_sub = self.create_subscription(
            Bool, '/movement/finished_execution', self.finish_callback, 10)
            
        # 0.5 second logging timer
        self.timer = self.create_timer(0.5, self.log_trajectory_callback)
        
        self.get_logger().info("📊 Benchmark Logger Initialized. Waiting for Odometry...")

    @staticmethod
    def _wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def planned_trajectory_callback(self, msg):
        """Keep the latest planner trajectory, encoded as flattened [x,y,z,yaw]."""
        if len(msg.data) < 4 or len(msg.data) % 4 != 0:
            return
        self.planned_trajectory = [
            tuple(float(value) for value in msg.data[index:index + 4])
            for index in range(0, len(msg.data), 4)
        ]

    def _nearest_planned_point(self):
        if not self.planned_trajectory:
            return None
        return min(
            self.planned_trajectory,
            key=lambda point: (
                (point[0] - self.current_x) ** 2
                + (point[1] - self.current_y) ** 2
                + (point[2] - self.current_z) ** 2))

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
        orientation = msg.pose.pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y)
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z)
        self.current_yaw = math.atan2(sin_yaw, cos_yaw)

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
            elapsed = max(current_time - last_pt[0], 1e-6)
            speed = segment_dist / elapsed
            horizontal_speed = math.hypot(dx, dy) / elapsed
            vertical_speed = dz / elapsed
        else:
            speed = 0.0
            horizontal_speed = 0.0
            vertical_speed = 0.0

        planned_point = self._nearest_planned_point()
        if planned_point is None:
            heading_error_deg = None
            planned_x = planned_y = planned_z = planned_yaw = None
        else:
            planned_x, planned_y, planned_z, planned_yaw = planned_point
            heading_error_deg = math.degrees(
                self._wrap_angle(planned_yaw - self.current_yaw))
            
        self.trajectory_log.append((
            current_time, self.current_x, self.current_y, self.current_z, self.total_distance
        ))
        self.telemetry.record_trajectory(
            current_time,
            (self.current_x, self.current_y, self.current_z),
            self.total_distance)
        self.diagnostic_log.append({
            'time_s': current_time,
            'speed_mps': speed,
            'horizontal_speed_mps': horizontal_speed,
            'vertical_speed_mps': vertical_speed,
            'heading_error_deg': heading_error_deg,
            'x_m': self.current_x,
            'y_m': self.current_y,
            'z_m': self.current_z,
            'planned_x_m': planned_x,
            'planned_y_m': planned_y,
            'planned_z_m': planned_z,
            'planned_yaw_rad': planned_yaw,
        })

    def finish_callback(self, msg):
        if msg.data and not self.mission_finished:
            self.mission_finished = True
            if self.start_time == 0.0:
                self.start_time = time.time() # Handle immediate aborts
                
            self.end_time = time.time()
            total_time = self.end_time - self.start_time
            
            self.get_logger().info(f"🛑 Mission Finished! Total Time: {total_time:.2f} s")
            self.save_benchmark_data(total_time)

    def _trajectory_rmse(self):
        errors = []
        for sample in self.diagnostic_log:
            if sample['planned_x_m'] is None:
                continue
            errors.append(
                (sample['x_m'] - sample['planned_x_m']) ** 2
                + (sample['y_m'] - sample['planned_y_m']) ** 2
                + (sample['z_m'] - sample['planned_z_m']) ** 2)
        if not errors:
            return None
        return math.sqrt(sum(errors) / len(errors))

    def _save_diagnostics(self, results_path, timestamp_str):
        diagnostics_path = results_path / f"trajectory_diagnostics_{timestamp_str}.csv"
        with diagnostics_path.open('w', newline='') as stream:
            fieldnames = [
                'Time_s', 'Speed_mps', 'Horizontal_Speed_mps',
                'Vertical_Speed_mps', 'Heading_Error_deg',
                'X_m', 'Y_m', 'Z_m', 'Planned_X_m', 'Planned_Y_m',
                'Planned_Z_m', 'Planned_Yaw_rad']
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            for sample in self.diagnostic_log:
                writer.writerow({
                    'Time_s': f"{sample['time_s']:.3f}",
                    'Speed_mps': f"{sample['speed_mps']:.5f}",
                    'Horizontal_Speed_mps': f"{sample['horizontal_speed_mps']:.5f}",
                    'Vertical_Speed_mps': f"{sample['vertical_speed_mps']:.5f}",
                    'Heading_Error_deg': '' if sample['heading_error_deg'] is None
                    else f"{sample['heading_error_deg']:.3f}",
                    'X_m': f"{sample['x_m']:.5f}",
                    'Y_m': f"{sample['y_m']:.5f}",
                    'Z_m': f"{sample['z_m']:.5f}",
                    'Planned_X_m': '' if sample['planned_x_m'] is None
                    else f"{sample['planned_x_m']:.5f}",
                    'Planned_Y_m': '' if sample['planned_y_m'] is None
                    else f"{sample['planned_y_m']:.5f}",
                    'Planned_Z_m': '' if sample['planned_z_m'] is None
                    else f"{sample['planned_z_m']:.5f}",
                    'Planned_Yaw_rad': '' if sample['planned_yaw_rad'] is None
                    else f"{sample['planned_yaw_rad']:.5f}",
                })

        try:
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().warning('Matplotlib unavailable; skipping diagnostic plots.')
            return self._diagnostic_metrics(), diagnostics_path

        times = [sample['time_s'] for sample in self.diagnostic_log]
        speeds = [sample['speed_mps'] for sample in self.diagnostic_log]
        horizontal_speeds = [
            sample['horizontal_speed_mps'] for sample in self.diagnostic_log]
        vertical_speeds = [
            sample['vertical_speed_mps'] for sample in self.diagnostic_log]
        heading_samples = [
            sample for sample in self.diagnostic_log
            if sample['heading_error_deg'] is not None]

        figure, axis = plt.subplots(figsize=(11, 6), constrained_layout=True)
        axis.plot(times, speeds, label='3D speed', color='#1d4ed8')
        axis.plot(times, horizontal_speeds, label='Horizontal speed', color='#16a34a')
        axis.plot(times, vertical_speeds, label='Vertical speed', color='#dc2626')
        axis.set_title('Momentum protection: speed over time')
        axis.set_xlabel('Time (s)')
        axis.set_ylabel('Speed (m/s)')
        axis.grid(True, alpha=0.25)
        axis.legend()
        speed_plot_path = results_path / 'momentum_protection_speed_time.png'
        figure.savefig(speed_plot_path, dpi=160)
        plt.close(figure)

        if heading_samples:
            figure, axis = plt.subplots(figsize=(11, 6), constrained_layout=True)
            axis.plot(
                [sample['time_s'] for sample in heading_samples],
                [sample['heading_error_deg'] for sample in heading_samples],
                color='#7c3aed',
                label='Heading error',
            )
            axis.axhline(0.0, color='#374151', linewidth=0.8)
            axis.set_title('Heading error over time')
            axis.set_xlabel('Time (s)')
            axis.set_ylabel('Heading error (deg)')
            axis.grid(True, alpha=0.25)
            axis.legend()
            heading_plot_path = results_path / 'heading_error_time.png'
            figure.savefig(heading_plot_path, dpi=160)
            plt.close(figure)

        metrics = self._diagnostic_metrics()
        if metrics['trajectory_rmse_m'] is not None:
            planned_samples = [
                sample for sample in self.diagnostic_log
                if sample['planned_x_m'] is not None]
            figure, axis = plt.subplots(figsize=(11, 7), constrained_layout=True)
            axis.plot(
                [sample['planned_x_m'] for sample in planned_samples],
                [sample['planned_y_m'] for sample in planned_samples],
                '--', color='#2563eb', label='Planned path')
            axis.plot(
                [sample['x_m'] for sample in planned_samples],
                [sample['y_m'] for sample in planned_samples],
                color='#111827', label='Real path')
            axis.set_title(
                f"Planned vs real trajectory (RMSE: "
                f"{metrics['trajectory_rmse_m']:.3f} m)")
            axis.set_xlabel('X (m)')
            axis.set_ylabel('Y (m)')
            axis.set_aspect('equal', adjustable='datalim')
            axis.grid(True, alpha=0.25)
            axis.legend()
            figure.savefig(results_path / 'planned_vs_real_trajectory.png', dpi=160)
            plt.close(figure)

        return metrics, diagnostics_path

    def _diagnostic_metrics(self):
        heading_errors = [
            abs(sample['heading_error_deg'])
            for sample in self.diagnostic_log
            if sample['heading_error_deg'] is not None]
        speeds = [sample['speed_mps'] for sample in self.diagnostic_log]
        trajectory_rmse = self._trajectory_rmse()
        return {
            'peak_speed_mps': max(speeds, default=0.0),
            'mean_abs_heading_error_deg': (
                sum(heading_errors) / len(heading_errors)
                if heading_errors else None),
            'max_abs_heading_error_deg': max(heading_errors, default=None),
            'trajectory_rmse_m': trajectory_rmse,
        }

    def save_benchmark_data(self, total_time):
        if self.data_saved:
            return

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
            metrics, diagnostics_path = self._save_diagnostics(
                results_path, timestamp_str)
            f.write(f"Peak Speed: {metrics['peak_speed_mps']:.3f} m/s\n")
            if metrics['mean_abs_heading_error_deg'] is None:
                f.write("Mean Absolute Heading Error: N/A\n")
                f.write("Maximum Absolute Heading Error: N/A\n")
            else:
                f.write(
                    "Mean Absolute Heading Error: "
                    f"{metrics['mean_abs_heading_error_deg']:.3f} deg\n")
                f.write(
                    "Maximum Absolute Heading Error: "
                    f"{metrics['max_abs_heading_error_deg']:.3f} deg\n")
            if metrics['trajectory_rmse_m'] is None:
                f.write("Planned vs Real Trajectory RMSE: N/A\n")
            else:
                f.write(
                    "Planned vs Real Trajectory RMSE: "
                    f"{metrics['trajectory_rmse_m']:.3f} m\n")
            
        self.get_logger().info(
            f"💾 Saved {csv_filename}, {diagnostics_path}, and {txt_filename}")
        plot_path = generate_results_plot(self.results_dir)
        if plot_path is not None:
            self.get_logger().info(f'Saved mid-run overview plot to {plot_path}')
        self.data_saved = True

    def destroy_node(self):
        """Save the partial trajectory when shutdown is requested."""
        if not self.data_saved:
            end_time = time.time()
            total_time = (
                end_time - self.start_time if self.start_time > 0.0 else 0.0)
            self.get_logger().info(
                'Shutdown requested; saving the current trajectory and plot.')
            self.save_benchmark_data(total_time)
        super().destroy_node()

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