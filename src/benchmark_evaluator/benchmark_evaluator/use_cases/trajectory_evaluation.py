import math
from auv_contracts.ports import TrajectorySink, Clock, Diagnostics
from ..domain.evaluation import ReportWriter, ReportSnapshot

class TrajectoryEvaluation:
    def __init__(self, telemetry: TrajectorySink, writer: ReportWriter,
                 clock: Clock, diagnostics: Diagnostics, show_reference_objects=True):
        self.telemetry = telemetry
        self.writer = writer
        self.clock = clock
        self.diagnostics = diagnostics
        self.show_reference_objects = show_reference_objects
        # State variables
        self.mission_started = False
        self.mission_finished = False
        self.start_time = 0.0
        self.end_time = 0.0
        self.total_distance = 0.0
        self.data_saved = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        self.trajectory_log = [] # Stores (timestamp, x, y, z, distance_from_start)
        self.diagnostic_log = []
        self.planned_trajectory = []

        self.diagnostics.info("📊 Benchmark Logger Initialized. Waiting for Odometry...")

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
        if not self.mission_started and (abs(msg.position.x) > 0.01):
            self.mission_started = True
            self.start_time = self.clock.time()
            self.diagnostics.info("🚀 Mission Started! Timer running.")

        self.current_x = msg.position.x
        self.current_y = msg.position.y
        self.current_z = msg.position.z
        orientation = msg.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y)
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z)
        self.current_yaw = math.atan2(sin_yaw, cos_yaw)


    def log_trajectory_callback(self):
        if not self.mission_started or self.mission_finished:
            return

        current_time = self.clock.time() - self.start_time

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
                self.start_time = self.clock.time() # Handle immediate aborts

            self.end_time = self.clock.time()
            total_time = self.end_time - self.start_time

            self.diagnostics.info(f"🛑 Mission Finished! Total Time: {total_time:.2f} s")
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
        snapshot = ReportSnapshot(tuple(self.trajectory_log), tuple(self.diagnostic_log),
                                  self.total_distance, self._diagnostic_metrics(),
                                  self.show_reference_objects)
        self.writer.write(snapshot, total_time)
        self.data_saved = True

    def close(self):
        """Save the partial trajectory when shutdown is requested."""
        if not self.data_saved:
            end_time = self.clock.time()
            total_time = (
                end_time - self.start_time if self.start_time > 0.0 else 0.0)
            self.diagnostics.info(
                'Shutdown requested; saving the current trajectory and plot.')
            self.save_benchmark_data(total_time)

