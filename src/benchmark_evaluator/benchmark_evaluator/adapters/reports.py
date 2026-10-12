import csv
from datetime import datetime
from pathlib import Path
from ..plot_adapter import generate_results_plot
from auv_contracts.mission_results import MISSION_STATS_FILENAME, distance_record
from .response_reports import write_motion_response

class FileReportWriter:
    def __init__(self, results_dir, diagnostics):
        self.results_dir = results_dir
        self.results_path = Path(results_dir).expanduser()
        self.diagnostics = diagnostics

    def write(self, snapshot, total_time):

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        results_path = self.results_path
        results_path.mkdir(parents=True, exist_ok=True)
        write_motion_response(results_path, snapshot, self.diagnostics)
        csv_filename = results_path / f"trajectory_benchmark_{timestamp_str}.csv"
        # txt_filename = f"mission_stats_{timestamp_str}.txt"
        txt_filename = results_path / MISSION_STATS_FILENAME

        # 1. Save Trajectory CSV
        with open(csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Time_s', 'X_m', 'Y_m', 'Z_m', 'Cumulative_Distance_m'])
            for row in snapshot.trajectory_log:
                writer.writerow([f"{row[0]:.2f}", f"{row[1]:.3f}", f"{row[2]:.3f}", f"{row[3]:.3f}", f"{row[4]:.3f}"])

        # 2. Save Academic Stats Summary
        with open(txt_filename, 'w') as f:
            f.write("=== AUV Exploration Benchmark ===\n")
            f.write(f"Total Mission Time (Makespan): {total_time:.2f} seconds\n")
            f.write(distance_record(snapshot.total_distance))
            if total_time > 0:
                f.write(f"Average Flight Speed: {(snapshot.total_distance / total_time):.3f} m/s\n")
            f.write(f"Total Trajectory Waypoints Logged: {len(snapshot.trajectory_log)}\n")
            metrics, diagnostics_path = self._save_diagnostics(
                results_path, timestamp_str, snapshot)
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

        self.diagnostics.info(
            f"💾 Saved {csv_filename}, {diagnostics_path}, and {txt_filename}")
        plot_path = generate_results_plot(
            self.results_dir, show_reference_objects=snapshot.show_reference_objects)
        if plot_path is not None:
            self.diagnostics.info(f'Saved mid-run overview plot to {plot_path}')


    def _save_diagnostics(self, results_path, timestamp_str, snapshot):
        diagnostics_path = results_path / f"trajectory_diagnostics_{timestamp_str}.csv"
        with diagnostics_path.open('w', newline='') as stream:
            fieldnames = [
                'Time_s', 'Speed_mps', 'Horizontal_Speed_mps',
                'Vertical_Speed_mps', 'Heading_Error_deg',
                'X_m', 'Y_m', 'Z_m', 'Planned_X_m', 'Planned_Y_m',
                'Planned_Z_m', 'Planned_Yaw_rad']
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            for sample in snapshot.diagnostic_log:
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
            self.diagnostics.warning('Matplotlib unavailable; skipping diagnostic plots.')
            return snapshot.metrics, diagnostics_path

        times = [sample['time_s'] for sample in snapshot.diagnostic_log]
        speeds = [sample['speed_mps'] for sample in snapshot.diagnostic_log]
        horizontal_speeds = [
            sample['horizontal_speed_mps'] for sample in snapshot.diagnostic_log]
        vertical_speeds = [
            sample['vertical_speed_mps'] for sample in snapshot.diagnostic_log]
        heading_samples = [
            sample for sample in snapshot.diagnostic_log
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

        metrics = snapshot.metrics
        if metrics['trajectory_rmse_m'] is not None:
            planned_samples = [
                sample for sample in snapshot.diagnostic_log
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
