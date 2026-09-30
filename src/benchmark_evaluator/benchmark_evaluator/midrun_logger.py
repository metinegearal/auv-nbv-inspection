"""File-backed telemetry and plots for mid-mission NBV evaluation."""

import csv
import json
import math
from datetime import datetime
from pathlib import Path


class MidrunLogger:
    """Write planner and vehicle state without adding ROS message types."""

    def __init__(self, results_dir='results', prefix='benchmark',
                 pipeline_only=False, trajectory_only=False):
        self.results_dir = Path(results_dir).expanduser()
        self.results_dir.mkdir(parents=True, exist_ok=True)
        self.prefix = prefix
        self.pipeline_only = pipeline_only
        self.trajectory_only = trajectory_only
        self.trajectory_path = self.results_dir / f'{prefix}_trajectory.csv'
        self.depth_pose_path = self.results_dir / f'{prefix}_depth_poses.csv'
        self.iteration_path = self.results_dir / f'{prefix}_nbv_iterations.jsonl'
        self.future_path = self.results_dir / f'{prefix}_future_trajectories.csv'
        self.pipeline_path = self.results_dir / f'{prefix}_pipeline_timing.csv'
        self._reset_files()

    def _reset_files(self):
        if self.pipeline_only:
            with self.pipeline_path.open('w', newline='') as stream:
                csv.writer(stream).writerow([
                    'stage', 'iteration', 'frame_id', 'duration_ms',
                    'source_stamp_s', 'details'])
            return
        with self.trajectory_path.open('w', newline='') as stream:
            csv.writer(stream).writerow(
                ['time_s', 'x_m', 'y_m', 'z_m', 'cumulative_distance_m'])
            if self.trajectory_only:
                return
        with self.depth_pose_path.open('w', newline='') as stream:
            csv.writer(stream).writerow([
                'depth_sequence', 'depth_time_s', 'x_m', 'y_m', 'z_m',
                'roll_rad', 'pitch_rad', 'yaw_rad'])
        with self.future_path.open('w', newline='') as stream:
            csv.writer(stream).writerow([
                'iteration', 'waypoint_index', 'x_m', 'y_m', 'z_m', 'yaw_rad'])
        with self.pipeline_path.open('w', newline='') as stream:
            csv.writer(stream).writerow([
                'stage', 'iteration', 'frame_id', 'duration_ms',
                'source_stamp_s', 'details'])
        self.iteration_path.write_text('', encoding='utf-8')

    @staticmethod
    def _vector(value):
        return [float(component) for component in value]

    def record_trajectory(self, time_s, position, cumulative_distance):
        with self.trajectory_path.open('a', newline='') as stream:
            csv.writer(stream).writerow([
                f'{time_s:.6f}', *[f'{value:.6f}' for value in self._vector(position)],
                f'{cumulative_distance:.6f}'])

    def record_stage(self, stage, duration_ms, iteration=None, frame_id=None,
                     source_stamp_s=None, details=''):
        with self.pipeline_path.open('a', newline='') as stream:
            csv.writer(stream).writerow([
                stage,
                '' if iteration is None else iteration,
                '' if frame_id is None else frame_id,
                f'{float(duration_ms):.6f}',
                '' if source_stamp_s is None else f'{float(source_stamp_s):.9f}',
                details,
            ])

    def record_depth_pose(self, sequence, depth_time_s, position, roll, pitch, yaw):
        with self.depth_pose_path.open('a', newline='') as stream:
            csv.writer(stream).writerow([
                sequence, f'{depth_time_s:.6f}',
                *[f'{value:.6f}' for value in self._vector(position)],
                f'{roll:.6f}', f'{pitch:.6f}', f'{yaw:.6f}'])

    def record_iteration(self, iteration, elapsed_ms, vehicle_position,
                         candidates, trajectory, published, route_blocked,
                         selected_targets=()):
        record = {
            'iteration': int(iteration),
            'elapsed_ms': float(elapsed_ms),
            'vehicle_position': self._vector(vehicle_position),
            'published': bool(published),
            'route_blocked': bool(route_blocked),
            'candidates': [],
        }
        selected_targets = [self._vector(target) for target in selected_targets]
        for index, candidate in enumerate(candidates):
            candidate_position = self._vector(candidate['position'])
            candidate_target = self._vector(candidate['target'])
            selected = any(
                sum((left - right) ** 2 for left, right in zip(
                    candidate_target, target)) < 1e-8
                for target in selected_targets)
            record['candidates'].append({
                'rank': index,
                'cluster_id': int(candidate.get('cluster_id', -1)),
                'position': candidate_position,
                'target': candidate_target,
                'utility': float(candidate.get('utility', 0.0)),
                'raw_gain': float(candidate.get('raw_gain', 0.0)),
                'raw_cost': float(candidate.get('raw_cost', 0.0)),
                'trajectory_id': iteration + 1 if published and selected else None,
            })
        with self.iteration_path.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(record) + '\n')

        if len(trajectory) == 0:
            return
        with self.future_path.open('a', newline='') as stream:
            writer = csv.writer(stream)
            for waypoint_index, waypoint in enumerate(trajectory):
                values = list(waypoint)
                writer.writerow([
                    iteration, waypoint_index,
                    *[f'{float(value):.6f}' for value in values[:3]],
                    f'{float(values[3]) if len(values) > 3 else 0.0:.6f}',
                ])


def generate_results_plot(results_dir='results', prefix='benchmark',
                          planner_prefix='planner'):
    """Create an XY overview with NBV scores, future routes, and camera cones."""
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle, Wedge
    except ImportError:
        return None

    root = Path(results_dir).expanduser()
    trajectory_path = root / f'{prefix}_trajectory.csv'
    depth_path = root / f'{planner_prefix}_depth_poses.csv'
    iteration_path = root / f'{planner_prefix}_nbv_iterations.jsonl'
    future_path = root / f'{planner_prefix}_future_trajectories.csv'
    if not trajectory_path.exists():
        return None

    trajectory = []
    with trajectory_path.open(newline='') as stream:
        for row in csv.DictReader(stream):
            trajectory.append((float(row['x_m']), float(row['y_m'])))

    poses = []
    if depth_path.exists():
        with depth_path.open(newline='') as stream:
            poses = list(csv.DictReader(stream))

    candidates = []
    if iteration_path.exists():
        with iteration_path.open(encoding='utf-8') as stream:
            for line in stream:
                if line.strip():
                    candidates.extend(json.loads(line)['candidates'])

    figure, axis = plt.subplots(figsize=(12, 9), constrained_layout=True)
    if trajectory:
        axis.plot(*zip(*trajectory), color='#1f2937', linewidth=1.6,
                  label='Vehicle trajectory', zorder=2)
        axis.scatter(*trajectory[0], color='#16a34a', s=45, label='Start', zorder=4)
        axis.scatter(*trajectory[-1], color='#dc2626', s=45, label='Latest', zorder=4)

    if candidates:
        scores = [candidate['utility'] for candidate in candidates]
        scatter = axis.scatter(
            [candidate['position'][0] for candidate in candidates],
            [candidate['position'][1] for candidate in candidates],
            c=scores, cmap='viridis', s=34, alpha=0.8,
            label='NBV viewpoints (utility)', zorder=3)
        figure.colorbar(scatter, ax=axis, label='NBV utility')
        for candidate in candidates:
            trajectory_id = candidate.get('trajectory_id')
            if trajectory_id is not None:
                axis.annotate(
                    str(trajectory_id),
                    (candidate['position'][0], candidate['position'][1]),
                    xytext=(3, 3), textcoords='offset points',
                    fontsize=7, color='#111827', zorder=5)

    axis.add_patch(Rectangle(
        (-14.1, -82.7), 2.1, 1.6,
        facecolor='#2563eb', edgecolor='#1e3a8a', alpha=0.16,
        linewidth=1.2, label='Rectangle object'))
    axis.add_patch(Rectangle(
        (-14.1, -87.4), 2.1, 1.6,
        facecolor='#2563eb', edgecolor='#1e3a8a', alpha=0.16,
        linewidth=1.2, label='Mirrored rectangle object'))
    pipe_objects = (
        ((-14.1, -81.1), (-15.7, -78.5)),
        ((-12.1, -81.1), (-11.0, -78.5)),
        ((-14.1, -88.4), (-15.7, -91.0)),
        ((-12.1, -88.4), (-11.0, -91.0)),
    )
    for index, (start, end) in enumerate(pipe_objects):
        axis.plot(
            [start[0], end[0]], [start[1], end[1]],
            color='#a16207', linewidth=7, alpha=0.22,
            solid_capstyle='round',
            label='Pipe objects' if index == 0 else (
                'Mirrored pipe objects' if index == 2 else None))
        axis.plot(
            [start[0], end[0]], [start[1], end[1]],
            color='#92400e', linewidth=1.2, alpha=0.9,
            solid_capstyle='round')

    if future_path.exists():
        by_iteration = {}
        with future_path.open(newline='') as stream:
            for row in csv.DictReader(stream):
                by_iteration.setdefault(row['iteration'], []).append(row)
        for index, (iteration, rows) in enumerate(by_iteration.items()):
            axis.plot([float(row['x_m']) for row in rows],
                      [float(row['y_m']) for row in rows],
                      '--', linewidth=1.0, alpha=0.65,
                      label='Future trajectory' if index == 0 else None)

    for index, pose in enumerate(poses):
        x, y = float(pose['x_m']), float(pose['y_m'])
        yaw = math.degrees(float(pose['yaw_rad']))
        axis.add_patch(Wedge((x, y), 0.45, yaw - 25, yaw + 25,
                             facecolor='#f97316', alpha=0.12,
                             edgecolor='#f97316', linewidth=0.7,
                             label='Camera cone' if index == 0 else None))
        axis.plot(x, y, '.', color='#f97316', markersize=2)

    axis.set_title('NBV planning process overview')
    axis.set_xlabel('X (m)')
    axis.set_ylabel('Y (m)')
    axis.set_aspect('equal', adjustable='datalim')
    axis.grid(True, alpha=0.25)
    handles, labels = axis.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    axis.legend(unique.values(), unique.keys(), loc='best')
    output_path = root / f'{prefix}_trajectory_overview.png'
    figure.savefig(output_path, dpi=160)
    plt.close(figure)
    return output_path


def default_run_id():
    return datetime.now().strftime('%Y%m%d_%H%M%S')
