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
        self.filter_path = self.results_dir / f'{prefix}_nbv_filters.jsonl'
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
        self.filter_path.write_text('', encoding='utf-8')

    def record_filter(self, stage, input_count, output_count, details,
                      iteration=None, frame_id=None):
        """Persist filter counts and reasons immediately for interrupted runs."""
        event = dict(stage=stage, iteration=iteration, frame_id=frame_id,
                     input_count=int(input_count), output_count=int(output_count),
                     details=self._json_safe(details))
        with self.filter_path.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(event, allow_nan=False) + '\n')

    @staticmethod
    def _json_safe(value):
        """Retain nonfinite diagnostic values without producing invalid JSON."""
        if isinstance(value, dict):
            return {str(key): MidrunLogger._json_safe(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [MidrunLogger._json_safe(item) for item in value]
        if hasattr(value, 'tolist'):
            return MidrunLogger._json_safe(value.tolist())
        if isinstance(value, float) and not math.isfinite(value):
            return str(value)
        return value

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


class PipelineFileSink:
    """Perception depends only on the timing role."""
    def __init__(self, results_dir='results', prefix='perception'):
        self._writer = MidrunLogger(results_dir, prefix, pipeline_only=True)
    def record_stage(self, *args, **kwargs):
        return self._writer.record_stage(*args, **kwargs)

class PlannerFileSink:
    def __init__(self, results_dir='results', prefix='planner'):
        self._writer = MidrunLogger(results_dir, prefix)
    @property
    def description(self):
        return str(self._writer.filter_path)
    def record_stage(self, *args, **kwargs):
        return self._writer.record_stage(*args, **kwargs)
    def record_filter(self, *args, **kwargs):
        return self._writer.record_filter(*args, **kwargs)
    def record_depth_pose(self, *args, **kwargs):
        return self._writer.record_depth_pose(*args, **kwargs)
    def record_iteration(self, *args, **kwargs):
        return self._writer.record_iteration(*args, **kwargs)
