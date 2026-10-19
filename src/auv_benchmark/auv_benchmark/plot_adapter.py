"""File-backed telemetry and plots for mid-mission NBV evaluation."""

import csv
import json
import math
from datetime import datetime
from pathlib import Path


def generate_results_plot(results_dir='results', prefix='benchmark',
                          planner_prefix='planner', show_reference_objects=True):
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

    if show_reference_objects:
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
