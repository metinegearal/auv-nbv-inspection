import heapq

import numpy as np
from scipy.interpolate import splprep, splev


def astar_3d(start_pos, goal_pos, occupied, resolution, max_steps=50000):
    """Find a 6-connected voxel path between two metric positions."""
    start = tuple(np.floor(start_pos / resolution).astype(int))
    goal = tuple(np.floor(goal_pos / resolution).astype(int))
    if start in occupied or goal in occupied:
        return []

    open_set = [(0, start)]
    came_from = {}
    costs = {start: 0}
    directions = (
        (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0),
        (0, 0, 1), (0, 0, -1),
    )

    def heuristic(index):
        return sum(abs(index[i] - goal[i]) for i in range(3))

    for _ in range(max_steps):
        if not open_set:
            return []
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return [
                (np.asarray(index, dtype=np.float64) + 0.5) * resolution
                for index in path
            ]

        for direction in directions:
            neighbor = tuple(current[i] + direction[i] for i in range(3))
            if neighbor in occupied:
                continue
            cost = costs[current] + 1
            if cost >= costs.get(neighbor, float('inf')):
                continue
            came_from[neighbor] = current
            costs[neighbor] = cost
            heapq.heappush(open_set, (cost + heuristic(neighbor), neighbor))
    return []


def inflate_occupied(occupied_indices, collision_radius, resolution):
    """Inflate occupied voxels with a precomputed spherical neighborhood."""
    if not occupied_indices:
        return set()

    radius_voxels = int(np.ceil(collision_radius / resolution))
    offsets = [
        (offset_x, offset_y, offset_z)
        for offset_x in range(-radius_voxels, radius_voxels + 1)
        for offset_y in range(-radius_voxels, radius_voxels + 1)
        for offset_z in range(-radius_voxels, radius_voxels + 1)
        if offset_x ** 2 + offset_y ** 2 + offset_z ** 2
        <= radius_voxels ** 2
    ]

    inflated = set()
    for occupied_x, occupied_y, occupied_z in occupied_indices:
        inflated.update(
            (occupied_x + offset_x,
             occupied_y + offset_y,
             occupied_z + offset_z)
            for offset_x, offset_y, offset_z in offsets)
    return inflated


def path_is_collision_free(path, occupied, resolution):
    """Check a metric path against occupied voxels, including segment gaps."""
    if len(path) == 0:
        return False
    for start, end in zip(path[:-1], path[1:]):
        distance = np.linalg.norm(end - start)
        samples = max(1, int(np.ceil(distance / (resolution * 0.5))))
        for fraction in np.linspace(0.0, 1.0, samples + 1):
            point = start + fraction * (end - start)
            voxel = tuple(np.floor(point / resolution).astype(int))
            if voxel in occupied:
                return False
    return True


def plan_multi_waypoint_route(current_position, candidates, occupied,
                              resolution):
    """Sequence NBV candidates and connect them with collision-safe A*."""
    if not candidates:
        return np.empty((0, 3)), np.empty((0, 3))

    remaining = list(candidates)
    ordered = []
    position = np.asarray(current_position, dtype=np.float64)
    while remaining:
        index = int(np.argmin([
            np.linalg.norm(position - candidate[0])
            for candidate in remaining
        ]))
        candidate = remaining.pop(index)
        ordered.append(candidate)
        position = candidate[0]

    full_path = []
    target_tracking = []
    start = np.asarray(current_position, dtype=np.float64)
    for viewpoint, target in ordered:
        segment = astar_3d(start, viewpoint, occupied, resolution)
        if not segment:
            continue
        segment = np.asarray(segment)
        if full_path and np.allclose(full_path[-1], segment[0]):
            segment = segment[1:]
        full_path.extend(segment)
        target_tracking.extend([target] * len(segment))
        start = viewpoint

    return np.asarray(full_path), np.asarray(target_tracking)


def generate_smooth_trajectory(
        path, targets, num_points=200, smoothing=2.0,
        sampling_spacing=None):
    """Generate a smooth 4D trajectory with optional dense sampling."""
    if len(path) < 4 or len(targets) != len(path):
        return np.empty((0, 4))

    yaw = np.unwrap(np.arctan2(
        targets[:, 1] - path[:, 1], targets[:, 0] - path[:, 0]))
    try:
        spline, _ = splprep([
            path[:, 0], path[:, 1], path[:, 2], yaw,
        ], s=smoothing, k=3)
    except (ValueError, TypeError):
        return np.empty((0, 4))

    sample_count = int(num_points)
    if sampling_spacing is not None and sampling_spacing > 0.0:
        route_length = np.linalg.norm(np.diff(path, axis=0), axis=1).sum()
        sample_count = max(
            sample_count,
            int(np.ceil(route_length / sampling_spacing * 10.0)) + 1,
        )

    values = splev(np.linspace(0.0, 1.0, sample_count), spline)
    smooth_yaw = (np.asarray(values[3]) + np.pi) % (2.0 * np.pi) - np.pi
    return np.column_stack(values[:3] + [smooth_yaw])


def resample_trajectory_by_distance(trajectory, spacing, max_points=None):
    """Sample by distance, then optionally limit the result point count."""
    trajectory = np.asarray(trajectory, dtype=np.float64)
    if len(trajectory) < 2 or spacing <= 0.0:
        return trajectory

    positions = trajectory[:, :3]
    segment_lengths = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    keep = np.concatenate(([True], segment_lengths > 1e-9))
    trajectory = trajectory[keep]
    positions = trajectory[:, :3]
    segment_lengths = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    total_length = cumulative[-1]
    if total_length <= 1e-9:
        return trajectory[[0]].copy()

    distances = np.arange(0.0, total_length, spacing)
    if distances[-1] < total_length:
        distances = np.append(distances, total_length)

    sampled_positions = np.column_stack([
        np.interp(distances, cumulative, positions[:, axis])
        for axis in range(3)
    ])
    unwrapped_yaw = np.unwrap(trajectory[:, 3])
    sampled_yaw = np.interp(distances, cumulative, unwrapped_yaw)
    result = np.column_stack((sampled_positions, sampled_yaw))
    if max_points is not None and len(result) > max_points:
        indices = np.linspace(0, len(result) - 1, int(max_points)).round()
        result = result[np.unique(indices).astype(int)]
    return result
