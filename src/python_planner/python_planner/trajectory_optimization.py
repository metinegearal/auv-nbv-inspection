"""Geometric motion continuity for the existing untimed [x, y, z, yaw] API.

Costs are soft: a necessary obstacle detour or return to a frontier is allowed.
This is not a time-parameterized dynamic feasibility solver.
"""

import numpy as np
from scipy.optimize import minimize

from .route_planning import path_is_collision_free


def estimate_velocity(history, window=1.0):
    """Estimate world velocity from timestamped poses, avoiding twist-frame ambiguity."""
    if len(history) < 2:
        return np.zeros(3)
    latest_time = history[-1][0]
    samples = [sample for sample in history
               if 0.0 <= latest_time - sample[0] <= window]
    if len(samples) < 2:
        return np.zeros(3)
    times = np.asarray([sample[0] - latest_time for sample in samples], dtype=float)
    positions = np.asarray([sample[1] for sample in samples])
    times -= times.mean()
    denominator = np.dot(times, times)
    if denominator < 1e-9 or not np.all(np.isfinite(positions)):
        return np.zeros(3)
    return np.sum(times[:, None] * (positions - positions.mean(axis=0)),
                  axis=0) / denominator


def continuity_cost(trajectory, velocity, recent_positions=(),
                    continuity_weight=8.0, reversal_weight=4.0,
                    history_weight=2.0, history_radius=0.5):
    """Score entry direction, local reversals, yaw winding, and recent retracing."""
    if len(trajectory) < 2:
        return 0.0
    steps = np.diff(trajectory[:, :3], axis=0)
    lengths = np.linalg.norm(steps, axis=1)
    directions = steps / np.maximum(lengths[:, None], 1e-6)
    moving = lengths > 1e-6
    directions = directions[moving]
    cost = 0.0
    speed = np.linalg.norm(velocity)
    if speed > 0.05 and len(directions):
        # A soft entry tangent, fading along the first few samples.
        alignment = directions[:3] @ (velocity / speed)
        cost += continuity_weight * min(speed / 0.3, 1.0) * np.sum(
            np.array([1.0, 0.5, 0.25])[:len(alignment)] * (1.0 - alignment)**2)
    if len(directions) > 1:
        alignment = np.sum(directions[1:] * directions[:-1], axis=1)
        cost += reversal_weight * np.sum(np.maximum(0.0, -alignment)**2)
        # Smooth circles have no locally reversed segments. Detect accumulated
        # turning over the next 10 m as well, without penalizing a long mission's
        # entire sequence of otherwise necessary turns.
        horizon = np.cumsum(lengths[moving]) <= 10.0
        turn_angles = np.arctan2(
            np.linalg.norm(np.cross(directions[:-1], directions[1:]), axis=1),
            alignment)
        excess_turn = max(0.0, np.sum(turn_angles[horizon[1:]]) - np.pi)
        cost += reversal_weight * excess_turn**2
    yaw = np.unwrap(trajectory[:, 3])
    yaw_steps = np.diff(yaw)
    # Penalize accumulated travel even if the final heading matches the start.
    shortest_yaw = abs(np.arctan2(np.sin(yaw[-1] - yaw[0]),
                                  np.cos(yaw[-1] - yaw[0])))
    excess = max(0.0, np.abs(yaw_steps).sum() - shortest_yaw - np.pi)
    cost += reversal_weight * (np.sum(yaw_steps**2) + excess**2)
    history = np.asarray(recent_positions, dtype=np.float64)
    if history.size and history_radius > 0.0:
        history = history.reshape(-1, 3)
        # Exclude the immediate pose neighborhood; departing it is unavoidable.
        history = history[np.linalg.norm(history - trajectory[0, :3], axis=1)
                          > 2.0 * history_radius]
        if len(history):
            distances = np.linalg.norm(
                trajectory[1:, None, :3] - history[None, :, :], axis=2).min(axis=1)
            cost += history_weight * np.sum(
                np.maximum(0.0, 1.0 - distances / history_radius)**2)
    return float(cost)


def optimize_trajectory(trajectory, current_position, current_yaw, velocity,
                        occupied, resolution, recent_positions=(),
                        continuity_weight=8.0, reversal_weight=4.0,
                        history_weight=2.0, history_radius=0.5,
                        max_deviation=0.5, max_iterations=40,
                        collision_spacing_factor=0.5):
    """Optimize bounded offsets, accepting only a safer, lower-cost trajectory.

    At most 32 controls keep replanning bounded. Offsets are interpolated onto
    every input sample so collision checks retain the original path detail.
    End positions remain fixed; the initial yaw equals the measured heading.
    """
    reference = np.asarray(trajectory, dtype=np.float64).copy()
    if len(reference) < 2:
        return reference
    reference[0, :3] = current_position
    reference[:, 3] = np.unwrap(np.r_[current_yaw, reference[:, 3]])[1:]
    reference[0, 3] = current_yaw
    if not path_is_collision_free(reference[:, :3], occupied, resolution,
                                  collision_spacing_factor):
        return np.empty((0, 4))
    weights = dict(continuity_weight=continuity_weight,
                   reversal_weight=reversal_weight,
                   history_weight=history_weight, history_radius=history_radius)
    indices = np.unique(np.linspace(0, len(reference) - 1,
                                    min(32, len(reference))).round().astype(int))
    sample_indices = np.arange(len(reference))
    reference_lengths = np.linalg.norm(np.diff(reference[:, :3], axis=0), axis=1)

    def decode(offsets):
        controls = offsets.reshape(-1, 4)
        return reference + np.column_stack([
            np.interp(sample_indices, indices, controls[:, axis])
            for axis in range(4)])

    def objective(offsets):
        trial = decode(offsets)
        fidelity = 2.0 * np.sum((trial[:, :3] - reference[:, :3])**2)
        observation = 0.5 * np.mean((trial[:, 3] - reference[:, 3])**2)
        steps = np.diff(trial[:, :3], axis=0)
        smoothness = np.sum(np.diff(steps, axis=0)**2)
        # Prevent lowering the directional cost by collapsing the first steps.
        length_cost = 4.0 * np.sum(((np.linalg.norm(steps, axis=1) - reference_lengths)
                                   / np.maximum(reference_lengths, 0.1))**2)
        return (fidelity + observation + smoothness + length_cost
                + continuity_cost(trial, velocity, recent_positions, **weights))

    bounds = [(-max_deviation, max_deviation)] * (len(indices) * 4)
    for axis in range(4):
        bounds[axis] = (0.0, 0.0)
    for axis in range(3):
        bounds[-4 + axis] = (0.0, 0.0)
    # Yaw has a wider range than metric position offsets.
    for index in range(1, len(indices)):
        bounds[index * 4 + 3] = (-np.pi, np.pi)
    initial = np.zeros(len(indices) * 4)
    result = minimize(objective, initial, method='L-BFGS-B', bounds=bounds,
                      options={'maxiter': int(max_iterations), 'maxfun': 12000})
    # An iteration-limited result can still be useful, but never trust NaNs or
    # an unsafe/lower-quality iterate. Backtracking preserves a safe fallback.
    if np.all(np.isfinite(result.x)):
        initial_cost = objective(initial)
        for scale in (1.0, 0.5, 0.25):
            trial = decode(scale * result.x)
            if (objective(scale * result.x) < initial_cost
                    and path_is_collision_free(trial[:, :3], occupied, resolution,
                                               collision_spacing_factor)):
                return trial
    return reference
