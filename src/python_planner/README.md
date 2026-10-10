# NBV viewpoint direction and utility tuning

`select_upward_view_axis()` chooses the target-to-viewpoint central line with
elevation fixed at +45 degrees to the horizontal. Only yaw varies: it scans
`nbv_view_axis_azimuth_steps` directions using all occupied map points, scores
minimum clearance along the departing ray, then mean clearance, and favors
heading toward the AUV on ties. Surface normals still determine clustering and
incidence gain. The existing 15/30 degree cone offsets surround this new axis;
the cone viewpoints themselves do not all have fixed elevation.

Clearance means absence of nearby occupied samples, not confirmed observed free
space. Inflated-map routing and trajectory collision checks remain authoritative.

In `src/bringup/config/nbv_params.yaml`, utility is
`nbv_utility_gain_weight * normalized_gain - nbv_utility_cost_weight * normalized_cost`.
Both default to 1.0. Reduce `nbv_utility_cost_weight` to 0.5 for less cost influence.
`nbv_distance_weight`, `nbv_rotation_weight`, and `nbv_collision_weight` adjust
relative raw cost components; scaling all three equally cancels during min/max
normalization. `nbv_complexity_weight` changes the rough-surface gain preference.

# Motion continuity during NBV replanning

The planner keeps two stages: collision-safe A* waypoint routing, followed by
bounded numerical trajectory optimization. Both use the measured direction of
travel. World-frame velocity is estimated by fitting positions from the last
second of timestamped odometry; the bridge does not always populate twist and
its velocity frame is ambiguous. Duplicate timestamps are ignored and a clock
reset clears old motion history.

NBV waypoint ordering preserves utility after filtering: the highest-utility
reachable candidate is visited first. Distance and a soft cost for candidates
behind the current velocity break equal-utility ties. A failed A* search is
skipped without updating the planning position or direction; remaining choices
are evaluated from the last successfully connected viewpoint. This favors early
information gain over the shortest total route. Generic callers that omit
`candidate_utilities` retain distance/continuity ordering.

The trajectory stage compares the spline and A* route, then uses L-BFGS-B to
minimize entry-direction mismatch, adjacent segment reversals, accumulated
turning over the next 10 metres, excess yaw travel, and retracing the last five
seconds of motion. Route fidelity, segment lengths, and smoothness are included
to prevent shortcutting or collapsing steps. Position and yaw are interpolated
separately so heading changes cannot bend the spatial spline.

The trajectory begins at the current camera-offset position and heading,
matching the existing controller's offset subtraction. End positions remain
fixed and XYZ optimization offsets are bounded to 0.5 metres. Every candidate
and optimized result is collision checked, including segment interiors. Unsafe
optimizer results are backtracked or discarded in favor of the safe reference.
Reversals remain possible when required to reach a frontier or avoid obstacles;
these penalties do not prohibit legitimate return paths.

Parameters are in `src/bringup/config/params.yaml`. Increase
`trajectory_continuity_weight` for stronger entry alignment,
`trajectory_reversal_weight` for stronger turn/loop penalties, or
`trajectory_history_weight` for stronger avoidance of recent travel.
`trajectory_optimization_enabled: false` disables motion costs and numerical
optimization. `trajectory_optimizer_max_iterations` limits solver work; at most
32 controls and 64 historical positions are used.

The `[x, y, z, yaw]` controller API has no timestamps, so this is geometric
continuity optimization, not enforcement of acceleration, speed, or yaw-rate
limits. Full dynamic guarantees require a timed trajectory and controller
support. Validate these default weights in the AUV simulator before tuning them
for a mission.

Run the standalone regressions from the ROS workspace root:

```bash
PYTHONPATH=src/python_planner python3 -m pytest -q src/python_planner/test/test_motion_continuity.py
```
