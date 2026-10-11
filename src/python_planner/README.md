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

The trajectory begins at the current vehicle position and heading. Action
results and observer messages contain vehicle coordinates. End positions remain
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

# TF and NBV action transport

Build `nbv_interfaces`, `python_planner`, `motion_control`, and `bringup`, then
source `install/setup.bash`. The Python node serves `nbv_interfaces/action/ComputeNBV`
on `/planner/compute_nbv`. Goals with `trigger=true` request the next safe route;
`force_replan=true` requests an immediate pass. Feedback includes Computing,
Blacklisting, sensor/TF waits, strikes, and blacklist size. Only one goal is
accepted at a time. Cancellation returns CANCELED; unexpected pipeline errors
return ABORTED; three failed passes return ABORTED with `strikeout=true`.

The C++ iterator executes successful action results as flat `[x,y,z,yaw]` vehicle
waypoints in `map`. It keeps a monitoring goal open for periodic, arrival, and
collision replanning. A depleted queue replaces that goal with an immediate
request. Strikeout transitions to `MissionComplete`; rejection, other aborts,
invalid results, a missing server, or a feedback/computation timeout transition
to `Failed`. Both terminal states clear the queue and publish the existing
`/movement/finished_execution` stop signal. `/planner/state` exposes the state.
Timeouts are configured with `nbv_server_timeout_s`, `nbv_feedback_timeout_s`,
and `nbv_goal_timeout_s`. The goal timeout applies while feedback reports
`Computing` or `Blacklisting`; healthy sensor, point-cloud, and TF waits do not
consume that budget. Missing feedback still triggers the feedback timeout.
Before the first plan, the mission waits for odometry, RGB, a fresh depth frame,
and at least 40 occupied map points, reporting the corresponding waiting state.
Startup waits preserve the pending request and do not count as failed plans.
The completion signal suppresses further waypoint control updates. The existing
controller does not explicitly zero thruster commands on this signal; motor
control behavior is outside this transport change.

`object/areaWaypoints` remains a vehicle-coordinate telemetry stream. The C++
iterator ignores its nonempty messages and uses an empty message only to hold
execution when the Python collision monitor invalidates a route. Execution
resumes when a successful action supplies a checked replacement route.

Bringup creates `map -> odom -> base_link -> camera_link`. Camera translation
and optical orientation live in the `camera_static_transform` section of
`src/bringup/config/params.yaml`, passed to `static_transform_publisher`. The
45-degree tilt and optical convention (X right, Y down, Z forward) are part of
this transform. `TfCameraTransforms` looks up `map <- camera_link` at each depth
image's capture timestamp. Missing or extrapolated TF defers that image;
projection never falls back to a newer pose. Viewpoint placement uses the
inverse `camera_link <- base_link` TF including the mount rotation, with an exact
yaw solution for the controller's target-facing vehicle pose. Unreachable mount
placements discard individual candidates instead of aborting the action.

`world_frame`, `body_frame`, and `camera_frame` select the TF frames. The current
depth image topic is interpreted in the configured camera optical frame; its
legacy `depth_link` header is not used as a TF frame. For HoloOcean only,
`publish_vehicle_tf=true` bridges its global odometry to `odom -> base_link`;
position-only LocationSensor data uses magnetometer yaw until odometry provides
a full quaternion. On a robot, set this parameter false and supply localization
and URDF TF, omitting the simulator's identity `map -> odom` and camera static
publishers when those edges already have another owner. Launch arguments
`publish_sim_tf:=false` and `publish_camera_tf:=false` disable these publishers.
A dynamic camera TF broadcaster can replace the static publisher without
changing the planner.

Run transport/TF regressions in an isolated ROS domain:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=174 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 -m pytest -q src/python_planner/test/test_tf_action_ros.py
```
