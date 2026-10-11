# Replace NBV camera math with TF2 and planner topics with a ROS action

## Summary

The Python NBV planner previously embedded its camera mount in code and projected
depth using manually constructed rotation matrices. The C++ waypoint iterator
requested plans through a boolean topic, with no progress, failure, or server
health feedback.

This change projects depth through timestamped TF2 lookups and uses a
`ComputeNBV` action for requests, feedback, and trajectory results. A Strike 3
abort explicitly transitions C++ to `MissionComplete`; other aborts, rejected
goals, invalid results, and timeouts transition it to `Failed`.

## Changes

- Configure the camera translation and optical orientation in
  `src/bringup/config/params.yaml` and publish the mount using
  `static_transform_publisher`. The launch adapter converts YAML values to the
  CLI flags required by the Jazzy executable.
- Add `tf2_ros.Buffer` and `TransformListener` to the Python node. Project each
  depth image using `map <- camera_link` at its capture timestamp. Missing,
  extrapolated, or zero-timestamp TF defers projection instead of substituting
  the latest vehicle pose.
- Convert camera viewpoints to vehicle positions using the inverse
  `camera_link <- base_link` transform. Remove hardcoded Python camera offsets
  and manual body rotation matrices. Solve target-facing mount placement exactly
  and discard physically unreachable candidates individually.
- Add the `nbv_interfaces/action/ComputeNBV.action` contract, an asynchronous
  Python action server, and a C++ `rclcpp_action::Client`.
- Return flat `[x, y, z, yaw]` vehicle waypoints in `map`; remove the old Python
  offset addition and C++ offset subtraction convention.
- Replace `/planner/request_nbv` with `/planner/compute_nbv`. Report computation,
  blacklisting, input/TF waits, strike count, and blacklist size as feedback.
- Accept one goal at a time and support cancellation. Keep a monitoring goal
  open during route execution for periodic, arrival, and collision replanning.
  A depleted or invalidated route requests an immediate replan.
- On Strike 3, return `ABORTED` with `strikeout=true`. C++ clears its queue,
  publishes `MissionComplete` on `/planner/state`, and sends the existing
  completion signal. Other failures use `Failed` and the same completion signal.
- Retain `object/areaWaypoints` for telemetry and collision invalidation. C++
  installs routes only from action results; empty observer messages clear its
  route and suspend execution until a checked replacement arrives.
- Add configurable server, feedback, and goal timeouts. Add launch switches
  `publish_sim_tf` and `publish_camera_tf` so localization, URDF, or a dynamic
  mount broadcaster can own those TF edges instead.
- Wait for sensor inputs and an initial cloud of at least 40 occupied points
  before planning. Startup waits preserve the request without consuming strikes;
  healthy input/TF waits do not consume the C++ computation timeout.

## First-plan abort fix

The reported early finish was an action abort on the first planning pass, after
572 map points and seven candidates were available. The iterative mount-placement
solver failed to converge for a valid near-lateral viewpoint. The exact solution
handles that recorded candidate; infeasible candidates follow normal route
failure/blacklisting logic. A separate startup gate prevents sparse initial maps
from accumulating strikes before enough data arrives.

## Camera placement verification

The optical orientation and reference sensor have been checked. The absolute
translation is preserved from the old implementation, not independently
calibrated against the installed simulator asset.

| Item | Finding |
| --- | --- |
| Depth reference image | `DownCamera`: the depth callback passes it as the first image to BGNet, despite the subscription variable names. |
| Sensor attachment | `CameraLeftSocket`, with scenario offset `(0, 0.275, -0.5)` metres. |
| Sensor rotation | `(roll, pitch, yaw) = (0, 45, 0)` degrees. |
| TF optical orientation | Matches the configured tilt and X-right, Y-down, Z-forward optical convention. |
| Current body-to-camera translation | `(0.35, 0.3875, -0.5)` metres, matching the previous implementation. |
| Remaining assumption | This translation assumes the socket is at `(0.35, 0.1125, 0)` relative to the vehicle pose origin. That socket pose has not been verified against the installed Ocean asset. |

HoloOcean defines scenario location/rotation as offsets from the selected
socket: [sensor placement documentation](https://byu-holoocean.github.io/holoocean-docs/v2.3.0/sensors/docs/sensor-config.html).
The installed Python package is version 2.3.0; its
[HoveringAUV documentation](https://byu-holoocean.github.io/holoocean-docs/v2.3.0/agents/agents/hovering-auv-agent.html)
lists camera sockets without numeric socket poses. Before treating the mount
as calibrated, verify the asset's socket transform and vehicle pose origin,
then update the YAML translation if needed.

## Validation

- All four affected packages built successfully:
  `nbv_interfaces`, `motion_control`, `python_planner`, and `bringup`.
- 107 planning and architecture checks passed, including startup with missing
  inputs or 0–39 map points, immediate planning on the first sufficient cloud,
  genuine Strike 3, and per-candidate mount rejection.
- 16 ROS integration checks passed using the multithreaded executor and an
  isolated ROS domain. Coverage includes TF interpolation, timestamp rejection,
  mount reconfiguration, inverse placement, the real static publisher, action
  success/feedback, concurrent-goal rejection, cancellation, aborts, C++ route
  execution, collision invalidation/resume, and silent-server detection. New
  coverage reproduces the recorded first-plan viewpoint, returns a real mission
  route after cloud startup, and verifies that healthy cloud waits survive the
  computation timeout while actual computation remains bounded.
- The C++ trajectory queue regression passed: 124 functional checks in total.
- The actual `NBVUtilityNode` initialized and shut down successfully, with
  smoke-test outputs confined to `/tmp`.
- Whitespace checks passed for the changed source files.

## Limits and review notes

- A full HoloOcean mission and physical camera extrinsic calibration have not
  been performed. TF tests verify transformation behavior, not the asset's
  absolute camera position.
- The simulator's position-only odometry uses magnetometer yaw until a full
  quaternion is available. It does not supply full attitude through that
  fallback. Disable simulator TF when a localization system owns the tree.
- The existing controller completion handler suppresses further control updates
  but does **not explicitly zero thruster commands**. This PR changes planning
  and transport; it does not modify motor control.
- Consumers of nonempty `object/areaWaypoints` messages must interpret them as
  vehicle coordinates. The C++ iterator executes action results.
- Existing generated result files, tracked bytecode changes, and the BGNet
  submodule's local changes are unrelated and should be excluded from this PR.
