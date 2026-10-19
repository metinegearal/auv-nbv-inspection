# Structural NBV correction and validation

## Corrections

- Gain uses the configured complexity weight and an average per surface point, removing the advantage of dense flat patches.
- Flat horizontal clusters are excluded as NBV targets, while their points remain obstacles. A configurable relative geometry threshold prioritizes significant structure.
- Historical proximity/yaw does not establish coverage or remove a structural frontier. An identified failed route target is temporarily blacklisted for 30 seconds so another structural target can be tried; it is not permanently removed. Temporary exclusion does not count as exploration completion.
- Depth projection and OctoMap sensor origins rotate the mounted camera offset with vehicle attitude. Candidate camera positions are converted to vehicle positions before routing.
- Collision validation traverses every voxel touched by the vehicle route, including edges/corners. Configured clearance is 0.3 m vehicle radius plus 0.4 m tracking margin. The final check reconstructs the vehicle coordinates from float32 transport and the iterator's offset conversion.
- If a combined spline collides, a spline ending at its first structural viewpoint is checked with the same clearance. Only the executed target is tracked for arrival/replanning.
- Waypoint completion requires reaching the 3D point; stale acknowledgements cannot skip the next waypoint. Cancellation clears the route and commands zero thrust.

## Saved-map replay

Input: existing `results/final_map_ros2.bt` and the last pose in `results/benchmark_trajectory.csv`. This is an offline replay of a historical map, not a new mission.

| Check | Result |
| --- | --- |
| Occupied input points | 55,351 |
| Frontier clusters | 35 |
| Flat horizontal target clusters rejected | 19 |
| Significant structural candidates retained | 4 |
| Selected structural target | (350.427777, 829.040554, -299.647777) |
| Combined spline | Rejected: intersects inflated occupancy |
| First-target spline | Accepted: 53 waypoints |
| Executed vehicle endpoint after float32 transport | (349.236151, 828.213940, -298.204773) |
| Executed vehicle route versus inflated occupancy | Passed exact voxel traversal |

## Other verification

- 65 Python regression tests passed: geometry selection, scoring, route budgets, exact collision traversal, coordinate transforms, float32 transport, spline fallback, diagnostics, and motion continuity.
- Isolated ROS test using the actual C++ nodes passed: full 3D completion, stale acknowledgement rejection, cancellation, and remaining idle until a new route.
- `motion_control`, `python_planner`, and `bringup` built successfully. Restart the nodes to use the changes; planner startup creates a fresh map.

No full simulator mission has been rerun. These checks establish behavior against known occupancy; they do not verify physical tracking/overshoot or protection against unmapped obstacles. Camera placement at future viewpoints assumes a level vehicle, and clearance still depends on the configured vehicle radius and tracking margin.
