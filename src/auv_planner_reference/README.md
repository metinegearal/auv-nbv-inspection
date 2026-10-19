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

In `src/auv_bringup/config/nbv_params.yaml`, utility is
`nbv_utility_gain_weight * normalized_gain - nbv_utility_cost_weight * normalized_cost`.
Both default to 1.0. Reduce `nbv_utility_cost_weight` to 0.5 for less cost influence.
`nbv_distance_weight`, `nbv_rotation_weight`, and `nbv_collision_weight` adjust
relative raw cost components; scaling all three equally cancels during min/max
normalization. `nbv_complexity_weight` changes the rough-surface gain preference.

# Seabed gain penalty

NBV gain also applies a per-surface-point seabed multiplier:
`1 - nbv_seabed_penalty_strength * exp(-max(0, z - floor_z) / nbv_seabed_decay_height)`.
The configured strength is 0.95 and decay height is 0.5 m, retaining 5% of gain
at/below the floor, 65% at 0.5 m above, and 87% at 1 m above. This affects raw
information gain before its minimum threshold and utility normalization; it
does not impose a vehicle depth limit or change collision handling.

Floor height is inferred automatically from the lowest well-supported broad
horizontal layer of the occupied map, without an absolute Z cutoff or fallback.
Height-density peaks, horizontal spread, and area coverage reject isolated low
noise, narrow pipes, and vertical-wall slices. Cached height-band sums/counts
and XY support are updated only for changed map voxels, including removals;
unchanged snapshots do not recompute the estimate or count observations twice.
The mean and unique floor-voxel count are retained for the run. If the map has
insufficient floor evidence, the gain penalty is disabled until a floor is
detected. This global estimate assumes a roughly level seabed and is a geometric
heuristic; a broad low horizontal structure can also resemble the floor.
Set `nbv_seabed_enabled: false` or strength to 0 to disable the gain effect.
The standalone visualization estimates the floor once from the loaded map.

# Motion continuity during NBV replanning

Bringup reserves `collision_radius + collision_tracking_margin` around occupied
voxels. The shipped tracking margin is 0.6 m: 0.4 m for waypoint handoff plus
0.2 m for tracking uncertainty. Keep this margin at least as large as the
controller's completion tolerance when tuning a custom configuration.

Obstacle inflation uses SciPy's compiled Euclidean distance transform on a
bounded local voxel grid, preserving the exact integer sphere and
`ceil(radius/resolution)` rule. Maps requiring more than four million grid
cells use sparse expansion instead. The clearance radius is unchanged; this
optimization reduces computation rather than removing collision voxels.

Active-route checks reconnect from the current vehicle position and validate
the remaining path; a safe old centerline is insufficient if the vehicle's new
entry crosses a wall. If an unfinished vehicle moves less than
`route_stall_min_progress_m` (0.15 m) over `route_stall_timeout_s` (10 s), the
planner clears the downstream queue and replans. This does not mark unobserved
space as an obstacle or relax collision checks. An occupied start still needs
a safe physical recovery; the planner will not route through the wall to escape.

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

Parameters are in `src/auv_bringup/config/params.yaml`. Increase
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
PYTHONPATH=src/auv_planner_reference python3 -m pytest -q src/auv_planner_reference/test/test_motion_continuity.py
```

# Frontier detection

Frontiers are boundaries in the occupied 3D surface cloud, rather than individual
depth-image pixels. Detection uses angular gaps between neighbors projected into
each point's tangent plane. The search is limited by `nbv_frontier_radius`
(default 0.5 m), with `nbv_frontier_neighbors` (40) as a cap and
`nbv_frontier_min_neighbors` (3) as the minimum usable support. This prevents
distant points across small holes from closing the apparent angular gap.
Normals are normalized and neighbors with zero tangent displacement are ignored.

Preprocessing retains statistically rejected boundary points if three other
points support them within `nbv_normal_radius`. Isolated outliers remain subject
to the statistical filter. The minimum cloud size follows the frontier-support
and cluster-size requirements instead of a fixed 40-point cutoff.

Clustering starts with dense compatible neighborhoods. Rejected small groups
leave their points available to other seeds. Short leftover boundary sections
can join an existing cluster when the combined patch still fits its FOV radius.
This avoids discarding real boundaries because of input order or a small tail
left by an earlier cluster.

Planning logs now report counts from preprocessing through detected frontiers,
clustered points, pose-history rejection, and minimum-gain rejection. A detected
frontier may still produce no waypoint because of these later filters, utility
thresholds, or route feasibility. The pose-history filter is a distance/yaw
heuristic; it does not establish visibility or account for occlusion.

Detection resolution still depends on voxel size, local sampling density,
normal estimation, and the radius/angle thresholds. Features lost during depth
filtering or voxel downsampling cannot be recovered by the angular-gap detector.
The radius must contain enough local surface neighbors; making it too small
leaves points unsupported, while making it too large can bridge holes again.

# Planning runtime

The planner limits candidate generation to nearby surfaces and reduces repeated work:

- Frontier query centers are restricted to surface points within
  `nbv_exploration_radius` of the current AUV position (default 15 m, configured in
  `auv_bringup/config/nbv_params.yaml`). Logs report focused and total counts. Returned
  indices remain global, and the full cloud supplies neighbors and obstacle
  context, so the focus cutoff does not create artificial surface boundaries.
  Frontiers outside this window are deferred; an empty window returns no candidates.
- PCA runs after clustering, only for the frontier points used in gain scoring.
  Each scored point still uses its original neighbors from the full surface
  cloud; the public PCA helper still scores every point when no subset is given.
- Frontier detection and PCA share the surface KDTree. Detection skips rows
  without enough local support and avoids processing padded missing neighbors.
- Clustering checks candidate cluster centers in arrays and sorts only eligible
  clusters when attaching leftover boundary points, preserving nearest-cluster
  preference and stable ties.
- Viewpoint gain and obstacle-distance/cost calculations run in batches using
  the same formulas, thresholds, viewpoints, and normal/PCA data.
- A* uses native integer voxel coordinates, skips superseded queue entries, and
  favors smaller remaining distance when total estimated costs tie. It retains
  the six-connected grid, admissible Manhattan heuristic, expansion limit, and
  downstream collision checks. Shortest grid-path cost is preserved; equally
  short paths may have different geometry on other maps.

Local saved-map replay on 2026-10-11, before adding the 15 m focus restriction,
median of three sequential runs per version:

| Stage | Before (ms) | After (ms) |
| --- | ---: | ---: |
| `nbv_pca` | 164.3 | 6.1 |
| `nbv_frontier_detection` | 187.5 | 176.1 |
| `nbv_clustering` | 146.8 | 24.7 |
| `nbv_candidate_evaluation` | 292.1 | 188.9 |
| `nbv_generation` (includes the stages above and preprocessing) | 959.9 | 501.6 |
| `route_search` | 1398.6 | 275.5 |

The replay used 19,674 occupied map points, the shipped tuning, an empty
observation history, and identical captured inputs for both versions. It retained
the same detected frontier indices, cluster memberships, 61 candidate viewpoints,
31 candidates above the utility threshold, and an identical 1,784-point route
visiting 27 targets (357.892855 m). Candidate numeric differences were below
4e-15. These timings exclude simulator/auv_stereo_depth contention and are not a replay
of the exact live iteration 704. Combined NBV-generation and route-search medians
fell from about 2.36 s to 0.78 s; frontier detection remains a significant stage.

An additional local detector replay with the 15 m window queried 6,571 of 12,355
surface points. Median detection time over five runs fell from 182.1 ms (full
cloud queries) to 86.7 ms (focused queries), about 2.1 times faster. The 641 focused
frontiers exactly matched the full detector's frontiers inside the window. This
isolates detector performance; it does not measure live total mission latency.

The equivalence regressions compare batched scores against the scalar formulas,
subset PCA against full-cloud PCA, and A* path lengths against exhaustive BFS
on obstacle maps. A separate open-volume test checks that equal-cost ties no
longer force A* to expand the whole volume before reaching the goal.
