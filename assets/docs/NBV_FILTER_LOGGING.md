# Finding the filter that removes all NBVs

Verbose diagnostics are enabled in `src/auv_bringup/config/params.yaml`:

```yaml
planner_node:
  ros__parameters:
    nbv_filter_logging: true
    results_dir: results
```

Restart the planner after changing this setting. The updated `auv_planner_reference`, `auv_benchmark`, and `auv_bringup` packages have been built. Logging does not change filter thresholds, blacklist behavior, candidate scoring, or route selection.

## Where to look

- Console messages beginning with `[NBV_FILTER iteration=... frame=...]` give input/output counts and details for every stage.
- `[NBV_EMPTY] No trajectory published: ...` names the final failure reason.
- `results/planner_nbv_filters.jsonl` contains the same structured events, with iteration and frame IDs. `results_dir` is resolved relative to the launch working directory; startup prints the log path.
- The JSONL file is reset at planner startup. Preserve it before starting another run if you need to compare runs.

Show only final outcomes and explicit NBV exhaustion:

```bash
rg '"stage": "(nbv_empty|planning_result|mission_completion)"' results/planner_nbv_filters.jsonl
```

Then inspect events sharing the failing iteration:

```bash
rg '"iteration": 100,' results/planner_nbv_filters.jsonl
```

## Meaning of common failures

| Reason | Evidence to inspect |
| --- | --- |
| `occupied_input_empty` | No occupied points reached NBV generation; inspect `depth_range`, `depth_point_cap`, and the map size. |
| `world_z_removed_all` | `world_z` records the active threshold, strict comparison, actual Z range, and number at/below the threshold. |
| `minimum_cloud_size` | Check counts after downsampling/outlier removal against the hardcoded minimum of 40. |
| `no_frontiers` | `frontier_detection` records neighbor count, angular threshold, and invalid-normal count. |
| `clustering_removed_all` | `cluster_min_samples` records each dropped group's seed, point count, and minimum; `frontier_clustering` summarizes retained points/clusters. |
| `only_flat_horizontal_frontiers` | `cluster_geometry` records complexity, normal variation, and horizontal alignment for each cluster; `structural_target_filter` counts retained structural targets. |
| `geometric_significance_removed_all` | `geometric_significance` compares each cluster with the strongest geometry using the configured fraction; `significant_frontiers` summarizes retained targets. |
| `cone_generated_none` | Inspect cone standoff, polar angles, and azimuth sample count. |
| `raw_gain_removed_all` | `candidate_raw_gain` records each viewpoint's gain and threshold; `raw_gain_filter` summarizes rejections. |
| `blacklist_removed_all` | Each `candidate_blacklist` lists matching banned coordinates and distances within 0.5 m. |
| `minimum_utility_removed_all` | Inspect each `candidate_minimum_utility`, plus raw/normalized gain and cost in `candidate_utility` and `utility_normalization`. Equal gains normalize to one and equal costs to zero, so a singleton structural candidate remains eligible. |
| `astar_removed_all` | `astar_search` identifies occupied endpoints, exhausted search queue, or exhausted step budget for each viewpoint and target. |
| `route_search_time_budget_exhausted` | No safe segment completed within the shared search deadline. The target is retained rather than blacklisted. |
| `route_collision` | `path_collision_check` identifies the first blocked voxel and segment. |
| `all_trajectory_options_rejected` | Inspect `trajectory_option` for spline validation and collision events. Raw voxel routes are no longer published as a fallback. |
| `trajectory_reconnection_rejected` | The vehicle moved during planning and no safe connection survived; inspect the latest position in `trajectory_reconnection`. |
| `nbv_exception` | Inspect exception type and message in `nbv_exception`. |

`blacklist_addition` records the actual failed target and its temporary cooldown (`route_blacklist_cooldown_s`, default 30 seconds). Spline rejection identifies the first target in travel order, which can differ from the highest-utility candidate. The planner requests a new pass to try another target; expiry permits a later retry. `blacklist_expired` reports targets becoming eligible again. `blacklist_wait` does not count temporary exclusion as mission completion. Search timeouts or failures without an identified target remain `blacklist_skipped`. `mission_completion` records the last failure reason when the three-empty-candidate-pass rule ends mapping.

Verbose mode writes per-candidate events and historical observation matches. Set `nbv_filter_logging: false` and restart the planner when the investigation is complete.

## Route search limits and safe resampling

The YAML now sets `route_search_time_budget_s: 2.0` and `route_max_candidates: 4`.
The node takes the highest-utility candidates, then orders that subset for travel.
Every A* call shares the same monotonic deadline; the 50,000-expansion limit still applies to each call.
The deadline is checked at each expansion and before each new candidate. Diagnostics and collision validation can add time beyond the search deadline, so this is a cooperative budget rather than a hard real-time guarantee.
If the deadline expires after some safe segments complete, those segments can still be published.

`route_candidate_limit` records candidates excluded by the cap; `route_candidates` records attempted/accepted counts, elapsed time, and budget exhaustion.
A* uses deeper-node tie breaking with Python integer voxel indices and skips closed/stale entries.

Only a validated spline is eligible for execution. If a combined spline intersects occupancy, the planner also tries a spline ending at the first structural viewpoint, validates it with the same clearance, and tracks only that target until arrival/replanning. `trajectory_option` reports `spline` and `first_target_spline` attempts. A raw A* polyline is not a validated motion trajectory for the controller and is no longer an execution fallback.
When new occupancy blocks the active route, the planner publishes an empty route. The iterator clears its queue and forwards cancellation; the controller publishes eight zero thrust commands and remains idle until a new waypoint arrives.

## Structure selection and vehicle collision checks

Gain is averaged per cluster point and uses `nbv_complexity_weight`; a dense flat patch cannot win simply by containing more samples. Flat horizontal clusters are excluded from targets using `nbv_flat_horizontal_max_complexity` and `nbv_flat_horizontal_max_tilt_deg`. Remaining targets must reach `nbv_min_geometry_fraction` of the strongest cluster's complexity/normal variation (default 0.25; zero disables this relative gate). These events include actual scores and thresholds. Seabed points remain in the occupancy map for collision checking.

`cluster_observation` now records nearby historical poses without discarding a cluster: proximity and yaw alone do not prove coverage. `nbv_visited_gain_discount` controls any score discount (default 1.0, no discount).

Depth projection rotates the mounted camera offset with vehicle attitude, and OctoMap rays start at that same sensor origin. Candidate camera positions are converted to vehicle positions for routing. Every route segment is checked against all voxels it touches, including voxel edges and corners. Occupancy is inflated by `collision_radius + collision_tracking_margin` (configured as 0.3 + 0.4 m). The final check reconstructs the vehicle route from the float32 waypoint message and the iterator's offset conversion.

The controller acknowledges arrival using full 3D distance and stops thrust at completion. The iterator accepts only an acknowledgement for its current waypoint. These checks cover known mapped obstacles and configured clearance; they do not establish safety against unmapped obstacles or vehicle overshoot in a full mission.
`trajectory_cancelled` records the cancellation. This cancels commands; it does not establish instantaneous physical braking or clearance from obstacles absent from the map.
