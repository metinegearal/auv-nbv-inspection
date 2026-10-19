# NBV filter audit

Scope: source code in `src/auv_planner_reference`, inspected on 2026-10-04. These are code findings, not proof of which condition caused a particular run to lose a target. No planner behavior was changed during this audit. Line numbers below refer to the audit snapshot before diagnostic logging was added; see `NBV_FILTER_LOGGING.md` to trace failures in a run.

## Checks that can discard a useful NBV

- **Permanent blacklist:** `planner_node.py:641` drops every target within a hardcoded **0.5 m** of a blacklisted coordinate. At line 660, a failed overall plan adds the first remaining candidate's target, even when the failure does not establish that this specific target is unreachable. There is no expiration or map-change retry. Utility rejection alone can also reach this blacklist branch.
- **Assumed observation:** `get_NBV.py:307-318` drops a whole cluster if any historical vehicle position lies within the configured distance range and yaw tolerance of its centroid. This does not test line of sight, occlusion, camera pitch/vertical FOV, or actual surface coverage. Defaults are **0.1 < distance < 3 m** and yaw error **< 30 degrees**.
- **Minimum cloud size:** `get_NBV.py:269` returns no NBVs if fewer than **40 cleaned points** remain. This is hardcoded and independent of cluster size or frontier neighbor parameters.
- **Utility normalization:** `get_NBV.py:225-229, 368-378` maps a constant gain/cost array to zeros. A single surviving candidate therefore receives utility **0**; `planner_node.py:385-390` rejects it when `min_utility_threshold > 0`. The current YAML threshold is **0.1**.
- **Early completion:** `planner_node.py:662-665` ends mapping after **three** consecutive planning passes with no candidates surviving the blacklist. This does not establish that all surfaces are observed; preprocessing, gain filtering, assumed observation, or blacklisting can also yield an empty list.

## Active pipeline filters and selection rules

| Stage / source | Rule | Configuration |
| --- | --- | --- |
| Odometry, `planner_node.py:183` | Discard poses when both absolute X and Y are below 0.01 m, regardless of Z. Valid starts at the origin can be ignored. | Hardcoded |
| Pipeline, `planner_node.py:523-538` | Wait for odometry, an RGB message, and a new depth frame; stop processing once mapping is marked finished. | State checks |
| Depth, `planner_node.py:267-278` | Require a 2D image and finite distances strictly between the minimum and maximum sensor range. | `min_depth`, `max_depth`; YAML 0.5 and 4.0 m |
| Depth sampling, `planner_node.py:273,313` | Sample every stride pixels; randomly retain at most 4000 projected points per frame. | `depth_stride`; 4000 is hardcoded |
| Map, `planner_node.py:318-330` | Quantize into voxels; OctoMap insertion limits ray range. The incremental occupied cache adds endpoint voxels and does not remove old entries after later free-space observations. Stale occupancy can obstruct routes. | `voxel_resolution`, `max_depth`, `use_incremental_map_cache` |
| World Z, `get_NBV.py:256-257` | Retain only occupied points with **Z > threshold**; equality is excluded too. | `nbv_z_min_target`; the separate `z_min_target` does not control this filter |
| Downsampling, `get_NBV.py:263` | Merge nearby points into voxels. | `nbv_voxel_downsample_size` |
| Denoising, `get_NBV.py:265` | Statistical outlier removal can remove sparse legitimate surfaces. | `nbv_outlier_neighbors`, `nbv_outlier_std_ratio` |
| Cloud size, `get_NBV.py:269` | Reject the entire cloud below 40 points after cleanup. | Hardcoded |
| Frontiers, `get_NBV.py:66-104` | Require usable normals and nonduplicate neighbors; keep points whose largest angular neighbor gap exceeds the threshold. | `nbv_frontier_neighbors`, `nbv_frontier_angle_threshold_deg`; numeric tolerances are hardcoded |
| Clusters, `get_NBV.py:125-131` | Group frontiers by distance and normal angle; discard groups below minimum sample count. | `nbv_cluster_fov_radius`, `nbv_cluster_max_angle_deg`, `nbv_cluster_min_samples` |
| Observation history, `planner_node.py:345-351` | Store another pose only after moving at least 0.5 m or turning at least 15 degrees. | Hardcoded |
| Visited clusters, `get_NBV.py:307-318` | Remove clusters assumed observed based on distance and yaw. | `nbv_visited_min_distance`, `nbv_visited_max_distance`, `nbv_visited_yaw_threshold_deg` |
| Candidate generation, `get_NBV.py:324` | Generate only viewpoints on the configured cone; useful viewpoints elsewhere are never considered. | `nbv_standoff`, `nbv_polar_angles_deg`, `nbv_azimuth_steps` |
| Raw gain, `get_NBV.py:179-187,335` | Ignore rays shorter than 0.0001 m; back-facing surfaces contribute zero; reject gains below the threshold. Complexity multiplier is hardcoded to 15 despite a declared `nbv_complexity_weight`. | `nbv_min_raw_gain`; multiplier/tolerance hardcoded |
| Utility selection, `get_NBV.py:368-395` | Normalize gain and cost, calculate gain minus cost, then keep only one winning viewpoint per cluster. Collision proximity penalizes cost rather than directly rejecting candidates here. | `nbv_distance_weight`, `nbv_rotation_weight`, `nbv_collision_weight`, `nbv_collision_distance_threshold`, `nbv_collision_distance_offset` |
| Blacklist, `planner_node.py:641` | Remove targets within 0.5 m of a previously banned target. | Hardcoded |
| Minimum utility, `planner_node.py:385-390` | Require candidate utility >= threshold. | `min_utility_threshold` |
| Collision inflation, `route_planning.py:54-76` | Inflate occupied voxels by a radius rounded upward to whole voxels. | `collision_radius`, `voxel_resolution` |
| A*, `route_planning.py:7-51,169-184` | Reject occupied start/goal, skip occupied neighbors, and stop after search budget or empty search queue. Unreachable candidate segments are skipped. Search uses six axis-aligned neighbors. | `astar_max_steps`; neighbor structure hardcoded |
| Path validation, `route_planning.py:79-96` | Reject empty/nonfinite routes and any sampled segment intersecting inflated occupancy. | `collision_check_spacing_factor` for planner validation; internal route validation uses default spacing |
| Spline, `route_planning.py:194-217` | Require at least four distinct path points and a valid fit. The main planner also tries the raw A* route, so spline failure alone need not reject the NBV. | `spline_smoothing`, `trajectory_points`, `waypoint_spacing` |
| Final route, `planner_node.py:442-467,496-509` | Publish only collision-free resampled trajectories and a safe connection from the current pose. Reject when no option passes. | Collision spacing, voxel resolution, trajectory sampling |

## Hardcoded geometry that can indirectly cause rejection

`planner_node.py:125,286,291` assumes camera offset `(0.35, 0.3875, -0.5)`, horizontal FOV **90 degrees**, and downward camera pitch **45 degrees**. The offset is added in world coordinates rather than rotated with the vehicle. Incorrect geometry distorts occupied points, visibility estimates, and collision checks. These values are not planner YAML parameters.

## Separate Tools pipeline

The main ROS entry point imports `get_NBV.py`, not `Tools/get_point_cloud.py`. The latter, used by `Tools/mapping_3d.py`, has a separate hardcoded **Z > -32** filter at lines 253-255, a camera depth filter **0.5 < depth < max_depth** at line 25 (called with 4.0 m), and sparsity gating at lines 259-269 requiring more than ten occupied points, normalized sparsity > 0.15, and neighbor distance > 1.2 times resolution. Those thresholds affect runs using that tool rather than `planner_node`.

`z_min_target` and `visit_radius` are declared in the main planner but do not implement the active NBV Z or visited filters. Changing them alone will not disable those filters.
