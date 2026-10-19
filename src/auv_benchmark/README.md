# Measured motion control responses

## Permanent planner ban spheres

The benchmark logger incrementally reads `planner_nbv_filters.jsonl` from its
`results_dir` once per second and saves `permanent_ban_spheres.json`. Each sphere
has its original map-coordinate centre, radius in metres, exclusion reason, and
first recorded iteration/frame. The snapshot includes permanent collision bans
(default 1 m), permanent retry bans (default 0.5 m), and retry-budget exclusions reconstructed
from the original attempt history. Temporary cooldowns and observed targets are
excluded. New planner logs clear the previous mission's sphere set; the saved
data is for inspection and does not restore bans into a later planner mission.
Completion/shutdown reports also archive `permanent_ban_spheres_TIMESTAMP.json`.
Keep planner filter logging enabled and use the same results directory for both
the planner and benchmark logger.

Export an already recorded run from the workspace root:

```bash
ros2 run auv_benchmark save_planner_bans --results-dir results
```

`test_env_2.py` refreshes this snapshot from the current planner log beside the
selected map when opened, then loads it. An explicit snapshot is left unchanged.
It draws labelled dark red wire spheres at their actual radii and removes those
regions from the preview's candidates before utility normalization. Override the
snapshot with `--ban-spheres /path/to/permanent_ban_spheres.json`; a missing
snapshot leaves the usual viewer available without spheres.

Tune the planner's `route_collision_blacklist_radius_m` and
`route_blacklist_radius_m` in auv_bringup's `config/params.yaml`, then restart the
planner. The recorder uses the radius and canonical centre emitted by that run;
changing YAML does not resize old saved spheres. Older logs retain their
historical 0.5 m retry-matching fallback.

## Motion response measurements

`run_baselines` observes `/trajectory/waypoint` and `holocean/odom`. It measures
the first XYZ target in the waypoint array, matching the controller's active
command. Repeated commands and yaw-only changes do not reset a position response.
Changing the position ends the previous episode and starts a new one. A command
received before odometry starts its episode at the first valid position sample.
All measurements stay in the evaluator; the controller and simulator are unchanged.

Every received odometry sample updates the response. Live ROS log summaries show:

- Rise time from 10% to 90% projected progress, with interpolated crossings.
- t63: time to 63.2% progress. This is a measured response time, not a fitted
  time constant except for a first-order response.
- Provisional settling time, using full 3D position error inside a band of 2%
  of the initial step distance. Leaving the band revokes the settling time.
- Time continuously observed inside the band and whether the configured dwell
  was met. Dwell confirmation still describes only the observation window.
- Projected overshoot percentage, time of the greatest overshoot, current
  position error, IAE (m*s), and ISE (m²*s).

Progress and overshoot are projected onto the initial-position-to-target line;
cross-track error still affects settling, IAE, and ISE. Zero-distance targets
have no response times. Unreached thresholds remain pending in live logs and
blank/N/A in reports. A target replaced before settling remains an interrupted
observation, not evidence that the controller eventually settled. Closely spaced
trajectory waypoints may therefore have many pending response times.

Waypoint messages contain no timestamp, so elapsed time uses the evaluator's
monotonic callback receipt clock for both commands and odometry. This measures
wall-clock response including delivery delay, not simulator physics time or
command-to-actuator latency. The initial position is the last received odometry;
its age depends on the odometry rate. No nominal plant poles, damping ratios,
estimator performance, or force model are inferred from these observations.

## Saved outputs

On `/movement/finished_execution` with `data: true`, or graceful evaluator
shutdown, the existing `results_dir` also receives:

- `motion_response_metrics.csv`: one row per position-target episode, including
  its initial/target position, end reason, response times, errors, and sample counts.
- `motion_response_samples.csv`: elapsed time, progress, and full/component
  position error for retained samples.
- `motion_response_summary.txt`: readable metrics and measurement definitions.
- `motion_control_response.png`: progress, full error, response times, and
  overshoot for the last 12 valid steps. Missing response times have no bar.

The samples retain at most 3000 observations per episode. Crossings, integrals,
and peaks include all observations even when older chart samples are discarded.
CSV metadata gives total and retained sample counts. No response chart is
generated if no nonzero position step was observed. Existing benchmark outputs
and their completion behavior remain available.

## Live map and planned trajectory

`live_viewer` is a separate ROS node with an Open3D window. Full auv_bringup starts it
by default; use `ros2 launch auv_bringup mapper.launch.py live_viewer:=false` for
headless runs. To attach it to an existing run:

```bash
ros2 launch auv_bringup live_viewer.launch.py
```

Tune the `live_viewer.ros__parameters` section of `auv_bringup/config/params.yaml`.
Build with the Python environment that contains Open3D and `pyoctomap`
(`envHolo` in this workspace): after activating it and sourcing ROS, use
`python -m colcon build --packages-select auv_benchmark auv_bringup --symlink-install`.
This makes the installed node entry point use that environment's interpreter.
Set `map_path` to match the planner's `octomap_path`. The viewer shows a cleaned,
downsampled occupied map, the latest published planned trajectory in cyan, and the
vehicle in orange. The active frontier target has a yellow wire sphere and a
yellow line from the planned viewpoint. Drag to orbit, scroll to zoom, and
press **F** to fit the map.
Closing the window stops only the viewer. A viewer attached mid-run receives
the trajectory on its next publication; route-clear and completion messages
remove the displayed trajectory.

The target overlay follows the C++ planner's existing `inspection_attempt_started`
and cancellation/observation/completion diagnostics. It shows the selected
frontier centroid, rather than recomputing frontier points or showing unselected
candidates. Keep the planner's `nbv_filter_logging` enabled. The viewer reads
`planner_nbv_filters.jsonl` beside `map_path`, or an explicit `frontier_log_path`.
It polls on a background worker at `frontier_poll_period_s` (default 0.5 s),
uploads marker geometry only on changes, and hides it immediately on a live
route clear. Initial attachment searches backwards in bounded 128 KiB blocks;
subsequent reads consume only appended bytes. A missing log leaves the map and
trajectory usable. `show_target_frontiers` disables the overlay and its polling;
`frontier_marker_radius_m` adjusts its display radius (default 0.35 m).

Map updates follow the planner's snapshot saves, normally at planning cycles.
An unchanged file costs only a metadata check. A worker reads a stable copy of
each changed snapshot and computes spatial differences, retaining distant map
regions. Only changed chunks are uploaded; deleted chunks are removed. The
full snapshot still needs decoding because the planner does not publish map
deltas. Occupied leaves are sampled without expanding free-space voxels.
As in `test_env_2.py`, statistical outlier removal follows downsampling:
`filter_outliers` defaults to true, with `outlier_neighbors: 15` and
`outlier_std_ratio: 1.5`. These settings are in bringup's viewer section and
affect only visualization. Set `filter_outliers: false` to inspect unfiltered
occupancy; sparse real surfaces can also be rejected by statistical filtering.
Unlike the offline target preview, the viewer keeps all heights. Filtering runs
once per changed snapshot on the worker, using neighbors across chunk boundaries.
Every snapshot replaces previous display contents, including points that became
free or were filtered out. Downsampled positions retain measured centroids rather
than snapping to artificial voxel centers. Saved maps and planner collision data
are unchanged.
`voxel_size_m` limits display density, `chunk_size_m` sets update granularity,
and `max_chunks_per_frame` bounds geometry uploads per frame. File polling and
render rates are independent. Camera framing is preserved after the first map
load. Partial writes retain the last valid display until a complete snapshot
arrives. Supports the full ColorOcTree/OcTree files written by the planners
(including their `.bt` filenames), rather than compact binary OctoMap exports.

## Evaluator parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `response_band_fraction` | `0.02` | Settling band / initial distance |
| `response_settling_dwell_s` | `1.0` | Observed continuous band dwell for confirmation |
| `response_target_tolerance_m` | `0.00001` | Position-command deduplication tolerance |
| `response_max_samples` | `3000` | Retained samples per response |
| `response_log_period_s` | `1.0` | Live summary interval, also logged at start/end |

These are parameters of the `run_baselines` evaluator node. For example:

```bash
ros2 run auv_benchmark run_baselines --ros-args \
  -p results_dir:=/tmp/controller_evaluation \
  -p response_settling_dwell_s:=2.0
```
