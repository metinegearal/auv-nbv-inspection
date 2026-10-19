# Bringup parameter reference

`params.yaml` configures the launched nodes. `nbv_params.yaml` supplies the Python planner's geometric scoring profile. `holoocean_config.json` is passed to HoloOcean as a scenario, rather than parsed as ROS parameters. `holoocean_config copy.json` is an alternate local scenario; the launch file does not load it automatically.

The tables distinguish **configured values** in the checked-in YAML from **code defaults** used when a value is omitted. This document describes existing behavior; it does not change tuning or algorithms.

## Loading, precedence and node names

```bash
ros2 launch bringup mapper.launch.py
ros2 launch bringup mapper.launch.py params_file:=/absolute/path/params.yaml nbv_params_file:=/absolute/path/nbv.yaml holo_config_file:=/absolute/path/scenario.json
```

| Launch argument | Default installed resource | Consumers |
|---|---|---|
| `params_file` | `bringup/config/params.yaml` | All six launched processes |
| `nbv_params_file` | `bringup/config/nbv_params.yaml` | Python planner only |
| `holo_config_file` | `bringup/config/holoocean_config.json` | Bridge's `holoocean_config_path` parameter |

The planner receives `[nbv_params_file, params_file]`; the main file wins when both define the same key. The bridge receives the main file and then the explicit scenario-path override. YAML keys use **launch names**: `waypoint_control_node`, `point_iterator`, `planner_node`, `run_baselines`, `depth_estimation`, and `holoocean_node`. Standalone constructors have different names for the queue (`path_planner`), planner (`nbv_utility_node`), observer (`benchmark_logger`), and depth node (`object_detector_node`); supply matching node keys or remap the name when running an executable directly.

Parameters are read at startup except `kp_yaw` and `kp_dist`, which the controller re-reads during movement. Rebuild `bringup` to refresh installed resources, then restart nodes to apply other edits. Relative result paths resolve against the launched process's working directory.

## `waypoint_control_node`

Source: [ROS composition](../../motion_control/src/waypoint_control_node.cpp), [controller policy](../../motion_control/include/motion_control/use_cases/waypoint_controller.hpp).

| Parameter | Configured | Code default | Meaning / units |
|---|---|---|---|
| `kp_yaw` | `0.5` | `0.6` | Proportional gain on wrapped yaw error in radians, before yaw clamp and output multiplier. |
| `kp_dist` | `0.75` | `0.75` | Gain on horizontal distance and absolute vertical error in metres. |
| `dvl_active` | `true` | `false` | Enables the existing DVL-position path; `/movement/dvl_reset` can also activate it. |
| `stopping_distance` | `0.4` | `0.25` | Waypoint completion tolerance in metres. |
| `initial_waypoint_x` | `354.0` | `10.0` | Initial target X in metres. |
| `initial_waypoint_y` | `828.0` | `10.0` | Initial target Y in metres. |
| `initial_waypoint_z` | `-298.0` | `5.0` | Initial target Z in metres. |
| `initial_waypoint_yaw` | `0.0` | Not declared | Retained YAML key; the controller does not read it. Initial queue yaw is owned by `point_iterator`. |
| `max_yaw_command` | `10.0` | `10.0` | Maximum absolute yaw contribution before `command_multiplier`. |
| `yaw_smoothing_rate` | `0.8` | `0.8` | Maximum desired-yaw change rate in radians per second. |
| `min_forward_thrust` | `1.0` | `1.0` | Lower clamp on base horizontal thrust before heading-dependent scaling. |
| `max_forward_thrust` | `8.0` | `10.0` | Upper clamp on base horizontal and vertical thrust. |
| `command_multiplier` | `5.0` | `2.0` | Final multiplier applied to all eight command values. |

Thrust values are simulator command units, not PWM or a calibrated force measurement. Horizontal thrust is also multiplied by `max(0.1, cos(yaw_error))`. The default mode remains `yaw_centerize`; its ROS mode subscription is commented out. The refactor moves allocation to `EightThrusterMixer` and retains the eight-channel ordering.

## `point_iterator`

Source: [ROS adapter](../../motion_control/src/point_iterator.cpp), [queue policy](../../motion_control/include/motion_control/use_cases/trajectory_queue.hpp).

| Parameter | Configured | Code default | Meaning / units |
|---|---|---|---|
| `camera_x` | `0.35` | `0.35` | X offset subtracted from each incoming camera-space waypoint; metres. |
| `camera_y` | `0.3875` | `0.1125 + 0.275` | Y offset subtracted from incoming waypoints; metres. |
| `camera_z` | `-0.5` | `-0.5` | Z offset subtracted from incoming waypoints; metres. |
| `initial_waypoint_x` | `354.0` | `-16.0` | X of the initial queue entry; metres. |
| `initial_waypoint_y` | `828.0` | `-81.0` | Y of the initial queue entry; metres. |
| `initial_waypoint_z` | `-298.0` | `-30.0` | Z of the initial queue entry; metres. |
| `initial_waypoint_yaw` | `0.0` | `0.0` | Initial queue yaw in radians. |
| `timer_period_s` | `1.0` | `1.0` | Queue republish period in seconds; new routes and completion events also publish immediately. |
| `waypoint_reached_distance` | `0.5` | `0.4` | XY distance threshold for removing the first queue entry after a completion report. |

The queue consumes flattened `[x,y,z,yaw]` groups. It ignores incomplete trailing groups and retains its current route when the received array is empty. This existing behavior is also described in [pipeline mechanics](../../docs/new_arch/exploration_pipeline_mechanics.md).

## `planner_node`: map, mission and route settings

Source: [configuration loader](../../python_planner/python_planner/domain/configuration.py), [mission policy](../../python_planner/python_planner/use_cases/mission.py), [route mathematics](../../python_planner/python_planner/route_planning.py).

| Parameter | Configured | Code default | Meaning / units |
|---|---|---|---|
| `voxel_resolution` | `0.2` | `0.1` | OctoMap / route voxel size in metres. |
| `max_depth` | `5.0` | `3.0` | Strict upper bound on accepted depth in metres. |
| `min_depth` | `0.5` | `0.5` | Strict lower bound on accepted depth in metres. |
| `z_min_target` | `-300.0` | `-3000.0` | World-Z filter for depth accumulation; separate from `nbv_z_min_target`. |
| `depth_stride` | `4` | `8` | Pixel sampling stride for map insertion. |
| `depth_scale` | `1.0` | `1.0` | Multiplier converting received depth values into the mapper's depth units. |
| `planning_period` | `100` | `10` | Accepted map-update iterations between periodic planning attempts; internal attribute `frame_period`. |
| `replan_on_target_arrival` | `true` | `false` | Also trigger planning near a selected inspection target. |
| `arrival_target_radius` | `2.5` | `1.5` | Distance to the selected target centroid that triggers arrival replanning; metres. |
| `min_utility_threshold` | `0.1` | `0.005` | Keep candidate utilities at or above this threshold. |
| `route_max_candidates` | `5` | `5` | Maximum highest-utility candidates passed to route planning after utility filtering; positive integer, read at startup. |
| `collision_radius` | `0.3` | `0.6` | Occupancy inflation radius in metres. |
| `collision_tracking_margin` | `0.4` | `0.4` | Additional inflation allowance in metres; added to `collision_radius`. |
| `trajectory_points` | `300` | `300` | Requested spline sample count before later trajectory processing. |
| `waypoint_spacing` | `0.5` | `0.5` | Distance-based resampling spacing in metres. |
| `trajectory_optimization_enabled` | `false` | `false` | Enables optional injected SciPy trajectory optimization. A* and spline generation still run when false. |
| `velocity_history_window` | `1.0` | `1.0` | Odometry history window for velocity estimation; seconds. |
| `trajectory_history_window` | `5.0` | `5.0` | Recent motion history supplied to continuity evaluation; seconds. |
| `trajectory_continuity_weight` | `8.0` | `8.0` | Continuity-cost weight for alignment with current motion. |
| `trajectory_reversal_weight` | `4.0` | `4.0` | Reversal penalty weight. |
| `trajectory_history_weight` | `2.0` | `2.0` | Recent-history penalty weight. |
| `trajectory_history_radius` | `0.5` | `0.5` | Radius used by the history cost; metres. |
| `trajectory_max_deviation` | `0.5` | `0.5` | Maximum allowed optimization displacement from the input path; metres. |
| `trajectory_optimizer_max_iterations` | `40` | `40` | Solver iteration limit. |
| `octomap_path` | `results/final_map_ros2.bt` | Same | Map snapshot output path. |
| `visit_radius` | `0.5` | `0.25` | Declared and stored legacy setting; the current mission does not use it to filter candidates. |
| `results_dir` | `results` | `results` | Root directory for telemetry and mission text logs. |
| `use_incremental_map_cache` | `true` | `true` | Maintains occupancy / inflation caches incrementally; false rebuilds the collision set. |
| `nbv_filter_logging` | `true` | `true` | Emits candidate/filter details to diagnostics and JSONL telemetry. |
| `astar_max_steps` | `50000` | `50000` | Per-search A* step bound. |
| `route_blacklist_cooldown_s` | `30.0` | `30.0` | Temporary failed-target ban duration; nonpositive values disable target bans. |
| `spline_smoothing` | `2.0` | `2.0` | Spline smoothing factor passed to trajectory generation. |
| `collision_check_spacing_factor` | `0.5` | `0.5` | Collision sample spacing relative to voxel resolution. |

The ROS timer period is fixed at `0.5` seconds. `planning_period=100` is not a timer period: iteration progresses only after a new depth frame passes the input gates. Forced replanning still waits for those gates. Pose matching accepts the closest odometry sample only when the timestamp difference is strictly below `0.1` seconds. The current route function has no declared `route_search_time_budget_s` parameter; adding that YAML key does not enable such a limit.

## `planner_node`: `nbv_params.yaml`

Defaults: [domain/nbv_parameters.py](../../python_planner/python_planner/domain/nbv_parameters.py). Policy: [use_cases/nbv.py](../../python_planner/python_planner/use_cases/nbv.py). Preprocessing: [adapters/point_cloud.py](../../python_planner/python_planner/adapters/point_cloud.py).

| Parameter | Configured | Code default | Meaning / units |
|---|---|---|---|
| `nbv_z_min_target` | `-300.0` | `-3000.0` | Strict world-Z lower bound on occupied points entering NBV scoring. |
| `nbv_exploration_radius` | `15.0` | `15.0` | Positive finite focus radius around the AUV in metres; full-cloud neighbors and obstacle context remain available. |
| `nbv_voxel_downsample_size` | `0.25` | `0.25` | Open3D downsample voxel size in metres. |
| `nbv_outlier_neighbors` | `15` | `15` | Statistical outlier-filter neighbor count. |
| `nbv_outlier_std_ratio` | `1.5` | `1.5` | Statistical outlier-filter standard-deviation ratio. |
| `nbv_pca_neighbors` | `20` | `20` | PCA neighborhood size for surface complexity. |
| `nbv_normal_radius` | `0.4` | `0.4` | Normal-estimation search radius in metres. |
| `nbv_normal_max_neighbors` | `30` | `30` | Normal-estimation neighbor cap. |
| `nbv_frontier_neighbors` | `40` | `40` | Frontier angular-gap neighborhood size. |
| `nbv_frontier_angle_threshold_deg` | `130.0` | `130.0` | Angular-gap threshold in degrees. |
| `nbv_cluster_fov_radius` | `1.0` | `1.0` | Spatial clustering radius in metres. |
| `nbv_cluster_max_angle_deg` | `45.0` | `45.0` | Cluster normal-angle tolerance in degrees. |
| `nbv_cluster_min_samples` | `5` | `5` | Minimum samples required to retain a frontier cluster. |
| `nbv_standoff` | `1.5` | `1.5` | Cone viewpoint distance from the cluster centroid; metres. |
| `nbv_polar_angles_deg` | `[15.0, 30.0]` | Same | Cone polar sampling angles in degrees. |
| `nbv_azimuth_steps` | `8` | `8` | Azimuth samples per polar angle. |
| `nbv_visited_min_distance` | `0.1` | `0.1` | Lower distance bound in observation-history matching; metres. |
| `nbv_visited_max_distance` | `3.5` | `3.0` | Upper observation-history distance bound; metres. |
| `nbv_visited_yaw_threshold_deg` | `40.0` | `30.0` | Bearing / historical-yaw match tolerance in degrees. |
| `nbv_complexity_weight` | `30.0` | `15.0` | PCA-complexity contribution to incidence-weighted gain. `0` removes this contribution. |
| `nbv_min_raw_gain` | `0.5` | `0.5` | Minimum raw information gain. |
| `nbv_distance_weight` | `1.0` | `1.0` | Travel-distance contribution to kinematic cost. |
| `nbv_rotation_weight` | `0.5` | `0.5` | Required-turn contribution to kinematic cost. |
| `nbv_collision_weight` | `2.0` | `2.0` | Nearby-obstacle penalty contribution. |
| `nbv_collision_distance_threshold` | `0.8` | `0.8` | Distance below which the obstacle penalty applies; metres. |
| `nbv_collision_distance_offset` | `0.2` | `0.1` | Denominator offset in the nearby-obstacle penalty; metres. |

NBV preprocessing requires at least 40 retained points. Gains and costs are normalized across the candidate batch before computing `utility = normalized_gain - normalized_cost`. Changing one candidate's score can therefore change other candidates' normalized utilities. Unsupported parameters from reverted implementations are not read.

## Perception and evaluation

| Node / parameter | Configured | Code default | Meaning |
|---|---|---|---|
| `depth_estimation.sync_queue_size` | `1` | `1` | Approximate stereo synchronizer queue length. |
| `depth_estimation.sync_slop_s` | `0.02` | `0.02` | Allowed stereo timestamp separation in seconds. |
| `depth_estimation.results_dir` | `results` | `results` | Perception timing-log directory. |
| `run_baselines.results_dir` | `results` | `results` | Observer CSV, text and figure output directory. |
| `run_baselines.show_reference_objects` | `false` | `true` | Draws reference geometry on the trajectory overview. |
| `holoocean_node.holoocean_config_path` | Launch-supplied JSON path | Empty string | Simulator scenario file loaded by the bridge. |

The depth node retains the fixed focal length `256` pixels and stereo baseline `0.5` metres. Neither is a declared ROS parameter. Its existing image conversion order, stereo input order, and source timestamp are preserved. Offline map scoring uses its own `MapEvaluationConfig`; its ground-truth path, ICP settings and coverage threshold are not configured by this ROS YAML.

## HoloOcean scenario

The bridge forwards the JSON scenario to HoloOcean. The following are the checked-in values, not assertions about simulator defaults.

| Scenario field | Value | Purpose |
|---|---|---|
| `name` | `test_rgb_camera` | Scenario identifier. |
| `world` / `package_name` | `OpenWater` / `Ocean` | Simulator world and environment package. |
| `main_agent` / `agents[0].agent_name` | `auv0` | Main vehicle name; the current adapter also applies currents to `auv0`. |
| `ticks_per_sec` | `30` | Simulator tick-rate configuration. |
| `agent_type` / `control_scheme` | `HoveringAUV` / `0` | Vehicle and eight-channel command scheme. |
| Agent `location` / `rotation` | `[350,828,-298]` / `[0,0,0]` | Initial simulator pose. |
| DVL | `Elevation=22.5`, `VelSigma=0.001`, `ReturnRange=true`, `MaxRange=50.0`, `RangeSigma=0.05`, `DebugLines=false` | DVL sensor geometry/noise/range settings. |
| Depth / magnetometer noise | `Sigma=0.05` / `Sigma=0.0003` | Sensor noise settings. |
| IMU | `AngVelSigma=0.0035`, `AccelSigma=0.03`, `AngVelBiasSigma=0.001`, `AccelBiasSigma=0.01` | Angular / acceleration noise and bias settings. |
| Range finder | Left camera socket, rotation `[0,45,0]`, location `[0,0.275,-0.5]`; max distance `30`, count `1`, angle `0`, debug `false` | Single-beam range sensor placement/settings. |
| Stereo cameras | `DownCamera` on `CameraLeftSocket`; `FrontCamera` on `CameraRightSocket` | Names consumed by the sensor dispatch registry. |
| Camera placement | Both rotation `[0,45,0]`; locations `[0,0.275,-0.5]` and `[0,0,-0.5]` | Existing socket-relative transforms. |
| Camera geometry | `CaptureWidth=512`, `CaptureHeight=512`, `FovAngle=90.0` | Image size and field of view. |

Both cameras retain identical rendering settings: `TargetGamma=1.0`, `ExposureMethod=AEM_Histogram`, `ExposureCompensation=1`, `ShutterSpeed=60.0`, `ISO=800.0`, `Aperature=2.0` (the existing JSON spelling), `FocalDistance=100.0`, `DepthBlurAmount=1.2`, `DepthBlurRadius=1.0`, `BladeCount=6`, `DepthOfFieldMinFstop=1.4`, `WhiteTemp=4800.0`, `WhiteTint=0.25`, `BloomIntensity=1.8`, `LensFlareIntensity=1.0`, `ExposureSpeedDown=3.0`, and `ExposureSpeedUp=2.0`. These are simulator-owned rendering fields. `LocationSensor` has no explicit configuration block.

## C++ planner alternative

`mapper.launch.py` launches the Python planner. The C++ `active_inspection_planner` executable is an alternative and is not started by that launch file. Its node is also called `nbv_utility_node` when run standalone. It declares `voxel_resolution=0.1`, `max_depth=5.0`, `min_depth=0.5`, `z_min_target=-32.0`, `info_gain_threshold=0.05`, `sparsity_multiplier=0.8`, and `min_utility_threshold=0.005`. These node defaults override the corresponding `MapperConfig` aggregate defaults. A YAML section named `planner_node` will not automatically configure this standalone node.
