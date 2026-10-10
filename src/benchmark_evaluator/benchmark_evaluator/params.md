Updated todo list

I’ll map the current ROS parameters to the actual planner/controller code and separate safe joint-search groups from parameters that should be tuned alone. I’ll also check the current Optuna objective so the recommendation matches what it can currently write.

Searched for regex `declare_parameter|parameters\['|self\.[a-zA-Z_]+ = self.declare_parameter` (`HoloSystem` · `NBV_paper_ros2/src/python_planner/python_planner/{planner_node.py,get_NBV.py,route_planning.py}`), 45 results

Read 

Read , lines 25 to 89

Yes. I recommend tuning in stages, not all parameters together.

**Stage 1: NBV decision quality**

Tune together:

```text
min_utility_threshold:       0.05 - 0.35
nbv_standoff:                1.0 - 2.5
nbv_distance_weight:         0.5 - 2.0
nbv_rotation_weight:         0.1 - 1.0
nbv_collision_weight:        1.0 - 5.0
nbv_min_raw_gain:            0.2 - 1.0
```

These directly control viewpoint selection, travel distance, and coverage.

**Stage 2: Frontier detection**

Tune together:

```text
nbv_frontier_angle_threshold_deg: 110 - 160
nbv_cluster_fov_radius:           0.5 - 1.5
nbv_cluster_max_angle_deg:        20 - 60
nbv_cluster_min_samples:          3 - 10
nbv_visited_max_distance:         2.0 - 4.0
nbv_visited_yaw_threshold_deg:    20 - 60
```

**Stage 3: Trajectory generation**

Tune together:

```text
spline_smoothing:             0.5 - 5.0
waypoint_spacing:             0.25 - 0.75
collision_radius:             0.2 - 0.6
arrival_target_radius:        0.75 - 2.0
collision_check_spacing_factor: 0.25 - 1.0
```

**Stage 4: Controller behavior**

Tune separately from planner parameters:

```text
kp_yaw:              0.3 - 0.9
kp_dist:             0.4 - 1.2
yaw_smoothing_rate:  0.3 - 1.5
min_forward_thrust:  0.5 - 2.0
max_forward_thrust:  5.0 - 10.0
command_multiplier:  2.0 - 4.0
```

Do not tune `voxel_resolution`, `depth_scale`, camera offsets, or `z_min_target` with Optuna unless you are calibrating the sensor/map itself.

Currently the optimizer only writes `min_utility_threshold`, so `write_params_yaml()` must be expanded before tuning the other groups.

Created 3 todos