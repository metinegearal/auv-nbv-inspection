# Bringup configuration

- `params.yaml`: controller, map, route, trajectory, logging, and evaluation settings.
- `nbv_params.yaml`: NBV geometry estimation, frontier clustering, viewpoint sampling, observation history, and scoring weights.
- `holoocean_config.json`: simulator configuration.

`ros2 launch bringup mapper.launch.py` loads both parameter YAML files automatically. To use another NBV tuning profile:

```bash
ros2 launch bringup mapper.launch.py nbv_params_file:=/absolute/path/my_nbv_params.yaml
```

Custom NBV files use the same `planner_node: ros__parameters:` structure. Missing values use the defaults in `get_NBV.py`. If a key also exists in the main/custom `params_file`, that file takes precedence, preserving existing combined parameter files.

`nbv_complexity_weight` controls the PCA complexity contribution to gain. Setting it to `0.0` removes that contribution; `15.0` retains the previous score. `nbv_pca_neighbors` controls the local neighborhood used to estimate complexity. The distance, rotation, and collision weights affect kinematic cost.

The flat-horizontal, relative-geometry, and visited-discount settings from the reverted NBV implementation are omitted because the current planner does not declare or use them.

Build `bringup` after adding configuration files. Restart the planner to load changed weights; these parameters are read at startup.
