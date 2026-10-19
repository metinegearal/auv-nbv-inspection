# `auv_bringup` architecture

`launch/mapper.launch.py` is the deployment composition root. It launches `auv_sim_bridge`, `auv_stereo_depth`, both `auv_motion_control` executables, `auv_planner_reference`, and the `auv_benchmark` observer. It does not launch the alternative C++ planner or an Optuna optimizer.

`package.xml` declares launch/runtime dependencies. `setup.py` installs the launch file, both YAML profiles, the active scenario JSON and its parameter reference. Launch names determine which ROS YAML sections apply.

`auv_bringup/configuration.py` owns trial-override translation into deployment YAML: planner values, planner map/output paths and observer output directory. It is pure. `auv_experiments/runner_adapter.py` owns YAML IO/process management and calls that translation, so deployment node keys have one owner.

```mermaid
graph TD
    EXP[auv_experiments runner adapter] --> CFG[auv_bringup configuration translation]
    EXP -. launches .-> LAUNCH[auv_bringup mapper.launch.py]
    LAUNCH -. launches .-> RUNTIME[bridge, auv_stereo_depth, control, Python planner, observer]
    RUNTIME --> CONTRACTS[policy-owned contracts]
```

There is no observer-to-auv_bringup launcher dependency. See the [parameter reference](../../auv_bringup/config/README.md) for arguments, precedence, node names and simulator fields.
