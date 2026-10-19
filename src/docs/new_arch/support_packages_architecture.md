# Support packages

## `auv_contracts`

The shared inner Python package contains immutable configuration, pose, trajectory and message-value snapshots in `values.py`. `ports.py` defines narrow timing, filter, trajectory, map, clock, diagnostics, optimization and trial-runner protocols. `diagnostics.py` converts diagnostics to JSON-compatible values without writing them. `mission_results.py` owns schema version 1 of the existing result filename/distance-text convention. None imports ROS, Torch, OctoMap or filesystem libraries.

`Configuration` freezes nested dictionaries/lists; it does not validate YAML keys. Algorithms still use NumPy arrays where preserving their established representation matters.

## `auv_telemetry`

`file_sinks.py` owns the original CSV/JSONL implementation and narrow `PipelineFileSink` / `PlannerFileSink` facades. Timing consumers no longer import the experiment/evaluation package. The observer uses the trajectory-only logger through the `TrajectorySink` port. File names, formatting and flush behavior are retained.

This component structurally implements the ports; Python `Protocol` does not require explicit inheritance. It has no reverse import into mission/auv_stereo_depth components.

## `auv_experiments`

`trial_policy.py` defines the injected trial objective, retaining six parameter ranges, timeout fitness and the RMSE coverage penalty. `runner_adapter.py` owns ROS process startup/termination, YAML persistence and result reading. It delegates node-key translation to `auv_bringup.configuration`. `optuna_optimizer.py` composes Optuna, the runner and map scoring; use `ros2 run auv_experiments optuna_optimizer` or its package module.

The optimizer moved out of `auv_benchmark`. Python/CLI ownership changes while its trial equations and ROS launch command are preserved. `auv_experiments` depends on auv_bringup/evaluation/contracts; runtime components do not import the optimizer.
