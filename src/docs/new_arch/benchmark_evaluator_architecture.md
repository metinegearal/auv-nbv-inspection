# `benchmark_evaluator` Architecture

## Overview
The `benchmark_evaluator` provides offline and mid-run analysis of the AUV's mapping accuracy and trajectory efficiency. 

Previously, this package formed a cyclic deployment dependency by housing the Optuna optimizer. The optimizer has now been moved to `auv_experiments`, and this package focuses purely on metrics.

## Layer Breakdown

### 1. Domain (`benchmark_evaluator/domain/`)
*   **`evaluation.py`:** Defines `MapEvaluationConfig` and the `ReportSnapshot` dataclass, along with `ReportWriter` and `EvaluationGeometry` Protocols.

### 2. Use Cases (`benchmark_evaluator/use_cases/`)
*   **`map_evaluation.py` (`EvaluateMap`):** Computes academic metrics (RMSE, surface coverage percentage) given abstract arrays of distances. 
*   **`trajectory_evaluation.py` (`TrajectoryEvaluation`):** Computes heading errors and RMSE against the nearest sampled planned waypoint. This remains a sample-distance metric, not projection onto continuous path segments.

### 3. Interface Adapters (`benchmark_evaluator/adapters/` & `run_baselines.py`)
*   **`map_geometry.py` (`Open3DMapEvaluation`):** Uses `Open3D` and `pyoctomap` to load the Ground Truth `.obj` mesh and the generated `.bt` map, run ICP alignment, and compute point-cloud distances.
*   **`reports.py` (`FileReportWriter`):** Handles CSV writing and triggers the Matplotlib visualization.
*   **`run_baselines.py` (`BenchmarkLogger`):** The ROS 2 Node that listens to odometry and waypoints, passing them into the `TrajectoryEvaluation` use case.

The observer also listens to `/movement/finished_execution` and saves partial results on shutdown. `plot_adapter.py` owns visualization. The existing `optuna_mission_stats.txt` filename and distance text are shared through `auv_contracts/mission_results.py` schema version 1; no on-disk format changed. Experiment objective policy and process/config adapters now live in `auv_experiments`, whose dependencies point toward bringup/evaluation without a reverse launcher dependency.
