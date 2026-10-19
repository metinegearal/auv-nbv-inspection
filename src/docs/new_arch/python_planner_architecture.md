# `auv_planner_reference` Architecture

## Overview
The `auv_planner_reference` is the brain of the active inspection system. It computes Next-Best-View (NBV) targets, handles 3D A* route planning, and performs geometric trajectory optimization.

## Layer Breakdown

### 1. Domain
*   Rely heavily on the shared `auv_contracts` package for interfaces.
*   **`domain/configuration.py`:** Reads values through an injected parameter reader and returns an immutable `Configuration`. It preserves existing parameter defaults and does not add validation of unknown YAML keys. `domain/nbv_parameters.py` owns the geometric defaults.

### 2. Use Cases (`auv_planner_reference/use_cases/`)
*   **`nbv.py` (`calculate_nbv_waypoints`):** The core algorithmic pipeline. Performs PCA complexity calculation, frontier detection, clustering, and viewpoint generation without touching ROS or OctoMap directly.
*   **`auv_planner_reference/route_planning.py`:** Remains at the package root and contains `astar_3d`, route sequencing, spline generation and collision checks over a plain voxel set.
*   **`trajectory_optimization.py`:** Computes the `continuity_cost` and optimizes splines to prevent sharp AUV reversals.
*   **`mission.py` (`InspectionMission`):** The master state machine. Maintains the latched poses, triggers map accumulation, and decides when to replan or blacklist a failed target.

### 3. Interface Adapters (`auv_planner_reference/adapters/`)
*   **`octomap.py`:** Wraps the `pyoctomap` C++ bindings to fulfill the `OccupancyMap` contract.
*   **`point_cloud.py`:** Uses `open3d` to downsample and estimate normals for point clouds, fulfilling the `PointCloudPreprocessor` contract.
*   **`optimization.py`:** Wraps `scipy.optimize.minimize` for trajectory smoothing.
*   **`planner_node.py`:** The ROS 2 Node. Registers callbacks via `MutuallyExclusiveCallbackGroup`, handles `cv_bridge` conversions, and passes time-synced data to `InspectionMission`.

`adapters/ros_output.py` retains the Float32 wire conversion and returns the reconstructed executed trajectory for policy-side collision validation. `adapters/files.py` owns evaluation-log opening. Compatibility entry points `get_NBV.py` and `trajectory_optimization.py` still construct default adapters when called directly.

The former `Tools` source files were removed separately in the workspace; the refactor does not restore them. Numerical libraries NumPy and SciPy's spatial/interpolation operations remain mathematical dependencies. See the [parameter reference](../../auv_bringup/config/README.md), [runtime mechanics](exploration_pipeline_mechanics.md), and [validation/status report](../architecture/refactoring_status.md).
