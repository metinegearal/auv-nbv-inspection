# `active_inspection_planner` Architecture (C++)

## Overview
The `active_inspection_planner` is a C++ alternative to the Python NBV planner. Its core A* and mapping policy can compile independently of ROS.

## Layer Breakdown

### 1. Domain (`include/active_inspection_planner/domain/`)
*   **`contracts.hpp`:** Defines `MapperConfig`, `MapReader`, `MapWriter`, and `SpatialIndex`. These interfaces completely abstract away OctoMap and PCL.
*   **`domain/search.hpp` & `src/search_algo.cpp`:** Define `VoxelIdx`, `VoxelHash`, and `SearchPolicy` A* / line-of-sight mathematics over voxel sets and Eigen vectors. Failure diagnostics are injected. The old `src/search_algo.hpp` API forwards through a compatibility adapter that preserves stderr output.

### 2. Use Cases (`include/active_inspection_planner/use_cases/`)
*   **`mapping.hpp` (`MapUpdate`, `RoutePlanner`, `CandidatePlanner`):** 
    *   `MapUpdate` handles pinhole camera projection and matrix rotations.
    *   `CandidatePlanner` queries the abstract `SpatialIndex` to find frontiers and evaluate information gain.
    *   `RoutePlanner` uses the existing greedy waypoint ordering and A* routing, then applies Catmull-Rom splines. It does not solve an exact TSP.
*   **`mission.hpp` (`InspectionMission`):** The C++ state machine controlling iteration ticks, coordinate latching, and safety aborts.

### 3. Interface Adapters (`include/active_inspection_planner/adapters/`)
*   **`include/active_inspection_planner/adapters/maps.hpp` (`OctomapAdapter`, `PclIndex`):** Concrete implementations of the domain contracts. `OctomapAdapter` wraps `octomap::ColorOcTree`, while `PclIndex` wraps `pcl::KdTreeFLANN`. `src/adapters/maps.cpp` builds the adapter library.
*   **`mission_io.hpp`:** Implementations for `RosDiagnostics` (wrapping `RCLCPP_INFO`) and `FileMissionMetrics`.
*   **`nbv_utility_node.cpp`:** The ROS 2 Composition Root.

`inspection_policy` depends on Eigen and the standard library. `inspection_map_adapters` adds OctoMap/PCL; the ROS executable adds ROS/OpenCV. Image conversion preserves the original BGR ordering and Float32 depth interpretation. The default launch uses `python_planner`; this executable is an alternative. Mapping-equivalence checks compare the original and extracted implementations, including byte-identical map output for the tested frames.
