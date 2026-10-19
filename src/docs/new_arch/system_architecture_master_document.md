# Global AUV System Architecture

## Overview
The active AUV pipelines now separate mission/math policy from ROS 2, simulator APIs, model loading and storage. Policy uses owned contracts; composition roots supply concrete implementations. This document describes the implemented boundaries rather than certifying that every historical audit finding is closed. See the [finding dispositions and validation evidence](../architecture/refactoring_status.md).

Instead, all dependencies point inward toward policy-owned contracts and domain entities.

## Global Dependency Topology

The runtime telemetry/experiment cycle was removed by extracting `auv_contracts`, `auv_telemetry`, and `auv_experiments`. There are ten ROS packages: the original seven plus those three. The default auv_bringup launches the Python planner; `auv_inspection_planner` remains an alternative.

```mermaid
graph TD
    subgraph Composition Root
        BR[auv_bringup] -. launches .-> SB[auv_sim_bridge]
        BR -. launches .-> PE[auv_stereo_depth]
        BR -. launches .-> MC[auv_motion_control]
        BR -. launches .-> PP[auv_planner_reference]
        BR -. launches .-> BE[auv_benchmark]
        EXP[auv_experiments] -. orchestrates .-> BR
    end

    subgraph Interface Adapters
        SB_ADAPT[auv_sim_bridge/adapters]
        PE_ADAPT[auv_stereo_depth/adapters]
        MC_ADAPT[auv_motion_control/adapters]
        PP_ADAPT[auv_planner_reference/adapters]
        BE_ADAPT[auv_benchmark/adapters]
        AIP_ADAPT[auv_inspection_planner/adapters]
        TEL[auv_telemetry]
    end

    subgraph Core Use Cases
        SB_UC[auv_sim_bridge/use_cases]
        PE_UC[auv_stereo_depth/use_cases]
        MC_UC[auv_motion_control/use_cases]
        PP_UC[auv_planner_reference/use_cases]
        BE_UC[auv_benchmark/use_cases]
        AIP_UC[auv_inspection_planner/use_cases]
    end

    subgraph Shared Domain & Contracts
        CON[auv_contracts]
        SB_DOM[auv_sim_bridge/domain]
        PE_DOM[auv_stereo_depth/domain]
        PP_DOM[auv_planner_reference/domain]
        BE_DOM[auv_benchmark/domain]
        MC_DOM[auv_motion_control/domain]
        AIP_DOM[auv_inspection_planner/domain]
    end

    %% Dependency arrows (always pointing inward)
    SB_ADAPT --> SB_UC
    PE_ADAPT --> PE_UC
    MC_ADAPT --> MC_UC
    PP_ADAPT --> PP_UC
    BE_ADAPT --> BE_UC
    AIP_ADAPT --> AIP_UC

    SB_UC --> CON
    SB_UC --> SB_DOM
    PE_UC --> CON
    PE_UC --> PE_DOM
    PP_UC --> CON
    PP_UC --> PP_DOM
    BE_UC --> CON
    BE_UC --> BE_DOM
    TEL -. implements .-> CON

    MC_UC --> MC_DOM
    AIP_UC --> AIP_DOM
    EXP --> CON
    EXP --> BE
```

Solid edges summarize aggregated source dependencies; dotted edges show launch/orchestration or structural protocol implementation. Adapters also depend directly on domain contracts. This is not a ROS topic graph; see [runtime mechanics](exploration_pipeline_mechanics.md).

## The Three Rings of the System

### 1. The Inner Ring: Domain Entities & Contracts
This ring contains no framework dependencies. It represents the absolute core truths of the AUV system.
* **`auv_contracts`:** A shared Python package defining pure `Protocol` interfaces (e.g., `TrajectorySink`, `Clock`, `OccupancyMap`) and immutable `dataclass` values (e.g., `PoseSample`, `Trajectory`).
* **`auv_motion_control/domain` (C++):** Defines `ControlConfig`, `MotorCommand`, and pure virtual interfaces like `ThrusterMixer` and `CommandOutput`.
* **`auv_inspection_planner/domain` (C++):** Defines immutable image/config/pose values, map/spatial-index ports, `VoxelIdx`, and `SearchPolicy` mathematics. Eigen remains a mathematical dependency.

### 2. The Middle Ring: Use Cases
This ring contains the application-specific business logic. It orchestrates data flow using the interfaces defined in the Inner Ring.
* **`auv_planner_reference/use_cases`:** Calculates Next-Best-View (NBV), optimizes trajectories, and manages the inspection mission state machine.
* **`auv_motion_control/use_cases`:** Computes PID control laws and manages the active waypoint queue.
* **`auv_benchmark/use_cases`:** Evaluates nearest-planned-sample trajectory error and map-distance RMSE / coverage against ground truth.

### 3. The Outer Ring: Interface Adapters & Frameworks
This ring interacts with the outside world. It translates ROS 2 messages, disk files, and external APIs into the pure data formats required by the Use Cases.
* **ROS 2 Nodes:** E.g., `planner_node.py`, `waypoint_control_node.cpp`. These simply unpack `sensor_msgs` or `nav_msgs`, pass data to Use Cases, and pack the results back into messages.
* **Framework Adapters:** `adapters/bgnet.py` (PyTorch), `adapters/octomap.py` (pyoctomap), `adapters/holoocean.py` (Simulator API).
* **`auv_telemetry`:** Concrete implementations of the logging contracts (writing to JSONL/CSV).
* **`auv_experiments`:** Optuna process runner that overrides configurations and spins up the system without injecting experiment logic into the core planner.

## Composition, builds and documentation

`auv_bringup/auv_bringup/configuration.py` owns translation from trial parameters to deployment YAML keys. `auv_contracts/mission_results.py` owns the existing mission-result filename/text convention shared by the observer and trial runner. The format remains unchanged.

The C++ libraries are `motion_policy`, `motion_platform_adapters`, `inspection_policy`, and `inspection_map_adapters`. ROS executables compose them; the two policy libraries build without ROS headers. Python domain/use-case imports are checked for framework and IO leaks. Compatibility facades are outer entry points and may construct default adapters.

Start with the [documentation index](README.md), [auv_bringup architecture](bringup_architecture.md), and [complete parameter reference](../../auv_bringup/config/README.md). Validation covers all ten package builds, Python policy/regression checks, C++ policy tests and a baseline mapping comparison. It does not replace a full HoloOcean/Torch mission replay.
