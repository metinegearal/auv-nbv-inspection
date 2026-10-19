# `auv_motion_control` Architecture

## Overview
The `auv_motion_control` package separates ROS 2 middleware and platform-specific thruster allocation from trajectory queueing and proportional waypoint guidance. The PID class remains a reusable mathematical utility; the active guidance law uses the existing proportional yaw/distance gains.

## Layer Breakdown

### 1. Domain (`include/auv_motion_control/domain/`)
*   **`contracts.hpp`:** Defines the pure C++ interfaces required by the control logic (`CommandOutput`, `WaypointOutput`, `ThrusterMixer`, `Diagnostics`).
*   **`math_pid_utils.hpp`:** Contains purely mathematical utilities (quaternion conversion, 2D/3D distance) and a framework-agnostic `PID` controller class.

### 2. Use Cases (`include/auv_motion_control/use_cases/`)
*   **`waypoint_controller.hpp`:** Computes heading errors, applies proportional gains, determines directional movement, and calls the injected `ThrusterMixer` to allocate command contributions. It uses standard C++ values and policy-owned ports.
*   **`trajectory_queue.hpp`:** Manages the sequential execution of waypoints. It pops waypoints when reached and triggers a request for a new trajectory when empty.

### 3. Interface Adapters (`include/auv_motion_control/adapters/` & `src/`)
*   **`ros_control_io.hpp`:** Implements `CommandOutput` and `Diagnostics` by wrapping `rclcpp::Publisher` calls.
*   **`thrusters.hpp` (`EightThrusterMixer`):** A platform-specific implementation of `ThrusterMixer` that maps directional keys and yaw contributions to the existing eight simulator command values. These values are not PWM signals. `ConfiguredModes` supplies the existing mode settings through the `Modes` port.
*   **`waypoint_control_node.cpp` & `point_iterator.cpp`:** The ROS 2 Composition Roots. They subscribe to topics, unpack ROS messages into domain `structs`, and execute the Use Cases.

`motion_policy` and `motion_platform_adapters` are separate CMake libraries. The policy library has no ROS dependency. The queue still publishes a flattened four-value waypoint format and retains its route on an empty input array. See [runtime mechanics](exploration_pipeline_mechanics.md) for that preserved limitation.

The unbuilt `src/PathPlanning.py` is a legacy 2D node, not part of the launched C++ controller. Its optional exploration/distance dependencies now have ports and a dedicated adapter; no absolute `sys.path` mutation remains. Its external repository helpers must be importable normally to run that dormant script.
