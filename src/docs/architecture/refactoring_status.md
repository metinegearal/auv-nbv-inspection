# Clean Architecture refactoring status

Scope: all seven original packages in `NBV_paper_ros2/src`. Three support packages were added: `auv_contracts`, `telemetry_adapters`, and `auv_experiments`. The original audit reports remain historical snapshots; this report describes implementation and validation rather than rewriting their evidence.

## Implemented boundaries

| Component | Policy / domain | Outer implementation |
|---|---|---|
| Python planner | Immutable configuration/pose values; injected mission, NBV scoring and optimization | Thin ROS node; OctoMap, Open3D, SciPy, text-file and trajectory-output adapters |
| C++ planner | Map/image/pose contracts; map projection, candidate and route policies; mission | OctoMap/PCL adapters; ROS/OpenCV conversion, metrics-file and output adapters |
| Motion control | Immutable config/input/command values; queue, guidance, clock/gain/mode/output/mixer ports | ROS nodes and eight-thruster platform mixer |
| Perception | Calibration values and disparity-estimator port; metric-depth policy | Torch/checkpoint/image-preprocessing adapter and synchronized ROS node |
| Simulation bridge | Command/session/dispatch/current-field contracts; stepping and vortex policy | Scenario/session, sensor-dispatch and ROS message adapters |
| Benchmark evaluator | Immutable report/config snapshots; trajectory/map metric use cases | ROS observer, map/ICP framework adapter, file reports and plotting |
| Bringup | Deployment-owned trial-override translation | Launch composition and installed configuration resources |

`motion_policy`, `motion_platform_adapters`, `inspection_policy`, and `inspection_map_adapters` are separate CMake targets. Shared Python telemetry ports have narrow consumer roles; their file implementation no longer lives in the evaluation component. Experiment orchestration moved outward to `auv_experiments`.

See [implemented topology and I/A/D metrics](refactoring_topology.md), [architecture documentation](../new_arch/README.md), [parameters](../../bringup/config/README.md), and the [complete source bundle](refactoring_complete_code.md). The ten-package dependency graph has no detected cycles or strictly increasing-I edges under its documented counting scope. Distance classifications remain heuristic; concrete adapters/value types do not become architectural violations solely because their release package lies below the main sequence.

## Audit finding dispositions

“Implemented” means the structural boundary identified in the audit was changed; it is not a claim that every runtime behavior has been exhaustively replayed. Protocol and numerical conventions explicitly retained under the no-behavior-change rule are called out below.

| Package / finding | Disposition | Implementation / retained constraint |
|---|---|---|
| [active_inspection_planner V-01](packages/active_inspection_planner_audit.md#v-01) | Implemented | Mapper responsibilities split into `MapUpdate`, `CandidatePlanner`, `RoutePlanner`, owned map/image ports and adapters. |
| [active_inspection_planner V-02](packages/active_inspection_planner_audit.md#v-02) | Implemented | Map leaves and spatial searches use `MapReader` / `SpatialIndexes`, implemented with OctoMap/PCL outside policy. |
| [active_inspection_planner V-03](packages/active_inspection_planner_audit.md#v-03) | Implemented | Map storage/console output are adapters; existing calibration values remain unchanged. |
| [active_inspection_planner V-04](packages/active_inspection_planner_audit.md#v-04) | Implemented | Mission policy is injected into a ROS input/composition node. |
| [active_inspection_planner V-05](packages/active_inspection_planner_audit.md#v-05) | Implemented | Separate policy and mapping-adapter libraries are exported/installed. |
| [active_inspection_planner V-06](packages/active_inspection_planner_audit.md#v-06) | Implemented | Direct ROS/math/mapping dependencies declared in build and package metadata. |
| [motion_control V-01](packages/motion_control_audit.md#v-01) | Implemented | Guidance uses injected clock, gains, diagnostics, output, mode selection and mixer ports. |
| [motion_control V-02](packages/motion_control_audit.md#v-02) | Implemented | Mode settings and eight-channel allocation are outer implementations of policy-owned ports. |
| [motion_control V-03](packages/motion_control_audit.md#v-03) | Implemented | Queue behavior extracted into `TrajectoryQueue`; ROS timing/publication remain in its node. |
| [motion_control V-04](packages/motion_control_audit.md#v-04) | Boundary implemented; format retained | Immutable `WaypointInput` and output ports isolate ROS types. The existing four-coordinate convention remains a cross-language protocol contract; changing it still requires coordinated adapters. |
| [motion_control V-05](packages/motion_control_audit.md#v-05) | Implemented for dormant integration | Legacy exploration/distance dependencies are injected through ports/adapters; absolute path mutation and unused route import removed. External helpers remain optional prerequisites for this unbuilt script. |
| [motion_control V-06](packages/motion_control_audit.md#v-06) | Implemented | ROS dependencies declared; unused JSON linkage removed from the active build. |
| [python_planner V-01](packages/python_planner_audit.md#v-01) | Implemented | `InspectionMission` receives config, map, telemetry, text, clock, diagnostics, output, NBV and optimization operations. |
| [python_planner V-02](packages/python_planner_audit.md#v-02) | Implemented | NBV policy imports no ROS/Open3D; preprocessing/clock/diagnostics are injected. Plain point/normal snapshots replace concrete cloud objects. |
| [python_planner V-03](packages/python_planner_audit.md#v-03) | Implemented | Planner depends on contracts and telemetry adapters, not the benchmark release. |
| [python_planner V-04](packages/python_planner_audit.md#v-04) | Removed separately in workspace | Legacy `Tools/get_point_cloud.py` is absent; its removal was preserved rather than attributed to this refactor. |
| [python_planner V-05](packages/python_planner_audit.md#v-05) | Removed separately in workspace | The old Tools mapping/import-routing files are absent. |
| [python_planner V-06](packages/python_planner_audit.md#v-06) | Removed separately in workspace | The parallel legacy Tools pipeline no longer occupies this package's source. |
| [python_planner V-07](packages/python_planner_audit.md#v-07) | Implemented | Optimization policy receives a solver; SciPy invocation is in an adapter. |
| [python_planner V-08](packages/python_planner_audit.md#v-08) | Boundary implemented; precision retained | ROS output adapter owns Float32 encoding/reconstruction. Collision validation remains policy; camera offset and four-value wire convention are unchanged. |
| [python_planner V-09](packages/python_planner_audit.md#v-09) | Metadata implemented | Runtime dependencies declared; pyoctomap is a Python-distribution prerequisite in setup metadata, not an invented ROS package name. |
| [perception V-01](packages/perception_audit.md#v-01) | Implemented | Model initialization/loading occurs in the concrete estimator constructor; pure depth policy imports without Torch/model loading. |
| [perception V-02](packages/perception_audit.md#v-02) | Implemented | Package-relative model imports and packaged checkpoint path replace workstation-specific import/model paths. |
| [perception V-03](packages/perception_audit.md#v-03) | Implemented | Metric-depth calculation is an injected use case; conversion/synchronization/publish/timing storage are outer concerns. |
| [perception V-04](packages/perception_audit.md#v-04) | Implemented | Perception uses a narrow timing sink outside benchmark_evaluator. |
| [perception V-05](packages/perception_audit.md#v-05) | Runtime packaging boundary implemented | Datasets/dataloader packages and research runner modules are excluded from the inference distribution. Vendored model internals retain their existing submodule ownership/provenance. |
| [perception V-06](packages/perception_audit.md#v-06) | Implemented | Research model/output selection uses registries; package/direct-script imports are supported. |
| [perception V-07](packages/perception_audit.md#v-07) | Metadata implemented | ROS/image/model dependencies declared in package/setup metadata. No model/framework dependency was installed by this task. |
| [sim_bridge V-01](packages/sim_bridge_audit.md#v-01) | Implemented | Scenario loading, sessions, stepping, dispatch and message encoding are separate roles. `main` explicitly starts the run after construction. |
| [sim_bridge V-02](packages/sim_bridge_audit.md#v-02) | Implemented | Ordered sensor handlers can be supplied without editing the stepping loop; per-sensor exception behavior retained. |
| [sim_bridge V-03](packages/sim_bridge_audit.md#v-03) | Boundary implemented; command layout retained | Immutable `Command` values enter the session port; the simulator adapter converts to its existing list API. Eight-channel ordering/defaults are unchanged. |
| [sim_bridge V-04](packages/sim_bridge_audit.md#v-04) | Metadata implemented | ROS/message/NumPy/HoloOcean prerequisites declared. |
| [sim_bridge V-05](packages/sim_bridge_audit.md#v-05) | Implemented | Current field and agent selection are injected into dispatch; vortex math and default `auv0` preserved. |
| [benchmark_evaluator V-01](packages/benchmark_evaluator_audit.md#v-01) | Implemented | Optimizer moved to `auv_experiments`; no evaluation-to-bringup launcher edge remains. |
| [benchmark_evaluator V-02](packages/benchmark_evaluator_audit.md#v-02) | Implemented | Shared ports and file telemetry have separate release packages; planner/perception no longer import evaluation. |
| [benchmark_evaluator V-03](packages/benchmark_evaluator_audit.md#v-03) | Implemented | Consumers depend on timing/filter/trajectory port roles and narrow file-sink facades. |
| [benchmark_evaluator V-04](packages/benchmark_evaluator_audit.md#v-04) | Implemented | ROS observer delegates to metric policy and immutable report snapshots; file/plot implementation is separate. |
| [benchmark_evaluator V-05](packages/benchmark_evaluator_audit.md#v-05) | Implemented | Map metrics consume a geometry port; storage/ICP/display use a configured adapter. Existing scenario defaults remain. |
| [benchmark_evaluator V-06](packages/benchmark_evaluator_audit.md#v-06) | Implemented | Trial objective receives runner/evaluator/reporter; process, YAML and result IO stay in adapters. |
| [benchmark_evaluator V-07](packages/benchmark_evaluator_audit.md#v-07) | Ownership implemented; format retained | Shared `mission_results.py` names schema version 1, filename, distance formatter/parser; existing human-readable file format preserved. |
| [benchmark_evaluator V-08](packages/benchmark_evaluator_audit.md#v-08) | Metadata implemented | Evaluation prerequisites declared; experiment-only dependencies moved to their package. |
| [bringup V-01](packages/bringup_audit.md#v-01) | Implemented | Runtime launch closure no longer loops back through an optimizer in the observer. |
| [bringup V-02](packages/bringup_audit.md#v-02) | Implemented | Launch libraries and all launched components declared. |
| [bringup V-03](packages/bringup_audit.md#v-03) | Implemented | Bringup owns config-schema translation; experiments call it rather than duplicating node-key knowledge. |

## Behavior preservation and public entry points

Algorithms, thresholds, gains, camera offsets, topic names, QoS choices, stereo-call order, timestamps and message layouts were preserved. Policy boundary extraction necessarily changes internal constructor/method APIs. The optimizer's new owner is `auv_experiments`; its ROS launch command and six sampling ranges remain unchanged. The old mapper implementation is replaced by policy/adapters; `src/search_algo.hpp` preserves the original standalone search API and stderr diagnostic.

An existing tuning test expected `nbv_complexity_weight=15` although the shipped YAML already contained `30`. The test now verifies both the unchanged mathematical default `15` and the unchanged configured override `30`; this resolves a stale expectation without tuning the system.

The planner publishes an empty trajectory for cancellation, while the current queue ignores empty arrays. This existing mismatch was documented and tested rather than changed. Likewise, `visit_radius` is stored but unused, and the controller's YAML `initial_waypoint_yaw` is not a declared controller parameter. See the parameter reference for these distinctions.

## Validation evidence and limits

| Check | Result / scope |
|---|---|
| ROS 2 Jazzy / colcon builds | All ten packages build, with separate policy/adapters and BUILD_TESTING enabled. |
| Python regression/policy suite | 76 tests pass: the 65 existing planner checks plus 11 new policy/boundary cases. |
| C++ policy regressions | Queue offsets/completion/empty-input retention, mixer allocation, projection/depth bounds, search and failure-port diagnostics pass. |
| Baseline C++ map comparison | Original and extracted mapper receive three deterministic depth/RGB/pose frames: leaf counts, candidate result counts and saved `.bt` contents match bytewise. The fixture deliberately selects no candidate frontiers; it does not prove all candidate routes equivalent. |
| Framework/IO import boundary checks | Python domain/use-case files are checked against forbidden framework/IO imports; both C++ policy libraries compile independently of ROS/mapping libraries. |
| C++ lint checks | cppcheck, CMake/style checks pass for both C++ packages; the legacy Python integration also passes flake8/pep257. Existing lint registrations remain enabled. |
| Package schemas | All ten package manifests pass the official ROS XML schema check with network access. |
| Clean perception packaging | Fresh build includes runtime model modules/checkpoint and excludes dataset/dataloader packages and research runner modules. |

Torch inference/checkpoint loading, live HoloOcean stepping, Open3D Python geometry/ICP and a full ROS mission replay were not executed in this verification environment. Component builds do not establish availability of every Python runtime backend. No actual Optuna trial or process-group termination was invoked. Existing result artifacts and the user's legacy-file removals were preserved.

## Reproducing the checks

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths NBV_paper_ros2/src --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=ON
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest NBV_paper_ros2/src/docs/architecture/verification/test_policy_boundaries.py NBV_paper_ros2/src/python_planner/test/test_motion_continuity.py NBV_paper_ros2/src/python_planner/test/test_route_search_limits.py NBV_paper_ros2/src/python_planner/test/test_nbv_math.py NBV_paper_ros2/src/python_planner/test/test_nbv_filter_logging.py NBV_paper_ros2/src/python_planner/test/test_nbv_config.py -q
ctest --test-dir build/active_inspection_planner --output-on-failure
ctest --test-dir build/motion_control --output-on-failure
```

The policy verification test adds the source-package roots for direct execution. Build directories used during this task were isolated under `/tmp/nbv_refactor_build`, with installs/logs also under `/tmp`. XML schema lint requires access to ROS's official schema URL. The baseline comparison harness is [mapping_equivalence.cpp](verification/mapping_equivalence.cpp); compile it against the pre-refactor mapper with `ORIGINAL_MAPPER`, then against the new policy/adapters without that define, and compare its text/map output.

Regenerate the topology and complete source bundle with `python3 NBV_paper_ros2/src/docs/architecture/verification/generate_refactor_artifacts.py`. [refactoring_code_manifest.json](refactoring_code_manifest.json) records file hashes for reviewing exact workspace contents. The generated bundle includes vendored BGNet import/registry edits; the submodule changes must be retained with any eventual commit. No commit or deployment was performed.
