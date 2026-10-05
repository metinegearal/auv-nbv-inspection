# Package Architecture Audit: benchmark_evaluator

## Current Metrics

- Fan-in: 3 | Fan-out: 1
- Instability (I): 0.250
- Abstractness (A): 0.000 (Na=0; Nc=2)
- Distance (D): 0.750
- Status: Zone of Pain

Scope: A runtime logging library and ROS observer share one release unit with plotting, map scoring and a process-level Optuna experiment driver. Its three incoming component clients make this the primary coupling hotspot.

Counts use the observed source/deployment graph, not ROS topic flow. See [Phase 1 definitions and denominator conventions](../phase_1_topology.md#metric-definitions-and-counting-rules). An asterisk denotes a mathematically undefined denominator replaced by zero only for display; the resulting D/status is conditional. Classification alone is not a violation. All file paths below are relative to `NBV_paper_ros2/src`.

## Violations Found
| ID | Rule Broken | File & Line / Symbol | Concrete Issue | Actor / Architectural Impact |
|---|---|---|---|---|
| <a id="v-01"></a>V-01 | ADP / SDP | `benchmark_evaluator/benchmark_evaluator/optuna_optimizer.py:63` / `objective launch command` | **Critical.** The experiment driver names bringup, which launches benchmark_evaluator, perception and python_planner; the latter two import this package. This closes three deployment cycles. I rises from 0.250 to 0.833 on benchmark_evaluator → bringup. | Experiment launcher and production runtime cannot be independently organized/released without carrying the cyclic deployment contract. |
| <a id="v-02"></a>V-02 | CCP / CRP / SAP | `benchmark_evaluator/benchmark_evaluator/midrun_logger.py:10` / `MidrunLogger release placement` | **Critical.** Planner and perception import one concrete logger from a component also containing ROS benchmarking, visualization, OctoMap scoring and Optuna. No abstract telemetry contract protects these stable incoming dependencies (A=0, D=0.750). | Production telemetry consumers depend on an experiment/reporting release they do not otherwise reuse. |
| <a id="v-03"></a>V-03 | ISP | `benchmark_evaluator/benchmark_evaluator/midrun_logger.py:13` / `MidrunLogger mode flags` | **Moderate.** pipeline_only and trajectory_only select subsets of a single logger API and file initialization, but all clients depend on the full filter/depth/iteration/trajectory implementation. | Perception needs stage timing while planning and evaluation need different event sets; unrelated event-schema changes enlarge each consumer dependency. |
| <a id="v-04"></a>V-04 | SRP / DIP | `benchmark_evaluator/benchmark_evaluator/run_baselines.py:12` / `BenchmarkLogger` | **Moderate.** The ROS node computes distance, timing, heading and trajectory error, persists CSV/text, generates figures and performs saving during destroy_node (lines 178, 283, 299, 352). | Evaluation researchers, report designers and ROS lifecycle maintainers change one class; IO is embedded in metric use cases. |
| <a id="v-05"></a>V-05 | SRP / DIP / OCP | `benchmark_evaluator/benchmark_evaluator/benchmark_evaluator.py:7` / `evaluate_map_accuracy` | **Moderate.** Scoring loads fixed map/GT paths, applies scene-specific transforms, performs alignment and presents output. single_station at line 9 and branches at lines 13 and 20 hardcode scenario variants. | Adding an experiment scene or changing storage/viewing requires editing the scoring routine; metrics cannot be invoked on supplied point sets alone. |
| <a id="v-06"></a>V-06 | SRP / DIP | `benchmark_evaluator/benchmark_evaluator/optuna_optimizer.py:38` / `objective` | **Moderate.** Trial policy samples parameters, rewrites ROS config, deletes prior result artifacts, constructs a shell command, manages a process group and parses mission results. | Research objective changes and deployment/process changes share one function; TrialRunner and structured TrialResult boundaries are absent. |
| <a id="v-07"></a>V-07 | CCP / REP (contract ownership) | `benchmark_evaluator/benchmark_evaluator/run_baselines.py:307` / `optuna_mission_stats.txt` | **Moderate.** Run completion and optimizer results are coupled through a concrete filename and presentation text (optimizer lines 15, 80, 105). The producer also emits human-oriented report wording. | Changing report wording or file protocol requires coordinated changes in trial execution; an explicit versioned result schema is missing. |
| <a id="v-08"></a>V-08 | REP (dependency metadata) | `benchmark_evaluator/package.xml:10` / `dependency declarations` | **Moderate.** Manifest declares matplotlib but omits ROS, NumPy, Open3D, pyoctomap, Optuna, YAML and pandas dependencies used by shipped modules; setup.py only declares setuptools and matplotlib. | Consumers of this reuse unit receive incomplete installation requirements; infer neither a release history nor a licensing conclusion from placeholders. |

## Proposed Structural Boundary
- Interfaces to extract: `PipelineTimingSink`, `FilterEventSink`, `TrajectorySink`, `IterationSink`, `TrialRunner`, `EvaluationMapSource`, `GroundTruthSource`, `MetricsSink`, `Clock`.
- Dependencies to invert or decouple: Let planner/perception own telemetry ports; move concrete file writers into an adapter component. Extract evaluation functions over plain observations. Move Optuna plus launch/config knowledge to an outer experiment composition root.

## Current Internal Package Diagram

Solid arrows are calls, construction, field ownership or source dependencies as labeled. These diagrams summarize source relationships, not measured runtime execution; dormant paths are labeled. Red nodes mix policy with details.

```mermaid
graph LR
 OPT[optuna_optimizer.objective] --> YAML[write_params_yaml]
 OPT --> PROC[subprocess / signals / file polling]
 YAML --> BR[bringup config schema]
 PROC --> LAUNCH[bringup mapper.launch.py]
 OPT --> EV[evaluate_map_accuracy]
 EV --> MAP[OctoMap / Open3D / GT files / viewer]
 RUN[BenchmarkLogger: callbacks / metrics / shutdown] --> LOG[MidrunLogger]
 RUN --> PLOT[generate_results_plot]
 RUN --> CSV[CSV / text / diagnostic figures]
 LOG --> FILES[CSV and JSONL files]
 CLI[plot_results.main] --> PLOT
 PLOT --> MPL[matplotlib and filesystem]
 classDef leak fill:#ffeded,stroke:#bf3030
 class OPT,EV,RUN leak
```

## Ideal Clean Internal Package Diagram

This is a proposed partition, not an implemented structure. Ports are policy-owned abstract interfaces (Python Protocol/ABC, C++ pure virtual classes); implementations depend on them. Entities cross ports as plain values. A grouped port box means separate role-specific interfaces, never one mandatory aggregate interface.

```mermaid
graph LR
 subgraph Adapters[Interface Adapters]
 ROS[ROS observation adapter]
 IO[CSV / JSONL / plots / map readers]
 RUNNER[Process runner and Optuna composition]
 end
 subgraph UseCases[Use Cases]
 E[EvaluateMap / EvaluateTrajectory / RunTrial]
 PORT[TrialRunner / MapSource / GroundTruthSource / MetricsSink interfaces]
 end
 subgraph Entities
 M[TrajectorySamples / EvaluationMetrics / TrialResult]
 end
 ROS --> E
 RUNNER --> E
 RUNNER -. implements .-> PORT
 IO -. implements .-> PORT
 E --> PORT
 PORT --> M
 E --> M
```

## Principle Assessment and Non-Findings

ADP/SDP refer to the observed deployment graph, not cyclic imports. SAP is supported by concrete MidrunLogger being depended upon by unrelated production clients. ISP applies to those clients depending on its aggregate API, not a claim that subclasses are forced to override methods. REP/CCP/CRP evidence is current dependency/reuse structure; team change history was not available. BenchmarkLogger.destroy_node calls its superclass; IO errors could interrupt cleanup, but a proven base-contract violation has not been established, so no LSP finding is asserted.

Actor labels are inferred responsibilities, not verified organizational roles. SOLID findings require responsibility, client or contract evidence; line count, inheritance, ordinary guards, framework adapters and missing abstract classes alone do not prove a violation. REP findings establish incomplete reuse/deployment contracts; no release-history audit was performed.

## Module-by-Module Target Placement

| File | Current defined classes / top-level functions | Ideal placement / diagnostic disposition |
|---|---|---|
| `benchmark_evaluator/benchmark_evaluator/__init__.py` | No classes/top-level Python functions | Empty package marker; no executable policy or class interaction to partition. |
| `benchmark_evaluator/benchmark_evaluator/benchmark_evaluator.py` | `evaluate_map_accuracy:7` | Evaluation use case and pure metrics; map/GT loaders and viewer become adapters. |
| `benchmark_evaluator/benchmark_evaluator/midrun_logger.py` | `MidrunLogger:10`; `generate_results_plot:150`; `default_run_id:269` | Concrete role-specific file sinks and a separate plot adapter; client-owned abstract event contracts. |
| `benchmark_evaluator/benchmark_evaluator/optuna_optimizer.py` | `write_params_yaml:18`; `objective:38` | Outer experiment composition plus RunTrial use case using TrialRunner/ResultStore ports. |
| `benchmark_evaluator/benchmark_evaluator/plot_results.py` | `main:6` | CLI composition adapter; delegating to plot output is appropriate. |
| `benchmark_evaluator/benchmark_evaluator/run_baselines.py` | `BenchmarkLogger:12`; `main:363` | ROS observation adapter, trajectory metric use case and report adapters. |
| `benchmark_evaluator/package.xml` | No classes/top-level Python functions | Release/build metadata; reconcile dependencies with source. No domain policy extraction. |
| `benchmark_evaluator/setup.cfg` | No classes/top-level Python functions | Release/build metadata; reconcile dependencies with source. No domain policy extraction. |
| `benchmark_evaluator/setup.py` | No classes/top-level Python functions | Release/build metadata; reconcile dependencies with source. No domain policy extraction. |
| `benchmark_evaluator/test/test_copyright.py` | `test_copyright:23` | Test-only consumer; excluded from production metrics. Static review only; no test execution claimed. |
| `benchmark_evaluator/test/test_flake8.py` | `test_flake8:21` | Test-only consumer; excluded from production metrics. Static review only; no test execution claimed. |
| `benchmark_evaluator/test/test_pep257.py` | `test_pep257:21` | Test-only consumer; excluded from production metrics. Static review only; no test execution claimed. |

## Class Counting Evidence

- `benchmark_evaluator/benchmark_evaluator/midrun_logger.py`: `MidrunLogger` (line 10).
- `benchmark_evaluator/benchmark_evaluator/run_baselines.py`: `BenchmarkLogger` (line 12).

## Function and Class Interaction Detail

The following module diagrams are static Python AST call/name and class-containment maps. Dotted edges indicate containment, solid edges an explicit local call. Repeated method names are resolved within the containing class; nested functions belong to their lexical owner. Dynamic dispatch, callbacks registered for later execution and arbitrary imported calls are not inferred as exact call edges. Package diagrams above supply the cross-module and C++ relationships. See [source coverage](../source_coverage.md) for all imports, C++ includes and function symbols.

### `benchmark_evaluator/__init__.py`

Ideal placement: Empty package marker; no executable policy or class interaction to partition.

No executable local symbol interaction (empty marker or import/registry facade).

### `benchmark_evaluator/benchmark_evaluator.py`

Ideal placement: Evaluation use case and pure metrics; map/GT loaders and viewer become adapters.

```mermaid
graph LR
 s0["evaluate_map_accuracy : 7"]
```

### `benchmark_evaluator/midrun_logger.py`

Ideal placement: Concrete role-specific file sinks and a separate plot adapter; client-owned abstract event contracts.

```mermaid
graph LR
 s0["MidrunLogger : 10"]
 s1["MidrunLogger.__init__ : 13"]
 s2["MidrunLogger._reset_files : 28"]
 s3["MidrunLogger.record_filter : 54"]
 s4["MidrunLogger._json_safe : 64"]
 s5["MidrunLogger._vector : 77"]
 s6["MidrunLogger.record_trajectory : 80"]
 s7["MidrunLogger.record_stage : 86"]
 s8["MidrunLogger.record_depth_pose : 98"]
 s9["MidrunLogger.record_iteration : 105"]
 s10["generate_results_plot : 150"]
 s11["default_run_id : 269"]
 s0 -.-> s1
 s0 -.-> s2
 s0 -.-> s3
 s0 -.-> s4
 s0 -.-> s5
 s0 -.-> s6
 s0 -.-> s7
 s0 -.-> s8
 s0 -.-> s9
 s1 --> s2
 s3 --> s4
 s6 --> s5
 s8 --> s5
 s9 --> s5
```

### `benchmark_evaluator/optuna_optimizer.py`

Ideal placement: Outer experiment composition plus RunTrial use case using TrialRunner/ResultStore ports.

```mermaid
graph LR
 s0["write_params_yaml : 18"]
 s1["objective : 38"]
 s1 --> s0
```

### `benchmark_evaluator/plot_results.py`

Ideal placement: CLI composition adapter; delegating to plot output is appropriate.

```mermaid
graph LR
 s0["main : 6"]
```

### `benchmark_evaluator/run_baselines.py`

Ideal placement: ROS observation adapter, trajectory metric use case and report adapters.

```mermaid
graph LR
 s0["BenchmarkLogger : 12"]
 s1["BenchmarkLogger.__init__ : 13"]
 s2["BenchmarkLogger._wrap_angle : 57"]
 s3["BenchmarkLogger.planned_trajectory_callback : 60"]
 s4["BenchmarkLogger._nearest_planned_point : 69"]
 s5["BenchmarkLogger.odom_callback : 79"]
 s6["BenchmarkLogger.log_trajectory_callback : 99"]
 s7["BenchmarkLogger.finish_callback : 153"]
 s8["BenchmarkLogger._trajectory_rmse : 165"]
 s9["BenchmarkLogger._save_diagnostics : 178"]
 s10["BenchmarkLogger._diagnostic_metrics : 283"]
 s11["BenchmarkLogger.save_benchmark_data : 299"]
 s12["BenchmarkLogger.destroy_node : 352"]
 s13["main : 363"]
 s0 -.-> s1
 s0 -.-> s10
 s0 -.-> s11
 s0 -.-> s12
 s0 -.-> s2
 s0 -.-> s3
 s0 -.-> s4
 s0 -.-> s5
 s0 -.-> s6
 s0 -.-> s7
 s0 -.-> s8
 s0 -.-> s9
 s10 --> s8
 s11 --> s9
 s12 --> s11
 s13 --> s0
 s6 --> s2
 s6 --> s4
 s7 --> s11
 s9 --> s10
```
