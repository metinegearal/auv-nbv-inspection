# Python inspection planner migration

All added runtime code lives in `active_inspection_planner`. The original Python
package is retained byte-for-byte at the source level; its SHA-256 inventory is
in `migration_evidence/reference_sha256.json`. No controller, perception,
simulator, shared-contract, or bringup sources were changed. The earlier C++
`nbv_utility_node` executable remains available; the migrated implementation is
`inspection_planner_node`.

## Component inventory

| Python reference | C++ replacement |
| --- | --- |
| `planner_node.py` | `src/migration/node.cpp`: ROS subscriptions, callback groups, two-thread executor, ROS-clock timer, publishers, shutdown |
| `domain/configuration.py`, `domain/nbv_parameters.py` | `configuration.cpp`, `config/reference_defaults.json`, parameter declarations in `node.cpp`, validation in `mission.cpp` |
| `adapters/point_cloud.py` | `nbv.cpp::prepare_cloud`, Open3D 0.19.0 downsampling, noise filtering, normals and orientation |
| `adapters/octomap.py` | `adapters.cpp::Octomap`: insertion, expanded occupied leaves, serialization |
| `domain/seabed.py` | `nbv.cpp::Seabed`: changed/removed voxel statistics, floor support and coverage |
| `get_NBV.py`, `use_cases/nbv.py` | `nbv.cpp`: PCA, tangent gaps, clustering/tails, upward axis, cones, observation occlusion, gain/cost normalization, stable top-three selection |
| `route_planning.py::astar_3d` | `route.cpp::astar`: six neighbors, Manhattan heuristic, tuple heap tie order, expansion accounting and failure reasons |
| Inflation, collision checking, multi-target routes | `route.cpp::inflate`, `collision_free`, `plan_route` |
| Smoothing, distance sampling, reconnection | `route.cpp::smooth`, `resample`, `linear_fallback`, `reconnect` |
| `trajectory_optimization.py`, `use_cases/trajectory_optimization.py`, `adapters/optimization.py` | `optimization.cpp`: velocity regression, continuity objective, bounded L-BFGS-B, finite differences and safe backtracking |
| `use_cases/mission.py`: depth projection/history | `mission.cpp::depth`, `depth_points`, `accumulate`, `odometry`, `magnetometer` |
| Mission planning, recovery, cancellation, completion | `mission.cpp::plan_and_publish`, `active_safe`, `stalled`, `cancel`, blacklist methods, `tick` |
| `adapters/ros_output.py` | `mission.cpp::publish_trajectory` and ROS publishers: float32 offset encoding and reconstructed vehicle-path validation |
| `adapters/files.py`, planner use of shared telemetry sinks | `adapters.cpp::FileSink`: CSV, JSONL, append-only evaluation log, reset-on-start planner files |
| Legacy bringup launch | Package-local `launch/mapper.launch.py` reuses the existing description and replaces its single planner action |

The unused reference cache/visited-target fields have no additional behavior
introduced in C++. Parameter defaults include those switches for compatibility.

## Numerical implementation

* Open3D **0.19.0**, matching the installed reference. No replacement normal
  orientation or clustering algorithm is used.
* SciPy **1.16.3** cKDTree C++ sources retain its balanced/compact construction,
  `KDTree` leaf size 10, nearest-neighbor ties and sorted radius results.
* SciPy **1.16.3** FITPACK `parcur`/`splev` Fortran routines are linked directly.
  Smoothing remains the FITPACK residual-bound parameter `s`; it is not a cubic
  interpolator or a different regularization objective. The Python degree,
  parameterization, workspace, sampling and endpoint rules are retained.
* PCA uses the same symmetric LAPACK eigenvalue algorithm. NumPy-style contiguous
  pairwise reductions are used in the optimizer objective to avoid amplification
  by its `1e-8` finite-difference step.
* SciPy's L-BFGS-B C implementation is vendored with the CPython wrapper removed.
  Fixed variables are eliminated as in `scipy.optimize.minimize`, with the same
  bounds, 32-control limit, stopping settings, `maxfun=12000`, and collision-checked
  backtracking scales. This is not an alternative optimizer.
* Depth sampling above 4,000 points uses MT19937's masked shuffle, verified
  against seeded NumPy `RandomState.choice`. Production runs are independently
  randomly seeded, as the reference is; identical unseeded subsamples across
  two separate processes are not promised.

`vendor/scipy/README.md` documents the limited upstream-source adaptations and
licenses. `tools/import_numerics.py` reproduces them from the SciPy 1.16.3 source
archive. No Python interpreter is embedded or invoked by the C++ node.

Two reference binding details are intentionally preserved: float32 point/origin
conversion at OctoMap insertion, and `pyoctomap`'s integer `dimension / 2` in
leaf expansion. The latter offsets one-voxel extracted samples by half a voxel.
Correcting it would change the current behavioral reference and is a separate
task. The map is serialized with `ColorOcTree::write`, despite the `.bt` suffix.

## Build

Use the existing ROS Jazzy environment and Python reference environment. Required
build dependencies include a C++17 compiler, gfortran, Eigen, BLAS/LAPACK,
OctoMap, OpenCV, and the existing ROS dependencies. The exact Open3D SDK is not
provided by this machine's apt package lists; the helper downloads a pinned SDK.

```bash
cd /home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/NBV_paper_ros2
source /opt/ros/jazzy/setup.bash
python3 src/active_inspection_planner/tools/bootstrap_open3d.py
colcon build --packages-select active_inspection_planner --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
```

The helper can copy the Open3D wheel's `libc++`/`libc++abi` runtime libraries into
the SDK. Alternatively supply `--runtime-directory` or install those system
runtime libraries. Keep the SDK directory available at runtime; the installed
executables retain its library search path. The helper defaults to the ignored,
persistent `src/active_inspection_planner/.dependencies` directory, which CMake
discovers automatically. For a different location, use `--directory` and pass
`-DOpen3D_ROOT=<that-directory>/open3d-devel-linux-x86_64-cxx11-abi-0.19.0` when building.

Validation in this task used `/tmp/planner-colcon-build`,
`/tmp/planner-colcon-install`, and `/tmp/planner-colcon-log` so the existing
workspace build/install products were not overwritten.

## Launch

```bash
# Full existing bringup, with the migrated planner selected by default:
ros2 launch active_inspection_planner mapper.launch.py

# Same bringup, Python behavioral reference:
ros2 launch active_inspection_planner mapper.launch.py planner_implementation:=python

# Planner alone, same YAML precedence and planner_node name:
ros2 launch active_inspection_planner planner.launch.py planner_implementation:=cpp
```

Both launches retain `params_file` and `nbv_params_file`; the main parameters file
wins. The full mapper also forwards the existing `holo_config_file` argument.
The original `ros2 launch bringup mapper.launch.py` still selects Python because
the final scope instruction prohibited changes outside this package. Use the
new package-local mapper entry point for the C++ default.

Both nodes retain `nbv_utility_node` as their standalone node name. Launch gives
them `planner_node`, matching the shipped YAML. Topics/types, reliable output
QoS, best-effort camera/depth QoS (depth 10), best-effort odometry/magnetometer
QoS (depth 1), and request QoS (reliable, depth 1) are retained. Camera offsets
remain `[0.35, 0.3875, -0.5]`, with original body/camera/vehicle transformations.

## Validation and reproducibility

```bash
ctest --test-dir build/active_inspection_planner \
  -R 'planner_differential|planner_telemetry|inspection_policy_regression' --output-on-failure

PLANNER_REPLAY="$PWD/install/active_inspection_planner/lib/active_inspection_planner/planner_replay" \
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 -m pytest -q src/active_inspection_planner/test/migration/test_differential.py \
  src/active_inspection_planner/test/migration/test_telemetry.py

# Uses localhost DDS on an isolated domain and temporary output directories:
PLANNER_NODE="$PWD/install/active_inspection_planner/lib/active_inspection_planner/inspection_planner_node" \
ROS_DOMAIN_ID=178 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 -m pytest -q src/active_inspection_planner/test/migration/test_ros_contract.py
```

Tests compare the current Python source directly, with source-hash guards. They
cover empty inputs, occupied starts/goals, exhausted/disconnected A*, route
priorities and ties, short/duplicate routes, unsafe smoothing, mandatory corners
beyond point caps, yaw wrapping, depth units/timestamps, map changes/freeing,
missing seabed support, movement during planning, rejected reconnection, stalls,
blacklist expiry, explicit empty cancellation, wire-rounding collisions, and
three-strike completion. Real ROS tests check default/shipped parameters, topic
types, reliability, and completion. Fast DDS discovery reports history depth as
unknown (0), so queue-depth values are specified in the adapters but cannot be
verified from this discovery API alone.

Discrete outputs are compared exactly. Ordinary geometry/scores use `atol=rtol=1e-9`;
optimization uses `atol=rtol=2e-6`. Observed maximum optimizer coordinate/yaw
difference is approximately `5.02e-7`; other tested numerical paths are much
closer. `migration_evidence/numerical_errors.json` records each measured case.
Safety checks and optimizer acceptance decisions are compared as well as output
coordinates. Timing fields are excluded from equality comparisons.

See [VALIDATION.md](VALIDATION.md) for measured latency, variability, throughput,
memory, test results, and the commands to reproduce performance measurements.

## Baseline defects and limits

The unmodified Python behavioral suite has **157 passed / 36 failed**. Failures
include obsolete two-value route/A* expectations, removed utility metadata
validation, obsolete inflation cache/validation expectations, and short-route
tests requiring the old linear fallback. Three lint-test imports are unavailable
in the reference environment. The unrelated cruise benchmark has **8 passed /
1 failed**: 6.30 seconds does not meet `< 0.65 * 9.50` seconds. Baseline reports
are retained under `migration_evidence`; none of those systems were fixed here.

Available recordings contain a map, candidate sets, poses and planned routes.
They do **not** contain original depth frames or a complete ROS mission bag.
Frozen output-log fixtures and synthetic sensor sequences are tested, but a full
recorded-sensor mission or fresh simulator mission equivalence claim is not
supported. Recorded pose replay uses identity quaternions; it tests positional
history and does not reconstruct missing magnetometer history. Provenance and
the fixture limitations are saved alongside `test/migration/recorded`.

Floating-point arithmetic and optimizer trajectories are not bit-identical.
CSV/JSONL fields and semantics are preserved; JSON whitespace/key ordering and
console/evaluation numeric rendering can differ. Full live-mission timing,
unseeded random subsampling identity, and all possible near-threshold numerical
ties remain outside the demonstrated evidence. Performance numbers are measured
on this host, which also had a running mission; inspect the raw variability.

## Changelog

* Added a standalone C++ planner and preserved the Python reference and legacy
  C++ executable.
* Ported all runtime policies and adapters, including optional optimization,
  recovery, diagnostics, telemetry, and completion.
* Added exact-version numerical dependencies, build integration, a dependency
  bootstrap helper, and selectable launches defaulting to C++ after validation.
* Added deterministic differential replay, frozen recorded-input fixtures, ROS
  integration checks, file-format tests, and reproducible benchmarks.
* No commits were created.
