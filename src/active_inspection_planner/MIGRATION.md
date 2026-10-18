# Python inspection planner migration

The planner runtime lives in `active_inspection_planner`; launch composition and
operational configuration belong to `bringup`. The original Python
package is retained byte-for-byte at the source level; its SHA-256 inventory is
in `migration_evidence/reference_sha256.json`. No controller, perception,
simulator, or shared-contract sources were changed. The earlier C++
`nbv_utility_node` executable remains available; the migrated implementation is
`inspection_planner_node`.

## Component inventory

| Python reference | C++ replacement |
| --- | --- |
| `planner_node.py` | `src/migration/node.cpp`: ROS subscriptions, callback groups, two-thread executor, ROS-clock timer, publishers, shutdown |
| `domain/configuration.py`, `domain/nbv_parameters.py` | `configuration.cpp`, `migration_evidence/reference_defaults.json`, parameter declarations in `node.cpp`, validation in `mission.cpp` |
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
| Bringup integration | `bringup/launch/mapper.launch.py` and `bringup/launch/planner.launch.py`, using `bringup.planner_launch` for planner selection |

The original migration kept the reference cache/visited-target fields behaviorally
unchanged. Parameter defaults include those switches for compatibility.

On 2026-10-16 the C++ mission gained a separate frontier cluster cache: local
geometry is refreshed within `nbv_exploration_radius`, while previously discovered
distant clusters remain in global candidate scoring. Geometry and complexity are
retained until locally refreshed or no longer supported by the map; viewpoint
gain and travel costs are recomputed each planning pass. This is an intentional
extension beyond the preserved Python reference. Standalone stateless `nbv`
replays retain the original behavior for differential validation. The sequence
tests in `test/migration/test_nbv_cache.py` cover the stateful extension.

The October 16 wall-stall fix also intentionally changes mission collision
geometry. Safety uses actual OctoMap leaf bounds, expanded into planner cells,
and `collision.cpp::inflate_clearance` reserves the configured distance between
whole cells. The legacy sample-center extraction and inflation helpers remain
for NBV/reference compatibility; they no longer build the live collision map.
The real-map mission fixture expects its corrected collision-cell count (2,483
instead of 978). Its mapping, generated NBVs, input states, and other unchanged
state remain compared to the Python reference; runtime execution has explicit
single-goal, actionable-wire, and collision-validation assertions.

Before NBV selection, a start inside the tracking buffer triggers a bounded
straight retreat search in 26 directions. It preserves yaw, checks continuous
segments against occupied boxes expanded by `collision_radius`, requires
nondecreasing sampled obstacle clearance, and ends beyond the full buffer with
room for voxel discretization and stopping short. Float32 wire coordinates are
checked too. Each depth update revalidates the remaining retreat against the
current map. Unsafe/stalled retreats clear the downstream queue; no safe retreat
means hold and retry after `route_stall_timeout_s`, without blacklisting NBVs or
declaring exploration complete. Normal planning resumes after clearance is
restored. This is a conservative local recovery, not a general escape planner;
it can hold when escaping would require a curved route or the body radius is
already obstructed. Controller and normal spline/trajectory generation are unchanged.
`test_collision_recovery.py` covers this behavior; set `PLANNER_WALL_MAP` to the
October 16 saved map to also reproduce the recorded stuck position.

Blocked NBV destinations now have a bounded standoff fallback before the target
is blacklisted. `route_goal_max_extension_m` defaults to 1.5 m (zero disables it).
The planner extends the original camera viewing ray in voxel-sized steps,
recomputes the vehicle pose with the camera offset, and requires a free cell in
the full collision map. The camera must stay within `max_depth`, with no mapped
occluder along its sightline (excluding the final two voxels at the surface
target). A* tries the admissible poses with one shared `astar_max_steps` budget
per blocked candidate. The resulting endpoint is retained through trajectory
generation, reconnection, and Float32 wire validation; the observed surface
target stays the same. `nbv_goal_standoff` telemetry records both vehicle goals
and the outcome. Clear goals retain the existing route behavior. This bounded
search can still fail if every extended viewpoint is obstructed or unreachable.

`test_goal_standoff.py` covers obstacle detours, visibility, range/extension/search
limits, and mission publication. Set `PLANNER_GOAL_MAP` and
`PLANNER_GOAL_ITERATIONS` to the October 16 13:00 run files for the distant-NBV
reproduction. That case now publishes a 16.9 m collision-checked route to a
viewpoint roughly 14.1 m from the vehicle, using the third candidate after the
first two sightlines are rejected. The differential defaults fixture explicitly
adds the new C++ parameter while continuing to compare all historical settings.

The subsequent repeated-target regression adds mission lifecycle tracking.
Runtime planning executes one reachable viewpoint, validates visibility even for
an initially free goal, and sends actionable waypoints without the current-pose
anchor. A fresh depth pose at the resolved goal, with the required heading and
clear sightline, records the surface target as observed even beyond the legacy
history radius. Cancelled execution retains a spatial attempt record and excludes
the target temporarily. `route_target_max_attempts=3` bounds retries across
cooldown expiry and cluster ID changes; `route_target_timeout_s=120` bounds moving
loops without restarting on same-target replans. Once the final allowed attempt
fails, the target enters the candidate blacklist without an expiry for the rest
of the mission, even when temporary cooldowns are disabled. Recovery now also
waits for its actual endpoint.
See `REGRESSION_CHECKS.md` for the incident, regression gate, and live checks.
Failed goal searches count toward the spatial attempt budget even if another
target is successfully published in the same planning pass. Multiple failed
viewpoints of one target count once per pass.

Collision handling additionally probes surrounding viewpoints when the original
view and its outward adjustment fail. A 15-degree spherical sweep includes the
vertical poles; each direction checks sensor range, raw-map sightline and inflated
vehicle clearance before A*. A fully blocked sweep immediately blacklists the
surface target and its 1 m neighborhood for this mission. Clear rays are tried
with a shared A* budget; search exhaustion is distinct from geometric blockage.
An active-route collision clears the downstream queue before checking its target,
and clear directions remain eligible for an immediate replacement route.

Runtime utility normalization now follows mission eligibility. NBV geometry keeps
the raw gain/cost for all views until observed, retry-exhausted and blacklisted
targets have been removed. The mission then normalizes the eligible pool, chooses
the three views per cluster and applies the existing utility threshold. A new
collision blacklist restarts scoring from that raw pool within the same planning
call, while preserving the route candidate budget and avoiding repeated view
searches. Positive equal-gain pools retain a gain of one so a singleton is not
silently discarded. `utility_normalization` telemetry records each pass and its
actual eligible gain/cost bounds; iteration candidates carry the resulting scores.
Stateless reference scoring retains its historical behavior for differential checks.

Bringup exposes `route_blacklist_radius_m` (default 0.5 m) for retry-region
matching, temporary bans and exhausted regions, and
`route_collision_blacklist_radius_m` (default 1 m) for all-rays-blocked bans.
Both are finite positive startup parameters in `bringup/config/params.yaml`.
Collision bans have an explicit type independent of their radius. Retry events
include their canonical region centre and radius so benchmark snapshots and
`test_env_2.py` display the actual configured volume. These settings do not alter
the vehicle's collision clearance or the observed-target history radius.

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
NBV extraction retains that behavior for reference compatibility; collision
geometry bypasses it using the true leaf bounds as described above. The map is
serialized with `ColorOcTree::write`, despite the `.bt` suffix.

## Build

Use the existing ROS Jazzy environment and Python reference environment. Required
build dependencies include a C++17 compiler, gfortran, Eigen, BLAS/LAPACK,
OctoMap, OpenCV, and the existing ROS dependencies. The exact Open3D SDK is not
provided by this machine's apt package lists; the helper downloads a pinned SDK.

```bash
cd /home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/NBV_paper_ros2
source /opt/ros/jazzy/setup.bash
python3 src/active_inspection_planner/tools/bootstrap_open3d.py
colcon build --packages-select active_inspection_planner bringup --cmake-args \
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
ros2 launch bringup mapper.launch.py

# Same bringup, Python behavioral reference:
ros2 launch bringup mapper.launch.py planner_implementation:=python

# Planner alone, same YAML precedence and planner_node name:
ros2 launch bringup planner.launch.py planner_implementation:=cpp
```

Both launches retain `params_file` and `nbv_params_file`; the main parameters file
wins. The full mapper also forwards the existing `holo_config_file` argument.
`ros2 launch bringup mapper.launch.py` now selects C++ by default. Both launch
files and all operational YAML are owned and installed by `bringup`. The
generated defaults snapshot is migration evidence, not a second configuration
profile. No launch or operational configuration files are installed by the
planner package.

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
* Moved launch ownership and planner selection into `bringup`; retained runtime
  YAML there and moved the generated defaults snapshot into migration evidence.
* Added deterministic differential replay, frozen recorded-input fixtures, ROS
  integration checks, file-format tests, and reproducible benchmarks.
* No commits were created.
