# Migration validation results

Initial runtime validation: 2026-10-13. Planner runtime belongs to `auv_inspection_planner`; launch composition and operational configuration belong to `auv_bringup`.

Launch ownership update (2026-10-14): both `auv_bringup` launches were rebuilt and checked through the installed launch CLI. Eight composition tests passed, covering default C++, explicit C++/Python selection, invalid selection, single-planner composition, and main-file parameter precedence. See `migration_evidence/bringup_launch_tests.txt`.

## Build and correctness

* Isolated Release colcon build/install: passed.
* Differential tests: **177 passed** (current Python source versus C++).
* File telemetry equivalence: **1 passed**.
* ROS and selectable-launch tests: **5 passed**, including default and shipped configurations for both nodes.
* Existing C++ policy regression: passed. Targeted CTest run: **3/3 passed**.
* Full automatic lint suite was not certified; reference lint imports are unavailable, and vendor code retains upstream formatting.
* Python source hash guard passed. No changes were committed.

Logs: `migration_evidence/{build,ctest,differential_tests,telemetry_tests,ros_tests}.txt`.

## Numerical errors

Discrete decisions, ordering, indices, reasons, and state transitions match in these fixtures. Ordinary floating-point outputs use absolute and relative tolerances of 1e-9. Optional optimization uses 2e-6 absolute and relative tolerances (metres/radians), because finite differences amplify rounding differences.

| Component | Largest observed absolute difference |
| --- | ---: |
| FITPACK spline | 0 |
| PCA | 1.52655666e-16 |
| Open3D preprocessing | 0 |
| NBV including recorded map | 5.32907052e-15 |
| L-BFGS-B trajectory | 5.01900695e-07 |
| OctoMap extraction | 0 |

## Performance method

Host: 13th Gen Intel(R) Core(TM) i7-13620H; Linux-7.0.0-38-generic-x86_64-with-glibc2.39.

Python 3.12, NumPy 1.26.4, SciPy 1.16.3, Open3D 0.19.0; GCC/GFortran 13, Release C++17. Two warmups followed by 15 measurements per workload, one OpenMP/BLAS thread. Both implementations receive the same serialized fixtures in persistent worker processes. Each workload gets a fresh worker for independent memory measurements. Stage measurements use internal monotonic clocks; process startup and pipe transport are excluded. Operation timings include adapter/replay construction and trace capture. Peak memory uses Linux `/proc/self/status` VmHWM, avoiding inherited pre-exec `getrusage` peaks. The machine also ran an existing mission, so CPU contention and variance remain visible. This is a reproducible microbenchmark, not a controlled full simulator mission comparison.

| Workload | Python median ± SD (ms) | C++ median ± SD (ms) | C++/Python | Peak RSS Python / C++ (MiB) |
| --- | ---: | ---: | ---: | ---: |
| astar_obstacles_0 | 0.233 ± 0.036 | 0.099 ± 0.013 | 0.422 | 304.8 / 30.2 |
| inflate_1.0 | 0.805 ± 0.171 | 0.609 ± 0.131 | 0.756 | 304.8 / 30.3 |
| spline_30_2.0 | 1.952 ± 0.188 | 1.902 ± 0.327 | 0.975 | 305.3 / 31.5 |
| pca_random | 2.235 ± 0.083 | 1.415 ± 0.238 | 0.633 | 305.7 / 31.0 |
| clusters_plane | 7.128 ± 1.051 | 2.881 ± 0.316 | 0.404 | 305.4 / 30.6 |
| preprocessing_corner | 1.676 ± 0.139 | 1.447 ± 0.058 | 0.864 | 305.0 / 31.1 |
| nbv_corner_0 | 27.131 ± 20.108 | 8.245 ± 0.855 | 0.304 | 306.5 / 31.9 |
| optimizer_10 | 170.514 ± 7.816 | 2.490 ± 0.076 | 0.015 | 305.7 / 30.8 |
| octomap_rays | 20.702 ± 3.371 | 0.815 ± 0.068 | 0.039 | 304.9 / 30.3 |
| depth_cap_randomstate | 4.113 ± 0.312 | 5.271 ± 0.519 | 1.281 | 309.2 / 32.2 |
| unsafe_spline_corner_fallback | 6.857 ± 17.563 | 0.672 ± 0.075 | 0.098 | 304.9 / 30.7 |
| recorded_candidates_248 | 35.383 ± 3.467 | 2.798 ± 0.136 | 0.079 | 306.7 / 30.9 |
| recorded_route_248 | 0.162 ± 0.063 | 0.013 ± 0.003 | 0.080 | 304.4 / 30.2 |
| recorded_pose_history | 3.786 ± 18.147 | 1.603 ± 0.055 | 0.423 | 305.2 / 30.9 |
| recorded_map_nbvs | 442.567 ± 33.018 | 358.112 ± 18.029 | 0.809 | 327.0 / 41.2 |

All per-run samples, p95, min/max, means, standard deviations and throughput values are retained in `migration_evidence/benchmark.json`.

## Full synthetic depth/map planning cycle

A 30×40 deterministic depth frame runs through real ColorOcTree insertion, extraction, inflation, NBV, routing, validation and actual map-file serialization. This is synthetic sensor input, not a recorded mission. Timings below are internal stage latencies.

| Stage | Python median ± SD (ms) | C++ median ± SD (ms) |
| --- | ---: | ---: |
| replay_total | 75.980 ± 4.721 | 20.518 ± 2.508 |
| depth_to_octomap | 3.333 ± 0.709 | 3.409 ± 0.531 |
| octomap_extract | 28.898 ± 2.614 | 0.398 ± 0.085 |
| collision_inflation | 2.002 ± 0.259 | 0.259 ± 0.068 |
| octomap_save | 0.457 ± 0.070 | 0.387 ± 0.153 |
| nbv_downsample | 0.138 ± 0.031 | 0.107 ± 0.031 |
| nbv_outlier_filter | 1.802 ± 0.223 | 1.386 ± 0.315 |
| nbv_normal_estimation | 1.463 ± 0.188 | 1.345 ± 0.218 |
| nbv_normal_orientation | 0.018 ± 0.004 | 0.008 ± 0.002 |
| nbv_frontier_detection | 3.105 ± 0.450 | 1.753 ± 0.367 |
| nbv_clustering | 1.749 ± 0.355 | 0.168 ± 0.031 |
| nbv_pca | 0.757 ± 0.129 | 0.443 ± 0.055 |
| nbv_candidate_evaluation | 15.021 ± 1.528 | 7.421 ± 0.838 |
| nbv_generation | 25.974 ± 2.441 | 13.254 ± 1.782 |
| route_search | 3.182 ± 1.111 | 0.117 ± 0.010 |
| trajectory_generation_and_validation | 1.705 ± 0.199 | 0.183 ± 0.017 |
| trajectory_publish | 1.274 ± 0.148 | 0.025 ± 0.003 |
| planning_cycle_total | 73.206 ± 4.477 | 19.056 ± 2.438 |

Full-cycle median speedup: **3.84×**. Mean-derived throughput: **13.59 versus 50.47 cycles/s**. Peak worker RSS: **312.7 versus 32.8 MiB** (Python versus C++).

## Regression investigation and remaining limits

The capped-depth **mock-map replay** regressed in the final measurement. That fixture materializes a 4,000-point insertion trace and many JSON state snapshots, unlike the live OctoMap adapter. It is retained as an observed replay/adapter regression; it is not hidden by averaging other workloads.

To investigate, the same capped frame was rerun against **real OctoMap**. The measured `depth_to_octomap` stage was 7.324 ± 0.468 ms in Python and 8.424 ± 0.308 ms in C++. This stage still includes insertion-trace capture, so it does not isolate production projection/insertion from harness serialization overhead. The capped-depth stage regression remains unresolved despite the faster complete cycle; see `capped_depth_benchmark.json` for all stages and samples.

No raw depth recording or complete mission ROS bag was available. Frozen map/candidate/pose/route output logs and synthetic sensor scenarios were used. Full recorded-sensor mission equivalence and live-mission performance are therefore unverified. Unseeded random subsamples differ across runs. Console numeric formatting and JSON whitespace/order are not byte-identical. The known baseline failures are documented in MIGRATION.md and were not fixed.

## Reproduce benchmarks

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 src/auv_inspection_planner/test/migration/benchmark.py \
  --cpp /tmp/planner-cpp-build/planner_replay --repeats 15 --output /tmp/planner-benchmark.json
PLANNER_REPLAY=/tmp/planner-cpp-build/planner_replay \
python3 src/auv_inspection_planner/test/migration/benchmark_pipeline.py /tmp/planner-pipeline.json
PLANNER_REPLAY=/tmp/planner-cpp-build/planner_replay \
python3 src/auv_inspection_planner/test/migration/benchmark_pipeline.py /tmp/planner-cap.json depth_cap_randomstate
```

Replace the executable with your installed `lib/auv_inspection_planner/planner_replay` path as needed.
