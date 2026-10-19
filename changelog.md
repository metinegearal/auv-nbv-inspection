
# Changelog

Historical entries summarize committed changes; pending work is marked
Uncommitted. The October 11 entry is retained as separate-branch history from
`arch/tf2_transform`.

Sep 12, 2026 — `abc1610`

feat: establish the ROS 2 underwater inspection workspace

- Add the six initial packages for simulation, perception, mapping/NBV planning, motion control, bringup, and benchmark evaluation.
- Integrate HoloOcean sensors, BGNet stereo depth, OctoMap mapping, A* routing, and waypoint execution.
- Add simulator configuration, launch files, baseline evaluation scripts, and initial experiment outputs.

Sep 28, 2026 — `c101ba5`

feat: add Python NBV planning and Optuna parameter optimization

- Add `python_planner` with viewpoint generation, utility scoring, and route planning.
- Add Optuna-driven mission tuning, standalone simulation scripts, and benchmark documentation.
- Update stereo depth processing, mapper integration, and waypoint tracking.

Sep 29, 2026 — `b8baf01`

feat: record mid-run NBV evaluation data

- Add shared mid-run logging for depth poses, planned trajectories, NBV iterations, and executed trajectories.
- Add plotting support and document the generated evaluation files under `results/`.

Sep 29, 2026 — `103b380`

perf: record pipeline timings and reduce planning overhead

- Log perception and planning timings alongside mission evaluation data.
- Adjust collision inflation and route checks, cap depth points inserted per frame, and tune planning and motion parameters.

Sep 29, 2026 — `4dded54`

perf: accelerate occupied-point extraction and incremental map updates

- Cache occupied and collision voxels for incremental map processing.
- Limit redundant observation-history entries and improve NBV computation and baseline reporting.
- Move experiment outputs into `results/` and remove obsolete root-level results.

Sep 30, 2026 — `32e188c`

feat: improve trajectory tracking and evaluation plots

- Refine waypoint tracking and trajectory smoothing behavior.
- Extend baseline reports with planned-versus-executed trajectories, heading error, and speed over time.
- Update mid-run logging and Optuna optimization support.

Oct 3, 2026 — `722b56f`

feat: centralize tuning and improve motion continuity during replanning

- Consolidate planner and controller tuning in bringup configuration.
- Add bounded trajectory optimization using measured velocity and recent motion history to reduce reversals and retracing.
- Add motion-continuity regressions, planner documentation, and a trajectory/map viewer.

Oct 4, 2026 — `93d9d17`

feat: expose NBV timing and filter diagnostics

- Record NBV stage timings and candidate-filter decisions; add audit and geometry documentation.
- Batch PCA and frontier computations and add bounded route-search support.
- Separate NBV tuning into `nbv_params.yaml` and add configuration, geometry, filter-logging, and route-limit tests.

Oct 5, 2026 — `3cf7c92`

refactor: separate ROS adapters from inspection and evaluation logic

- Introduce shared AUV contracts and separate domain logic, use cases, and adapters across the ROS packages.
- Add dedicated experiment and telemetry packages and central configuration loading.
- Document package architecture and the exploration pipeline, with policy regression coverage.

Oct 10, 2026 — `d790e63`

feat: align NBV viewpoints with the upward-looking camera

- Select an upward view axis at 45 degrees using occupied-point clearance and heading tie-breaking.
- Retain cone offsets around that axis and prioritize NBV utility during route selection.
- Add view-axis and route-priority regression tests and document the new parameters.

Oct 11, 2026 — `3c190ce` (branch: `arch/tf2_transform`)

feat: add TF2 camera transforms and the NBV action interface

- Add `nbv_interfaces/action/ComputeNBV` and action-based route requests, feedback, cancellation, and failure handling.
- Publish the vehicle/camera TF chain and use capture-time transforms for depth projection and vehicle-coordinate waypoints.
- Add sensor-readiness waits, mission strikeout handling, and ROS TF/action integration tests.

Oct 12, 2026 — `fe3e5f0`
feat: add DLQR control and response benchmarks, optimize NBV planning

- Add exact-ZOH DLQR tracking with buoyancy compensation and legacy controller support.
- Add transient response metrics, monitoring, and plots.
- Improve local frontier detection, NBV evaluation performance, and A* search.
- Make exploration radius and route candidate limits configurable.
- Update bringup tuning, documentation, tests, and experiment results.

Oct 13, 2026 — `abd0adf`

feat: improve NBV planning, routing, and waypoint control

- Improve NBV scoring by reducing information gain for seabed points and keeping the top three viewpoints per cluster.
- Update frontier detection, route recovery, A* failure handling, and spline fallbacks.
- Refresh OctoMap collision checks, inflation, route validation, and replanning behavior.
- Tune waypoint control with stronger position feedback, a 0.8 m/s cruise speed, braking, and speed logging.
- Fix BGNet imports, add prediction tracking, update assets and ignore rules, and remove obsolete files.

Oct 14, 2026 — `e1ed7ec`

feat: migrate Python inspection planner to C++

- Add `inspection_planner_node` in `active_inspection_planner`, preserving Python planner behavior and ROS interfaces.
- Port NBV generation, OctoMap mapping, collision checks, routing, trajectory optimization, and mission recovery to C++.
- Add selectable C++/Python launches, differential tests, replay fixtures, and performance benchmarks.
- Validate with 177 passing differential tests; measure a 3.84× speedup in the synthetic planning-cycle benchmark.

Oct 18, 2026 — `5f58a65`

fix: retain distant NBVs and improve routing, collision checks, and blacklisting

- Retain previously discovered clusters outside the local update radius and recalculate their NBV scores using current distance, heading, and risk; increase distance and route-cost penalties.
- Correct occupied-cell geometry and clearance inflation, validate complete routes and trajectory fallbacks, and require safe retreat completion before resuming inspection.
- Try alternative viewpoints around a target using spherical ray sampling and bounded A* searches; permanently blacklist the surrounding region for the mission when every sampled viewing direction is blocked.
- Filter observed, blacklisted, and retry-exhausted targets before normalizing scores and selecting cluster viewpoints; recalculate scores when routing adds a new blacklist region.
- Bound repeated target attempts and require measured observation completion, with regression coverage for replanning, failed execution, cooldown expiry, and recovery.
- Expose retry and collision blacklist radii through bringup's `route_blacklist_radius_m` and `route_collision_blacklist_radius_m` parameters.
- Save permanent ban spheres in the benchmark package and display their configured radii in `test_env_2.py`.
- Move planner launch/configuration ownership to bringup and add incident replay fixtures and a mandatory pre-install regression gate.

Oct 18, 2026 — `88d5fb8`

feat: add an incremental live map and planned-trajectory viewer

- Add a separate benchmark ROS node showing occupied map geometry, the latest planned route, and vehicle position in Open3D.
- Decode stable map snapshots on a background worker, retain unchanged distant chunks, remove deleted chunks, and bound geometry uploads per frame.
- Add default full-system startup, a standalone viewer launch, a headless toggle, and bringup configuration for map polling, rendering, and display density.
- Clear routes on empty plans or mission completion and add snapshot, incremental-update, launch, and ROS subscription checks.

Oct 18, 2026 — `ed9639e`

feat: show targeted frontiers in the live map

- Display the active frontier centroid with a yellow wire sphere and a line from the planned viewpoint.
- Follow existing planner diagnostics using bounded initial log reads and incremental background polling, without recomputing NBV geometry.
- Clear target overlays on cancellation, observation, completion, or live route clears; expose overlay settings in bringup.

Oct 19, 2026 — Uncommitted

refactor: standardize package names, declare licensing, and clean live map displays

- Rename `bringup` to `auv_bringup`, `sim_bridge` to `auv_sim_bridge`, `perception` to `auv_stereo_depth`, and `active_inspection_planner` to `auv_inspection_planner`.
- Rename `python_planner` to `auv_planner_reference`, `motion_control` to `auv_motion_control`, `benchmark_evaluator` to `auv_benchmark`, and `telemetry_adapters` to `auv_telemetry`.
- Update package metadata, Python imports, C++ include paths, launch references, tests, documentation, and the BGNet submodule path while preserving runtime interfaces.
- Support a separate planner regression build through `check_before_install.py --build-dir`; ignore `build_renamed/`, `install_renamed/`, and `log_renamed/`. Keep frozen reference manifests and incident fixtures intact, normalizing only package-name and license-metadata changes for source-integrity checks.
- Declare Apache-2.0 for project-authored code and documentation using one root `LICENSE` and `NOTICE`, with matching package metadata and centralized third-party notices and license texts.
- Preserve SciPy and nlohmann/json terms and document unresolved BGNet/model permissions; omit duplicate per-package license files and installation rules.
- Rewrite the README around adaptive NBV inspection, tangent-plane frontiers, PCA complexity, constrained viewpoints, and gain-versus-cost scoring; highlight scoring in Mermaid while preserving technical instructions. Record planned routing and reconstruction improvements.
- Add display-only statistical outlier removal after downsampling, following the filtering approach in `test_env_2.py`. Preserve measured point centroids instead of snapping displayed positions to voxel centers.
- Expose `filter_outliers`, `outlier_neighbors`, and `outlier_std_ratio` in bringup. Filter changed snapshots on the worker across chunk boundaries, retain unchanged geometry, and replace/remove points when occupancy or filter results change. Saved occupancy and planner collision data are unchanged.
- Add regressions for movement within a display voxel, point removal, occupied-to-free transitions, newly supported surfaces, chunk-boundary filtering, and ROS parameter overrides.
- Validate package renames with all ten package builds and the seven required planner replay suites. For the viewer cleanup, pass 41 focused checks and rebuild `auv_benchmark` and `auv_bringup` using `build_renamed` and `install_renamed`.
- Compare the viewer with `test_env_2.py` on a frozen saved map: the viewer filter hides 665 statistical outliers, processes a changed snapshot in about 0.59 s, and checks unchanged files in about 0.03 ms. These are saved-map/ROS checks, not a new live simulator mission.
- Refresh recorded mission maps, trajectories, NBV diagnostics, benchmark plots, and permanent-ban snapshots in `results/`.
- Archive unreferenced root drafts, legacy evaluation output, standalone export/plotting scripts, trajectory experiments, and caches under ignored `local/`; preserve helper defaults pointing to workspace results. Keep the active `test_env_2.py`, agent instructions, and Optuna runtime paths in place, and explicitly ignore local tooling directories and regenerated root evaluation logs.
