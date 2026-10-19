
# Changelog

Historical entries summarize committed changes. The October 11 entry is on
`arch/tf2_transform` and is not an ancestor of the current branch,
`enhancement/lqr_motion_control`.

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

Oct 18, 2026 — Uncommitted

fix: retain distant NBVs and improve routing, collision checks, and blacklisting

- Retain previously discovered clusters outside the local update radius and recalculate their NBV scores using current distance, heading, and risk; increase distance and route-cost penalties.
- Correct occupied-cell geometry and clearance inflation, validate complete routes and trajectory fallbacks, and require safe retreat completion before resuming inspection.
- Try alternative viewpoints around a target using spherical ray sampling and bounded A* searches; permanently blacklist the surrounding region for the mission when every sampled viewing direction is blocked.
- Filter observed, blacklisted, and retry-exhausted targets before normalizing scores and selecting cluster viewpoints; recalculate scores when routing adds a new blacklist region.
- Bound repeated target attempts and require measured observation completion, with regression coverage for replanning, failed execution, cooldown expiry, and recovery.
- Expose retry and collision blacklist radii through bringup's `route_blacklist_radius_m` and `route_collision_blacklist_radius_m` parameters.
- Save permanent ban spheres in the benchmark package and display their configured radii in `test_env_2.py`.
- Move planner launch/configuration ownership to bringup and add incident replay fixtures and a mandatory pre-install regression gate.
