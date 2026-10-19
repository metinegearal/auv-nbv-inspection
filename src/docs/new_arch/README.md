# Implemented architecture documentation

These documents describe this workspace. The earlier [architecture audit](../architecture/clean_architecture_audit_summary.md) remains a historical diagnosis; [refactoring status](../architecture/refactoring_status.md) records dispositions and verification limits.

| Area | Reference |
|---|---|
| Whole-system topology | [System architecture](system_architecture_master_document.md) |
| Topic flow, numerical pipeline and failures | [Exploration pipeline](exploration_pipeline_mechanics.md) |
| Deployment composition | [Bringup](bringup_architecture.md) |
| Python mission / NBV / trajectory policy | [Python planner](python_planner_architecture.md) |
| Alternative C++ mapping/planner | [C++ planner](active_inspection_planner_architecture.md) |
| Controller / queue / thrusters | [Motion control](motion_control_architecture.md) |
| Stereo depth / Torch adapter | [Perception](perception_architecture.md) |
| Simulator session / sensor dispatch | [Simulation bridge](sim_bridge_architecture_documentation.md) |
| Observer metrics / reports | [Benchmark evaluator](benchmark_evaluator_architecture.md) |
| Shared contracts, storage and experiments | [Support packages](support_packages_architecture.md) |
| Parameters and defaults | [Bringup configuration](../../auv_bringup/config/README.md) |
| Full changed/new source and build files | [Complete code bundle](../architecture/refactoring_complete_code.md) |
