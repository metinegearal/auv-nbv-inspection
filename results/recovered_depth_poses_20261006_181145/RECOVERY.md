# Recovered evaluation

The benchmark node crashed at startup because its installed auv_contracts package lacked mission_results.py. These reports reconstruct a sampled vehicle trajectory from planner_depth_poses.csv, using its recorded pose timestamps and positions, not the missing benchmark odometry stream.

Recovered 1907 monotonically timed samples. Distance and speed use that sampling; duration is the span of the pose timestamps rather than the evaluator wall-clock mission timer.

The overview includes copied NBV and future-path logs. Heading-error and planned-versus-real metrics are unavailable because those paths have no timestamp alignment with the poses. They are not inferred or fabricated. Original results are unchanged.
