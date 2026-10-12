# Measured motion control responses

`run_baselines` observes `/trajectory/waypoint` and `holocean/odom`. It measures
the first XYZ target in the waypoint array, matching the controller's active
command. Repeated commands and yaw-only changes do not reset a position response.
Changing the position ends the previous episode and starts a new one. A command
received before odometry starts its episode at the first valid position sample.
All measurements stay in the evaluator; the controller and simulator are unchanged.

Every received odometry sample updates the response. Live ROS log summaries show:

- Rise time from 10% to 90% projected progress, with interpolated crossings.
- t63: time to 63.2% progress. This is a measured response time, not a fitted
  time constant except for a first-order response.
- Provisional settling time, using full 3D position error inside a band of 2%
  of the initial step distance. Leaving the band revokes the settling time.
- Time continuously observed inside the band and whether the configured dwell
  was met. Dwell confirmation still describes only the observation window.
- Projected overshoot percentage, time of the greatest overshoot, current
  position error, IAE (m*s), and ISE (m²*s).

Progress and overshoot are projected onto the initial-position-to-target line;
cross-track error still affects settling, IAE, and ISE. Zero-distance targets
have no response times. Unreached thresholds remain pending in live logs and
blank/N/A in reports. A target replaced before settling remains an interrupted
observation, not evidence that the controller eventually settled. Closely spaced
trajectory waypoints may therefore have many pending response times.

Waypoint messages contain no timestamp, so elapsed time uses the evaluator's
monotonic callback receipt clock for both commands and odometry. This measures
wall-clock response including delivery delay, not simulator physics time or
command-to-actuator latency. The initial position is the last received odometry;
its age depends on the odometry rate. No nominal plant poles, damping ratios,
estimator performance, or force model are inferred from these observations.

## Saved outputs

On `/movement/finished_execution` with `data: true`, or graceful evaluator
shutdown, the existing `results_dir` also receives:

- `motion_response_metrics.csv`: one row per position-target episode, including
  its initial/target position, end reason, response times, errors, and sample counts.
- `motion_response_samples.csv`: elapsed time, progress, and full/component
  position error for retained samples.
- `motion_response_summary.txt`: readable metrics and measurement definitions.
- `motion_control_response.png`: progress, full error, response times, and
  overshoot for the last 12 valid steps. Missing response times have no bar.

The samples retain at most 3000 observations per episode. Crossings, integrals,
and peaks include all observations even when older chart samples are discarded.
CSV metadata gives total and retained sample counts. No response chart is
generated if no nonzero position step was observed. Existing benchmark outputs
and their completion behavior remain available.

## Evaluator parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `response_band_fraction` | `0.02` | Settling band / initial distance |
| `response_settling_dwell_s` | `1.0` | Observed continuous band dwell for confirmation |
| `response_target_tolerance_m` | `0.00001` | Position-command deduplication tolerance |
| `response_max_samples` | `3000` | Retained samples per response |
| `response_log_period_s` | `1.0` | Live summary interval, also logged at start/end |

These are parameters of the `run_baselines` evaluator node. For example:

```bash
ros2 run benchmark_evaluator run_baselines --ros-args \
  -p results_dir:=/tmp/controller_evaluation \
  -p response_settling_dwell_s:=2.0
```
