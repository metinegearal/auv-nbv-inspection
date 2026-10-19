# DLQR motion control

`ros2 run auv_motion_control waypoint_control_node` now runs the Python DLQR
controller. Existing auv_bringup launch files select this executable automatically
after rebuilding. The previous C++ controller remains available as
`legacy_waypoint_control_node`; run only one motor-command publisher at a time.
The trajectory queue and `auv_planner_reference` are unchanged.

Build with the ROS environment and workspace dependencies available:

```bash
colcon build --packages-up-to auv_motion_control
source install/setup.bash
ros2 run auv_motion_control waypoint_control_node --ros-args \
  -p target_state:='[0.0, 0.0, 0.0, 10.0, 0.0, -10.0]'
```

Python dependencies are NumPy, SciPy, and python-control (`control` when using
pip). The node reuses `auv_benchmark.domain.response.ResponseMonitor`.
The package declares these dependencies along with `rclpy` and message packages.

## Dynamics and tuning

The world-frame state is `[vx, vy, vz, px, py, pz]`, with metres, seconds, and
positive z upward. The default plant has mass 31.02 kg, linear drag 3 N/(m/s),
and sample period 1/30 s. `FeedbackSystem.matrices()` implements the exact ZOH
model, including stable evaluation near zero drag. It exposes `A`, `B`, and
the affine buoyancy disturbance `d`; `predict()` advances this nominal plant.

`control.dlqr` solves the DARE once at startup. State control uses
`Q = diag(80, 80, 80, 16, 16, 16)` and `R = diag(0.1, 0.1, 0.1)`:

```text
F_world = K_state @ (reference - state) + damping * reference_velocity - buoyancy
```

Velocity control uses a separate three-state DARE with `Qv = diag(10, 10, 10)`.
Its six-column gain has zero position columns and `N = Kv + damping * I`, so
constant velocity tracking does not fight position integrators.

With `perfect=true` the model is neutrally buoyant. With `perfect=false` the
upward net buoyancy is `(997 * 0.03554577 - mass) * 9.81` N; the controller
subtracts it from vertical thrust. This parameter must match the simulator.

`mass`, `damping`, `update_period`, `perfect`, `state_weights`,
`velocity_weights`, and `force_weights` are startup-only parameters. Weights
are positive diagonal entries. Restart to redesign the gains. Default
continuous-equivalent poles on each position axis are approximately
`-0.722760` and `-0.564166` s^-1: zeta = 1.007681, wn = 0.638559 rad/s,
dominant tau = 1.772527 s. This tuning is slightly overdamped, so no finite
overshoot peak is expected.

Bringup's `params.yaml` uses a faster waypoint profile:
`state_weights: [80, 80, 80, 256, 256, 256]` and unchanged
`force_weights: [0.1, 0.1, 0.1]`. It increases position urgency while retaining
velocity damping. In the nominal plant, a 0.5 m step reaches 90% in about 2.33 s
instead of 6.17 s, with approximately 1.7% overshoot. The motor saturation limit,
sensor freshness checks, and waypoint completion radius are unchanged. These
are model results; simulator drag and execution speed affect live performance.
Restart the node to apply changes to these startup-only weights. Increasing
the legacy `command_multiplier` does not tune DLQR.

Bringup additionally enables `waypoint_cruise_speed: 1.0` m/s with
`velocity_weights: [400, 400, 400]`. For waypoint commands, DLQR velocity feedback
tracks a cruise reference on straight queue legs instead of requesting zero
velocity at every point. The next queued point determines corner speed;
`waypoint_braking_acceleration: 0.8` m/s^2 sets a square-root braking envelope
before turns and the final point. At the end the position controller brakes and
holds. This parameter shapes the reference, not a guaranteed acceleration limit.
The active waypoint is still reached/reported before the queue advances; no
waypoints are skipped. An empty queue cancels cruising and holds current position.

`waypoint_cruise_speed: 0.0` restores fixed-target waypoint control and is the
standalone node default. Cruise speed and braking settings can be updated at
runtime; gain weights require restart. Explicit `target_state` updates and
`control_mode=velocity` retain their existing semantics. Cruise speed is a
velocity reference, not a hard physical speed bound. Periodic logs show DVL
measured speed, requested speed, and target distance; wall-clock displacement can
be slower if the simulator runs below real time.

## State and commands

The node bypasses estimation. With the default `state_source=dvl`, it reads:

| Topic | Use |
| --- | --- |
| `holocean/odom` | World position directly from pose |
| `holocean/dvl` | Body-frame measured velocity |
| `holocean/mag` | Yaw from the reference magnetic direction `[1, 0, 0]` |
| `/trajectory/waypoint` | First target plus optional remaining `[x,y,z,yaw]` queue points |
| `/movement/finished_execution` | Stop motors when true |

For a provider that supplies complete odometry, `state_source=odom` uses its
position, valid quaternion yaw, and **body-frame** twist. The current bridge
usually leaves twist empty, so this mode is opt-in. Both modes assume a level
vehicle; there is no roll/pitch observer or attitude regulator.

`control_mode=state` tracks `target_state`. Nonzero reference velocity advances
the reference position using ROS elapsed time. `control_mode=velocity` tracks
`target_velocity` and ignores reference position. These parameters can change
at runtime. A waypoint replaces the active state target; repeated identical
waypoints preserve the response episode. A later `target_state` parameter
update replaces that active target again.

The controller rotates world force to body axes and uses the engine's actual
eight-thruster geometry to allocate signed Newton commands. It requests zero
roll/pitch torque and proportional yaw torque toward the waypoint heading
(`kp_yaw`, capped by `max_yaw_command`, now in N m). The allocation uniformly
scales all motors when any request exceeds 77.55 N. Legacy PID distance gains,
command multipliers, minimum thrust, DVL reset offsets, centering modes, and
ArUco angle overrides do not apply to the DLQR controller. World targets are
absolute. The legacy executable retains those behaviors.

`/holo/cmd` retains its eight-value command format. `/motion/report` reports
a fixed target once the full 3D distance is within `stopping_distance`; the
controller continues holding until another target arrives. Position alone
determines waypoint handoff, allowing continuous trajectories through the queue.
On startup, stale required inputs (default timeout 0.5 s), backward clock jumps,
mission completion, and normal shutdown, motors receive zeros. Control resumes
only with fresh state and starts a new response episode. Use ROS simulation
time when `/clock` is available; the timer and moving targets use the same clock.

## Response evaluation

The existing benchmark still observes `/trajectory/waypoint` and odometry and
owns `motion_control_response.png`; its files and charts are not regenerated by
this controller change. For fixed state targets, the node also logs the existing
monitor's 10–90% rise time, provisional 2% settling time, overshoot/peak time,
IAE, and ISE. Zero-distance and moving targets have no position-step metrics.
It logs nominal equivalent poles, zeta, wn, and dominant tau at startup.

Two `geometry_msgs/Vector3Stamped` outputs expose force in the `odom` frame:
`/motion/commanded_force` before motor limiting and `/motion/applied_force`
reconstructed from limited motor commands. Applied force is a geometric
prediction, not a force-sensor measurement. Motor peak and limit scale are
logged alongside response metrics.

Pole-derived metrics describe the nominal unsaturated sampled system. Tests
verify its full step trajectory against `(A - B K)^k` and independently
integrate IAE/ISE. Continuous second-order formulas approximate sampled
responses; saturation, yaw motion, sampling delays, and simulator drag can
change live results. The implementation does not assert exact equality between
those live measurements and nominal theory.

## Verification

```bash
colcon test --packages-select auv_motion_control --ctest-args \
  -R 'dlqr_.*regression|motion_policy_regression' --output-on-failure
```

The regressions cover exact ZOH against a matrix exponential, DARE gains,
hover and moving-reference equilibria, nominal steps with overdamped and
underdamped tuning, allocation/saturation, frame conversion, parameter changes,
waypoint reports, stale state, clock resets, and mission stop behavior.
