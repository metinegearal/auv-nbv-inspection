# Planner update checks

The October 16, 2026 run from 14:29 to 15:07 published 168 routes and made 649
unsuccessful goal-extension checks. Targets near (358.5, 826.2, -297.6) repeatedly
returned, including after route stalls. Earlier checks proved that individual
routes were collision-free but did not exercise the complete attempt lifecycle.

Confirmed gaps addressed by the lifecycle checks:

- Extended viewing positions could lie beyond the old 3 m visited-history range.
  Completion must use the resolved vehicle goal, a measured fresh depth pose,
  valid depth measurements, target-facing yaw, sensor range, and unobstructed
  camera sightline. An all-invalid depth frame cannot mark the target observed.
- Multiple viewpoints of one target could be chained into one route. Execute one
  reachable inspection viewpoint per planning cycle.
- Cancelling a stalled/unsafe route discarded its target without excluding it.
  Preserve the target before clearing the route and apply its cooldown.
- A 30 s cooldown was not a retry limit. Count attempts by spatial target region
  (`route_blacklist_radius_m`, default 0.5 m), independently of changing cluster IDs or cooldown expiry.
  Default maximum: three attempts per region per mission. Observed regions stay
  suppressed. After the last failed search or failed execution, add the region
  to the candidate blacklist for the rest of that mission, with no expiry.
  Earlier failures may use the normal cooldown; cooldown expiry, disabled
  cooldowns, later blacklist calls, and small map updates must not release a
  repeatedly failed region. The final allowed active attempt may still finish.
  Failed goal searches consume the budget even if nothing is published or a
  different target succeeds in that planning pass. Group alternate views of the
  same spatial target into one attempt, rather than counting each view separately.
- Moving around a loop could keep resetting the displacement stall timer.
  Preserve a 120 s attempt deadline across replans of the same active target.
- An occupied destination check did not guarantee that a free destination could
  see its target. Validate the sightline for original and adjusted viewpoints.
- On a collision or occluded NBV, try the selected viewing ray and its outward
  adjustment, then probe the original direction plus a 15-degree spherical grid
  (24 azimuths, 11 elevations, and both poles: 267 probes including the original).
  Sample viewing distances at map resolution within sensor range and the existing
  extension bound. Check raw-map camera sightlines and inflated vehicle endpoints;
  run A* toward clear views with one shared expansion budget. If every sampled
  direction collides, immediately blacklist that surface target and candidates
  within `route_collision_blacklist_radius_m` (default 1 m) in 3D for this mission,
  including candidates in the same planning pass.
  A collision on an executing route must stop motion before probing. Clear rays
  stay eligible for replanning. When all camera views become occluded, cancel
  any old route to that blacklisted region even if its vehicle path is clear.
  A* timeout or disconnected routes must not be
  reported as all rays colliding. This finite sweep cannot prove that no opening
  exists between sampled angles. Retain the retry guard for execution/search failures.
- Recovery could finish immediately after crossing a buffer boundary. Require
  proximity to its actual endpoint as well as restored clearance. The current
  pose is a validation anchor, not an executable first waypoint after a stop.
- Cooldowns or exhausted attempt budgets must not be reported as successful
  exploration completion. Hold when no eligible alternative remains.
- Both blacklist radii are finite positive startup parameters in bringup's
  `params.yaml`. Retry matching, temporary bans, exhausted regions, telemetry,
  saved spheres and viewer filtering must agree on the configured radius.
  Identify collision bans explicitly, not by comparing their radius with 1 m:
  a collision ban may be smaller than a retry ban. Validate before opening result
  files. Preserve historical defaults, and test smaller/larger non-default radii.
- The later October 16 run (18:34-19:04), iteration 1600, exposed normalization
  starvation. An exhausted target with raw gain 70 still set the global gain
  scale; 97 remaining recorded views all fell below utility 0.1. Retain raw views
  until observed targets, exhausted regions, and current blacklists are removed.
  Normalize gain/cost over the eligible pool, then select three views per cluster
  and apply the unchanged minimum utility. After a new collision blacklist,
  rebuild the pool and scores in the same planning call without resetting the
  route candidate budget or repeating an already searched view. A singleton or
  equal positive-gain pool keeps a normalized gain of one; zero information is
  not promoted. The frozen score fixture recovers 18 above-threshold views from
  those 97 records on a controlled empty map; this is not historical flight proof.

## Required command

From the workspace root, with the existing CMake build configured:

```bash
source /opt/ros/jazzy/setup.bash
python3 src/active_inspection_planner/tools/check_before_install.py
```

For a checked local installation, add `--install`. The script rebuilds the replay
and ROS node, verifies that all seven required CTest suites are registered, runs
them, and installs only after success. CTest uses the Python interpreter recorded
in the CMake cache; it must have the Python reference dependencies installed.

Required suites: `planner_differential`, `planner_telemetry`, `planner_nbv_cache`,
`planner_collision_recovery`, `planner_goal_standoff`, `planner_target_lifecycle`,
`planner_eligible_scoring`.
The lifecycle suite includes a frozen incident fixture in
`test/migration/fixtures/repeat_target_20261016.json`. These are recorded spatial
inputs used with controlled map/motion sequences, not a full historical simulation.
It verifies observation beyond 3 m, no false observation while facing away or
occluded, repeated failures with changing cluster IDs, cooldown expiry, moving
loops, alternative-target selection, and no false mission completion. It also
checks that repeated failures enter the actual candidate blacklist, survive
expiry, and are removed before route selection.
The eligible-scoring suite uses `fixtures/score_starvation_20261016.json` to check
the recorded failure, every eligibility exclusion, cooldown expiry, degenerate
score ranges, shortlist ordering, same-call rescaling, and bounded search work.
The goal-view suite also checks alternate directions, vertical openings, complete
enclosure, the 1 m boundary in 3D, same-pass exclusions, active-route collision
rechecks, and the distinction between blocked rays and exhausted A* searches.

Two optional map reproductions need explicitly supplied historical run files.
Do not point them at whichever mutable `results/` files happen to exist. Their
skips do not skip the mandatory synthetic and frozen-input lifecycle regressions.
Preserve live results during checks: do not overwrite `results/`, source logs,
or captured maps. Add a new frozen regression when another failure is found.

A passing replay gate does not establish live behavior. After a new simulator
run, inspect `inspection_attempt_started`, `inspection_target_observed`,
`inspection_attempt_timeout`, `inspection_target_suppressed`,
`inspection_waiting`, and `trajectory_cancelled`. Check measured movement and
observation completion, not only `published=true`. Report any missing live
validation explicitly.
