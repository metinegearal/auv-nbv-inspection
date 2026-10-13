"""Verify the shipped DLQR speed profile with limited motor commands."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from motion_control.dlqr import FeedbackSystem, waypoint_velocity
from motion_control.thrusters import force_to_thrust, thrust_to_force, MAX_THRUST


def shipped_profile():
    config = Path(__file__).parents[2] / 'bringup/config/params.yaml'
    return yaml.safe_load(config.read_text())['waypoint_control_node']['ros__parameters']


def step_response(system, axis, distance):
    state = np.zeros(6)
    reference = np.zeros(6)
    reference[axis + 3] = distance
    positions, speeds = [], []
    for _ in range(int(12. / system.dt)):
        command = force_to_thrust(system.control_force(state, reference), perfect=system.buoyancy_force == 0.)
        command *= min(1., MAX_THRUST / max(np.abs(command).max(), 1e-9))
        assert np.abs(command).max() <= MAX_THRUST + 1e-10
        applied = thrust_to_force(command, perfect=system.buoyancy_force == 0.)[:3]
        state = system.predict(state, applied)
        positions.append(state[axis + 3])
        speeds.append(abs(state[axis]))
    positions = np.asarray(positions)
    near = np.flatnonzero(abs(positions - distance) <= abs(distance) * .1)
    assert len(near), 'Target must be reached'
    progress = positions / distance
    return (near[0] + 1) * system.dt, max(speeds), progress.max() - 1., positions[-1]


@pytest.mark.parametrize('axis,distance', [(0, .5), (1, 4.), (2, -.5)])
@pytest.mark.parametrize('perfect', [True, False])
def test_shipped_profile_is_faster_with_bounded_overshoot(axis, distance, perfect):
    config = shipped_profile()
    baseline = FeedbackSystem(perfect=perfect)
    tuned = FeedbackSystem(perfect=perfect, state_weights=config['state_weights'],
                           force_weights=config['force_weights'])
    old_time, old_speed, _, _ = step_response(baseline, axis, distance)
    time, speed, overshoot, final = step_response(tuned, axis, distance)
    assert time < .6 * old_time
    assert speed > 1.5 * old_speed
    assert overshoot < .03
    assert final == pytest.approx(distance, abs=.001)


def test_shipped_profile_retains_stable_damped_position_feedback():
    config = shipped_profile()
    system = FeedbackSystem(state_weights=config['state_weights'],
                            force_weights=config['force_weights'])
    assert np.abs(np.linalg.eigvals(system.A - system.B @ system.K_state)).max() < 1.
    for metrics in system.damping_metrics().values():
        assert metrics['zeta'] >= .75


def test_cruise_reference_keeps_speed_on_straights_and_brakes_for_turns():
    position, target = np.zeros(3), np.array([.5, 0., 0.])
    straight = waypoint_velocity(position, target, [1., 0., 0.])
    corner = waypoint_velocity(position, target, [.5, 1., 0.])
    end = waypoint_velocity(position, target)
    np.testing.assert_allclose(straight, [1., 0., 0.])
    assert 0 < np.linalg.norm(corner) < np.linalg.norm(straight)
    np.testing.assert_allclose(corner, end)
    np.testing.assert_array_equal(waypoint_velocity(target, target), np.zeros(3))


def traverse_queue(system, cruise_speed):
    targets = [np.array([x, 0., 0.]) for x in np.arange(.5, 5.01, .5)]
    state = np.zeros(6)
    elapsed = 0.
    peak_motor = 0.
    while targets and elapsed < 60.:
        target = targets[0]
        following = targets[1] if len(targets) > 1 else None
        desired = waypoint_velocity(state[3:], target, following, speed=cruise_speed,
                                     braking_acceleration=.8, stopping_distance=.4)
        reference = desired if np.linalg.norm(desired) > 1e-9 else np.r_[np.zeros(3), target]
        force = system.control_force(state, reference if cruise_speed > 0 else np.r_[np.zeros(3), target])
        command = force_to_thrust(force)
        command *= min(1., MAX_THRUST / max(np.abs(command).max(), 1e-9))
        peak_motor = max(peak_motor, np.abs(command).max())
        state = system.predict(state, thrust_to_force(command)[:3])
        elapsed += system.dt
        if np.linalg.norm(target - state[3:]) <= .4:
            targets.pop(0)
    assert not targets, 'Queue must finish within the test horizon'
    # At the final point the same position controller must settle and hold.
    for _ in range(300):
        force = system.control_force(state, [0., 0., 0., 5., 0., 0.])
        command = force_to_thrust(force)
        command *= min(1., MAX_THRUST / max(np.abs(command).max(), 1e-9))
        state = system.predict(state, thrust_to_force(command)[:3])
    np.testing.assert_allclose(state[3:], [5., 0., 0.], atol=.005)
    return elapsed, peak_motor


def test_cruise_finishes_short_waypoint_queue_faster_and_holds_final_target():
    config = shipped_profile()
    system = FeedbackSystem(state_weights=config['state_weights'],
                            force_weights=config['force_weights'],
                            velocity_weights=config['velocity_weights'])
    stop_time, _ = traverse_queue(system, 0.)
    cruise_time, peak = traverse_queue(system, config['waypoint_cruise_speed'])
    assert cruise_time < .65 * stop_time
    assert peak <= MAX_THRUST
