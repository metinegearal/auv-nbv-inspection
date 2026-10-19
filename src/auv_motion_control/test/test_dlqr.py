"""Check the implemented plant and feedback against independent equations."""

from auv_benchmark.domain.response import ResponseMonitor
from auv_motion_control.dlqr import FeedbackSystem
from auv_motion_control.thrusters import force_to_thrust, MAX_THRUST, thrust_to_force
import numpy as np
import pytest
from scipy.linalg import expm, solve_discrete_are


@pytest.mark.parametrize('damping', [0., 1e-12, 3., 300.])
@pytest.mark.parametrize('dt', [1. / 30., 0.2])
def test_zoh_matches_augmented_matrix_exponential(damping, dt):
    system = FeedbackSystem(dt, damping=damping, perfect=False)
    continuous = np.zeros((9, 9))
    continuous[:3, :3] = -np.eye(3) * damping / system.mass
    continuous[3:6, :3] = np.eye(3)
    continuous[:3, 6:] = np.eye(3) / system.mass
    discrete = expm(continuous * dt)
    np.testing.assert_allclose(system.A, discrete[:6, :6], atol=1e-14)
    np.testing.assert_allclose(system.B, discrete[:6, 6:], atol=1e-14)
    np.testing.assert_allclose(system.d, system.B @ system.buoyancy)


def test_gains_solve_dare_and_poles_match_equivalent_metrics():
    system = FeedbackSystem()
    a, b = system.A, system.B
    q, r = np.diag([80.] * 3 + [16.] * 3), np.eye(3) * 0.1
    p = solve_discrete_are(a, b, q, r)
    gain = np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)
    np.testing.assert_allclose(system.K_state, gain, atol=1e-12)
    closed_loop = a - b @ gain
    assert np.max(np.abs(np.linalg.eigvals(closed_loop))) < 1
    for metrics in system.damping_metrics().values():
        np.testing.assert_allclose(np.exp(metrics['poles'] * system.dt),
                                   metrics['discrete_poles'])
        assert metrics['zeta'] > 0
        assert metrics['wn'] > 0
        assert metrics['dominant_tau'] > 0


@pytest.mark.parametrize('perfect', [True, False])
def test_hover_and_moving_reference_are_exact_solutions(perfect):
    system = FeedbackSystem(perfect=perfect)
    state = np.array([0., 0., 0., 10., -5., -30.])
    force = system.control_force(state, state)
    np.testing.assert_allclose(force, -system.buoyancy, atol=1e-12)
    np.testing.assert_allclose(system.predict(state, force), state, atol=1e-12)
    state[:3] = [1., -0.5, 0.2]
    for reference in (state.copy(), state[:3].copy()):
        force = system.control_force(state, reference)
        np.testing.assert_allclose(force, system.damping * state[:3] - system.buoyancy)
        expected = state.copy()
        expected[3:] += state[:3] * system.dt
        np.testing.assert_allclose(system.predict(state, force), expected, atol=1e-12)


@pytest.mark.parametrize('velocity_weight', [80., 40.])
def test_nominal_step_matches_closed_loop_and_monitor_integrals(velocity_weight):
    system = FeedbackSystem(state_weights=[velocity_weight] * 3 + [16.] * 3)
    state = np.zeros(6)
    target = np.array([0., 0., 0., 10., 0., 0.])
    monitor = ResponseMonitor(state[3:], target[3:])
    closed_loop = system.A - system.B @ system.K_state
    theoretical_error = state - target
    distances = [10.]
    motor_peak = 0.
    for step in range(1, 1501):
        force = system.control_force(state, target)
        motor_peak = max(motor_peak, np.max(np.abs(force_to_thrust(force))))
        state = system.predict(state, force)
        theoretical_error = closed_loop @ theoretical_error
        np.testing.assert_allclose(state, target + theoretical_error, atol=2e-11)
        monitor.update(step * system.dt, state[3:])
        distances.append(np.linalg.norm(target[3:] - state[3:]))
    metrics = monitor.metrics()
    assert motor_peak < MAX_THRUST
    assert metrics['settling_dwell_met']
    assert metrics['rise_time_s'] > 0
    assert metrics['final_error_m'] < 1e-8
    # The equivalent second-order formula is approximate for sampled output.
    values = system.damping_metrics()['x']
    zeta, wn = values['zeta'], values['wn']
    if zeta < 1:
        expected_overshoot = 100 * np.exp(-np.pi * zeta / np.sqrt(1 - zeta**2))
        expected_peak = np.pi / (wn * np.sqrt(1 - zeta**2))
        assert metrics['overshoot_pct'] == pytest.approx(expected_overshoot, abs=0.02)
        assert metrics['peak_time_s'] == pytest.approx(expected_peak, abs=system.dt)
    else:
        assert metrics['overshoot_pct'] == 0.
        assert metrics['peak_time_s'] is None
    assert metrics['iae_m_s'] == pytest.approx(np.trapz(distances, dx=system.dt))
    assert metrics['ise_m2_s'] == pytest.approx(np.trapz(np.square(distances), dx=system.dt))


@pytest.mark.parametrize('perfect', [True, False])
def test_thruster_allocation_preserves_force_and_torque_under_uniform_limit(perfect):
    wrench = np.array([300., -200., 100., 0., 0., 3.])
    command = force_to_thrust(wrench, perfect=perfect)
    np.testing.assert_allclose(thrust_to_force(command, perfect=perfect), wrench, atol=1e-12)
    scale = min(1., MAX_THRUST / np.max(np.abs(command)))
    assert scale < 1
    command *= scale
    assert np.max(np.abs(command)) <= MAX_THRUST
    np.testing.assert_allclose(thrust_to_force(command, perfect=perfect), wrench * scale,
                               atol=1e-12)


@pytest.mark.parametrize('options', [
    {'dt': 0.}, {'dt': float('nan')}, {'mass': -1.}, {'damping': -1.},
    {'state_weights': [0.] * 6}, {'force_weights': [float('inf')] * 3},
])
def test_invalid_configuration_is_rejected(options):
    with pytest.raises(ValueError):
        FeedbackSystem(**options)


def test_invalid_inputs_are_rejected():
    system = FeedbackSystem()
    for state, reference in [(np.zeros(3), np.zeros(3)),
                             (np.zeros(6), np.zeros(4)),
                             (np.zeros(6), [0., 0., float('nan')])]:
        with pytest.raises(ValueError):
            system.control_force(state, reference)
    with pytest.raises(ValueError):
        system.predict(np.zeros(6), np.zeros(3), -1.)
