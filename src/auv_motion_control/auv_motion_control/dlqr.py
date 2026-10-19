"""Exact ZOH plant and DLQR feedback; no state estimation or ROS dependencies."""

import control
import numpy as np


def vector(value, size, name):
    """Return a finite vector with an explicit dimension."""
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f'{name} must contain {size} finite values')
    return result.copy()


def waypoint_velocity(position, target, next_target=None, *, speed=1.0,
                      braking_acceleration=0.8, stopping_distance=0.4):
    """Cruise toward the active waypoint and brake for corners or a final stop.

    The next queue point sets corner speed; the active target is never skipped.
    Speed is a reference magnitude, not a hard bound on physical velocity.
    """
    position, target = vector(position, 3, 'position'), vector(target, 3, 'target')
    if not np.isfinite([speed, braking_acceleration, stopping_distance]).all():
        raise ValueError('waypoint speed settings must be finite')
    if speed < 0 or braking_acceleration <= 0 or stopping_distance < 0:
        raise ValueError('waypoint speed settings must be nonnegative with positive acceleration')
    delta = target - position
    distance = np.linalg.norm(delta)
    if distance < 1e-9:
        return np.zeros(3)
    direction = delta / distance
    corner_speed = 0.0
    if next_target is not None:
        following = vector(next_target, 3, 'next_target') - target
        length = np.linalg.norm(following)
        if length > 1e-9:
            corner_speed = speed * np.clip(np.dot(direction, following / length), 0., 1.)
    braking_distance = max(0., distance - stopping_distance)
    requested_speed = min(speed, np.sqrt(corner_speed**2 + 2 * braking_acceleration * braking_distance))
    return direction * requested_speed


class FeedbackSystem:
    """
    World state [vx, vy, vz, px, py, pz], force [Fx, Fy, Fz] in N.

    The linear drag is a nominal tuning model, not simulator ground truth.
    Buoyancy is the net upward force after gravity, in a z-up frame.
    """

    def __init__(self, dt=1.0 / 30.0, *, mass=31.02, damping=3.0, perfect=True,
                 state_weights=(80., 80., 80., 16., 16., 16.),
                 velocity_weights=(10., 10., 10.), force_weights=(0.1, 0.1, 0.1)):
        self.dt, self.mass, self.damping = map(float, (dt, mass, damping))
        if not np.isfinite(self.mass) or self.mass <= 0:
            raise ValueError('mass must be finite and positive')
        if not np.isfinite(self.damping) or self.damping < 0:
            raise ValueError('damping must be finite and nonnegative')
        self.buoyancy_force = 0.0 if perfect else (997 * 0.03554577 - self.mass) * 9.81
        self.buoyancy = np.array([0., 0., self.buoyancy_force])
        self.A, self.B, self.d = self.matrices(self.dt)
        q_state = vector(state_weights, 6, 'state_weights')
        q_velocity = vector(velocity_weights, 3, 'velocity_weights')
        r_force = vector(force_weights, 3, 'force_weights')
        if any(np.any(weights <= 0) for weights in (q_state, q_velocity, r_force)):
            raise ValueError('DLQR weights must be positive')
        self.K_state, self.P_state, _ = control.dlqr(
            self.A, self.B, np.diag(q_state), np.diag(r_force), method='scipy')
        gain, _, _ = control.dlqr(
            self.A[:3, :3], self.B[:3], np.diag(q_velocity), np.diag(r_force),
            method='scipy')
        # Exclude position integrators from constant-velocity tracking.
        self.K = np.hstack((gain, np.zeros((3, 3))))
        self.N = gain + np.eye(3) * self.damping

    def matrices(self, dt):
        """Discretize linear drag exactly, including its zero-drag limit."""
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be finite and positive')
        rate_dt = self.damping * dt / self.mass
        decay = np.exp(-rate_dt)
        # Series avoids cancellation in the position integral near zero drag.
        if abs(rate_dt) < 1e-4:
            velocity_integral = dt * (
                1 - rate_dt / 2 + rate_dt**2 / 6 - rate_dt**3 / 24)
            position_integral = dt**2 * (
                0.5 - rate_dt / 6 + rate_dt**2 / 24 - rate_dt**3 / 120)
        else:
            velocity_integral = -np.expm1(-rate_dt) * dt / rate_dt
            position_integral = (dt - velocity_integral) * dt / rate_dt
        a = np.eye(6)
        a[:3, :3] *= decay
        a[3:, :3] = np.eye(3) * velocity_integral
        b = np.vstack((np.eye(3) * velocity_integral,
                       np.eye(3) * position_integral)) / self.mass
        return a, b, b @ self.buoyancy

    def predict(self, state, applied_force, dt=None):
        """Advance the nominal plant with held applied force, without estimation."""
        a, b, d = self.matrices(self.dt if dt is None else dt)
        return (a @ vector(state, 6, 'state')
                + b @ vector(applied_force, 3, 'applied_force') + d)

    def control_force(self, state, reference):
        """
        Track velocity (3 values) or full state (6 values).

        A moving full-state reference must advance its position externally by
        v_ref * dt. Feedforward then makes this ramp an exact nominal solution.
        """
        state = vector(state, 6, 'state')
        reference = np.asarray(reference, dtype=float)
        if reference.shape == (3,):
            reference = vector(reference, 3, 'reference')
            force = self.N @ reference - self.K @ state
        else:
            reference = vector(reference, 6, 'reference')
            force = self.K_state @ (reference - state) + self.damping * reference[:3]
        return force - self.buoyancy

    def damping_metrics(self):
        """Continuous-equivalent pole-pair metrics for nominal position control."""
        closed_loop = self.A - self.B @ self.K_state
        results = {}
        for axis, name in enumerate('xyz'):
            indices = [axis, axis + 3]
            z = np.linalg.eigvals(closed_loop[np.ix_(indices, indices)]).astype(complex)
            poles = np.log(z) / self.dt
            wn = float(np.sqrt(np.prod(poles).real))
            results[name] = {
                'discrete_poles': z, 'poles': poles, 'wn': wn,
                'zeta': float(-np.sum(poles).real / (2 * wn)),
                'dominant_tau': float(-1 / np.max(poles.real)),
            }
        return results
