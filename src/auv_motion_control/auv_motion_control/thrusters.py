"""
HoveringAUV body-frame thruster allocation (x forward, y left, z up).

Geometry comes from holooceanPacket/engine/Source/Holodeck/Agents/
Public/HoveringAUV.h and Private/HoveringAUV.cpp, not the unavailable
HoveringAUV.thruster_d/thruster_p Python attributes.
Thruster order: vertical front right, front left, rear left, rear right,
then angled thrusters in the same order. Values are signed forces in N,
not PWM or normalized motor speeds. Torques are in N m about the COM.
"""

import numpy as np

# Engine mesh positions: centimetres in Unreal's left-handed body frame.
_MESH_POSITIONS = np.array([
    [18.18, 22.14, -4.00], [18.18, -22.14, -4.00],
    [-31.43, -22.14, -4.00], [-31.43, 22.14, -4.00],
    [7.39, 18.23, -0.21], [7.39, -18.23, -0.21],
    [-20.64, -18.23, -0.21], [-20.64, 18.23, -0.21],
])
_DIAGONAL = 1 / np.sqrt(2)
thruster_d = np.array([
    [0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1],
    [_DIAGONAL, _DIAGONAL, 0], [_DIAGONAL, -_DIAGONAL, 0],
    [_DIAGONAL, _DIAGONAL, 0], [_DIAGONAL, -_DIAGONAL, 0],
])


def _mapping(perfect):
    if perfect:
        center = (_MESH_POSITIONS[0] + _MESH_POSITIONS[2]) / 2
        center[2] = _MESH_POSITIONS[7, 2]
    else:
        center = np.array([-5.9, 0.46, -2.82])
    # Convert to metres and reverse y for the HoloOcean client frame.
    positions = (_MESH_POSITIONS - center) * np.array([0.01, -0.01, 0.01])
    return positions, np.vstack((thruster_d.T, np.cross(positions, thruster_d).T))


# Engine default is Perfect=true. Use perfect=False for its asymmetric COM.
thruster_p, B_thrust = _mapping(True)
MAX_THRUST = 77.55  # AUV_MAX_LIN_ACCEL * mass / 4 in HoveringAUV.h.
_B_ASYMMETRIC = _mapping(False)[1]
_ALLOCATIONS = {True: np.linalg.pinv(B_thrust), False: np.linalg.pinv(_B_ASYMMETRIC)}


def force_to_thrust(force, *, perfect=True):
    """
    Allocate [Fx,Fy,Fz] or [Fx,Fy,Fz,Tx,Ty,Tz] to eight thrusters.

    A three-component input requests zero torque. Uses the minimum-norm
    pseudoinverse allocation; no saturation is applied. The engine clamps
    each thruster to +/-77.55 N, so excessive requests will not be achieved.
    """
    force = np.asarray(force, dtype=float)
    if force.shape == (3,):
        force = np.concatenate((force, np.zeros(3)))
    if force.shape != (6,) or not np.all(np.isfinite(force)):
        raise ValueError('force must contain three or six finite values')
    return _ALLOCATIONS[bool(perfect)] @ force


def thrust_to_force(thrust, *, perfect=True):
    """
    Return [Fx,Fy,Fz,Tx,Ty,Tz] for eight signed thruster forces.

    This is the geometric mapping without engine saturation. Supply the
    actual applied forces when checking what the simulator can achieve.
    """
    thrust = np.asarray(thrust, dtype=float)
    if thrust.shape != (8,) or not np.all(np.isfinite(thrust)):
        raise ValueError('thrust must contain eight finite values')
    matrix = B_thrust if perfect else _B_ASYMMETRIC
    return matrix @ thrust
