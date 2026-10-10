"""Check upward central-axis geometry and obstacle-aware yaw selection."""

import numpy as np
import pytest
from scipy.spatial import KDTree

from python_planner.use_cases.nbv import select_upward_view_axis, generate_cone_viewpoints


@pytest.mark.parametrize('vehicle', [[10., 0., 0.], [0., -10., 3.], [-5., 3., -2.]])
def test_central_axis_has_locked_45_degree_elevation(vehicle):
    target = np.zeros(3)
    axis = select_upward_view_axis(target, np.asarray(vehicle), KDTree([target]))
    assert np.linalg.norm(axis) == pytest.approx(1.)
    assert axis[2] == pytest.approx(1. / np.sqrt(2.))
    assert np.arctan2(axis[2], np.linalg.norm(axis[:2])) == pytest.approx(np.pi / 4.)
    preferred = np.asarray(vehicle[:2]) / np.linalg.norm(vehicle[:2])
    np.testing.assert_allclose(axis[:2] * np.sqrt(2.), preferred, atol=1e-12)


def test_obstructed_vehicle_facing_ray_rotates_toward_clearance():
    target = np.zeros(3)
    toward_vehicle = np.array([1., 0., 1.]) / np.sqrt(2.)
    blockers = np.linspace(.3, 1.5, 20)[:, None] * toward_vehicle
    axis = select_upward_view_axis(target, np.array([10., 0., 0.]),
                                   KDTree(np.vstack((target, blockers))))
    assert axis[0] < 0.
    assert axis[2] == pytest.approx(1. / np.sqrt(2.))


def test_obstacle_between_surface_and_endpoint_influences_yaw():
    target = np.zeros(3)
    blocked_axis = np.array([1., 0., 1.]) / np.sqrt(2.)
    # The endpoint itself is clear; obstruction lies along the departing ray.
    tree = KDTree([target, blocked_axis * .3])
    axis = select_upward_view_axis(target, np.array([10., 0., 0.]), tree)
    assert axis[0] < 0.


def test_cone_polar_offsets_and_candidate_count_are_unchanged():
    target = np.array([1., 2., -4.])
    axis = np.array([1., 0., 1.]) / np.sqrt(2.)
    candidates = generate_cone_viewpoints(target, axis, standoff=1.5)
    assert len(candidates) == 17
    rays = np.array([candidate['position'] - target for candidate in candidates]) / 1.5
    np.testing.assert_allclose(np.linalg.norm(rays, axis=1), 1.)
    angles = np.degrees(np.arccos(np.clip(rays @ axis, -1., 1.)))
    np.testing.assert_allclose(angles, [0.] + [15.] * 8 + [30.] * 8, atol=1e-6)
    np.testing.assert_allclose(candidates[0]['position'], target + 1.5 * axis)


@pytest.mark.parametrize('kwargs', [{'standoff': 0.}, {'azimuth_steps': 0}])
def test_invalid_axis_sampling_is_rejected(kwargs):
    with pytest.raises(ValueError, match='must be positive'):
        select_upward_view_axis(np.zeros(3), np.ones(3), KDTree([[0., 0., 0.]]), **kwargs)
