"""Running floor statistics and the per-surface-point exponential gain penalty."""

import numpy as np
import pytest

from auv_planner_reference.domain.seabed import SeabedEstimate
from auv_planner_reference.use_cases.nbv import (
    seabed_gain_weights, evaluate_viewpoint, evaluate_viewpoints,
)


def plane(z, offset_x=0., size=10):
    x, y = np.meshgrid(np.arange(size) * .2 + .1 + offset_x,
                       np.arange(size) * .2 + .1)
    return np.column_stack((x.ravel(), y.ravel(), np.full(x.size, z)))


@pytest.mark.parametrize('z', [-500.1, -30.1, 12.1, 210.1])
def test_floor_height_is_automatically_found_at_arbitrary_world_height(z):
    estimate = SeabedEstimate(.2)
    points = np.vstack((plane(z), plane(z + 5., offset_x=5., size=20), [[0., 0., z - 50.]]))
    estimate.update(points)
    assert estimate.mean_z == pytest.approx(z)
    assert estimate.sample_count == 100


def test_incremental_sums_and_counts_match_changed_map_without_double_counting():
    estimate = SeabedEstimate(.2)
    first = plane(-30.1)
    assert estimate.update(first) == 100
    assert estimate.update(np.vstack((first, first))) == 0
    assert estimate.sample_count == 100
    second = plane(-30.12, offset_x=2.)
    estimate.update(np.vstack((first, second)))
    assert estimate.sample_count == 200
    assert estimate.mean_z == pytest.approx(-30.11)
    estimate.update(second)
    assert estimate.sample_count == 100
    assert estimate.mean_z == pytest.approx(-30.12)


def test_no_supported_floor_has_no_fixed_height_fallback():
    estimate = SeabedEstimate(.2)
    assert estimate.update(np.empty((0, 3))) == 0
    estimate.update([[0., 0., float('nan')], [0., 0., float('-inf')], [0., 0., -299.]])
    assert estimate.mean_z is None and estimate.sample_count == 0
    estimate.update(plane(-30.1))
    estimate.update(np.empty((0, 3)))
    assert estimate.mean_z is None and estimate.sample_count == 0


def test_vertical_wall_and_narrow_pipe_do_not_supply_broad_floor_support():
    x, z = np.meshgrid(np.arange(30) * .2, np.arange(30) * .2)
    wall = np.column_stack((x.ravel(), np.zeros(x.size), z.ravel()))
    estimate = SeabedEstimate(.2)
    estimate.update(wall)
    assert estimate.mean_z is None
    estimate.update(np.column_stack((np.arange(100) * .2, np.zeros(100), np.full(100, -10.))))
    assert estimate.mean_z is None


def test_floor_penalty_decays_exponentially_with_surface_height():
    heights = np.array([-1., 0., .5, 1., 2.])
    weights = seabed_gain_weights(-300. + heights, -300., .5, .95)
    np.testing.assert_allclose(weights, [0.05, 0.05, 1-.95/np.e,
                                       1-.95*np.exp(-2), 1-.95*np.exp(-4)])
    assert np.all(np.diff(weights) >= 0.)


def test_seabed_penalty_preserves_incidence_and_complexity_math():
    heights = np.array([0., .5, 1.])
    points = np.column_stack((np.zeros(3), np.zeros(3), -300. + heights))
    normals = np.tile([0., 0., 1.], (3, 1))
    complexity = np.array([0., .1, .2])
    viewpoints = np.array([[0., 0., -297.], [1., 0., -297.]])
    kwargs = dict(seabed_z=-300., seabed_decay_height=.5, seabed_penalty_strength=.95)
    batch = evaluate_viewpoints(viewpoints, points, normals, complexity, **kwargs)
    scalar = [evaluate_viewpoint(viewpoint, points, normals, complexity, **kwargs)
              for viewpoint in viewpoints]
    np.testing.assert_allclose(batch, scalar)
    expected = np.sum((1. + 15. * complexity) * seabed_gain_weights(points[:, 2], -300.))
    assert batch[0] == pytest.approx(expected)
    original = evaluate_viewpoints(viewpoints, points, normals, complexity)
    assert np.all(batch < original)
    # Height of the camera does not change the floor penalty itself.
    assert evaluate_viewpoint([0., 0., -290.], points, normals, complexity, **kwargs) == pytest.approx(expected)


def test_disabled_or_zero_strength_leaves_gain_unchanged():
    np.testing.assert_array_equal(seabed_gain_weights([-300., -299.], None), [1., 1.])
    np.testing.assert_array_equal(seabed_gain_weights([-300., -299.], -300., strength=0.), [1., 1.])


@pytest.mark.parametrize('kwargs', [{'decay_height': 0.}, {'strength': 1.1},
                                  {'strength': -1.}, {'decay_height': float('nan')}])
def test_invalid_gain_parameters_are_rejected(kwargs):
    with pytest.raises(ValueError):
        seabed_gain_weights([-300.], -300., **kwargs)
