"""Utility must survive NBV filtering and dominate reachable visit order."""

import numpy as np
import pytest

from python_planner.route_planning import plan_multi_waypoint_route, path_is_collision_free


def candidate(x):
    return np.array([x, .5, .5]), np.array([x, 2.5, .5])


def visit_order(targets):
    changes = np.r_[True, np.any(np.diff(targets, axis=0) != 0, axis=1)]
    return targets[changes, 0].tolist()


def test_far_high_utility_view_precedes_near_low_utility_view():
    start = np.array([.5, .5, .5])
    candidates = [candidate(1.5), candidate(6.5), candidate(3.5)]
    route, targets = plan_multi_waypoint_route(
        start, candidates, set(), 1., candidate_utilities=[.1, .9, .5])
    assert visit_order(targets) == [6.5, 3.5, 1.5]
    np.testing.assert_array_equal(route[0], start)
    assert path_is_collision_free(route, set(), 1.)


def test_utility_priority_overrides_reversal_penalty():
    route, targets = plan_multi_waypoint_route(
        np.array([.5, .5, .5]), [candidate(2.5), candidate(-1.5)], set(), 1.,
        current_velocity=np.array([1., 0., 0.]), reversal_weight=100.,
        candidate_utilities=[.2, .8])
    assert visit_order(targets) == [-1.5, 2.5]
    assert path_is_collision_free(route, set(), 1.)


def test_equal_utilities_use_distance_from_each_successful_endpoint():
    _, targets = plan_multi_waypoint_route(
        np.array([.5, .5, .5]), [candidate(5.5), candidate(2.5), candidate(-.5)],
        set(), 1., reversal_weight=0., candidate_utilities=[.4, .4, .4])
    assert visit_order(targets) == [-.5, 2.5, 5.5]


def test_equal_utilities_retain_forward_motion_preference():
    _, targets = plan_multi_waypoint_route(
        np.array([.5, .5, .5]), [candidate(-.5), candidate(2.5)], set(), 1.,
        current_velocity=np.array([1., 0., 0.]), reversal_weight=2.,
        candidate_utilities=[.4, .4])
    assert visit_order(targets) == [2.5, -.5]


def test_unreachable_high_utility_view_does_not_shift_later_ordering():
    occupied = {(100, 0, 0)}
    route, targets = plan_multi_waypoint_route(
        np.array([.5, .5, .5]), [candidate(100.5), candidate(2.5), candidate(-.5)],
        occupied, 1., reversal_weight=0., candidate_utilities=[.9, .3, .3])
    assert visit_order(targets) == [-.5, 2.5]
    assert path_is_collision_free(route, occupied, 1.)


def test_all_blocked_returns_consistent_empty_arrays():
    route, targets = plan_multi_waypoint_route(
        np.array([.5, .5, .5]), [candidate(2.5)], {(2, 0, 0)}, 1.,
        candidate_utilities=[.8])
    assert route.shape == targets.shape == (0, 3)


@pytest.mark.parametrize('utilities', [[], [.1, .2], [np.nan], [np.inf], [[.3]]])
def test_invalid_utility_metadata_is_rejected(utilities):
    with pytest.raises(ValueError, match='one finite value per candidate'):
        plan_multi_waypoint_route(np.array([.5, .5, .5]), [candidate(2.5)], set(), 1.,
                                 candidate_utilities=utilities)
