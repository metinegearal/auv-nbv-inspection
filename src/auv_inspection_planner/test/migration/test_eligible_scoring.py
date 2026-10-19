"""Excluded targets must not set the scale or hide eligible camera views."""

import collections
import json
from pathlib import Path

import pytest

from test_nbv_cache import query, replay, sequence, step  # noqa: F401
from test_goal_standoff import shell_map

INCIDENT = json.loads((Path(__file__).parent / 'fixtures/score_starvation_20261016.json').read_text())


def candidate(target, camera, gain, cost, cluster_id):
    return dict(target=target, position=camera, raw_gain=gain, raw_cost=cost,
                utility=-10., cluster_id=cluster_id)


HIGH = candidate([15., 10., 9.], [13., 10., 10.], 1000., 1., 1)
GOOD = candidate([15., 14., 9.], [13., 14., 10.], 20., 10., 2)
LOW = candidate([15., 18., 9.], [13., 18., 10.], 1., 30., 3)


def plan(candidates, time=0.):
    return dict(type='plan', candidates=candidates, use_map_collision=True, time=time)


def mission(replay, events, position=None, **config):
    return query(replay, dict(op='mission', config=dict(voxel_resolution=.2, max_depth=5.,
                 nbv_utility_gain_weight=1.5, nbv_utility_cost_weight=1.5,
                 min_utility_threshold=.1, **config), events=[
                 dict(type='odom', position=position or [10., 10., 10.], stamp=0.), *events]))['result']


def details(out, stage):
    return [e['details'] for e in out['events'] if e.get('stage') == stage]


def test_recorded_exhausted_high_gain_target_cannot_starve_remaining_views(replay):
    incident = INCIDENT
    excluded = incident['observed'] + [a['target'] for a in incident['attempts']]
    remaining = [c for c in incident['candidates'] if c['target'] not in excluded]
    assert len(remaining) == incident['historical_remaining'] == 97
    assert sum(c['utility'] >= .1 for c in remaining) == incident['historical_above_threshold'] == 0
    out = mission(replay, [dict(type='target_history', observed=incident['observed'],
                   attempts=incident['attempts']), plan(incident['candidates'])],
                   position=incident['vehicle_position'])
    scores = details(out, 'candidate_minimum_utility')
    assert sum(c['utility'] >= .1 for c in scores) == incident['eligible_renormalized_above_threshold'] == 18
    normalization = details(out, 'utility_normalization')[0]
    assert normalization['gain_max'] == pytest.approx(21.74098160963604)
    assert normalization['raw_candidates'] == 97
    assert out['states'][-1]['result'] is True and out['published'][-1]
    assert not any(t in excluded for t in out['states'][-1]['state']['selected_targets'])
    assert not out['completed']  # Controlled empty map establishes scoring, not historical flight.


@pytest.mark.parametrize('exclusion', ['observed', 'exhausted', 'permanent', 'cooldown'])
def test_every_eligibility_exclusion_precedes_normalization(replay, exclusion):
    history = dict(type='target_history')
    if exclusion == 'observed':
        history['observed'] = [HIGH['target']]
    elif exclusion == 'exhausted':
        history['attempts'] = [dict(target=HIGH['target'], count=3)]
    else:
        ban = dict(target=HIGH['target'], radius=1. if exclusion == 'permanent' else .5)
        if exclusion == 'cooldown':
            ban['expires_at'] = 30.
        history['blacklist'] = [ban]
    out = mission(replay, [history, plan([HIGH, GOOD, LOW])])
    score = details(out, 'utility_normalization')[0]
    assert score['gain_max'] == 20. and score['cost_min'] == 10.
    assert out['states'][-1]['result'] is True
    assert all(t == GOOD['target'] for t in out['states'][-1]['state']['selected_targets'])


def test_expired_cooldown_rejoins_the_scale_only_after_expiry(replay):
    out = mission(replay, [dict(type='target_history', blacklist=[
                  dict(target=HIGH['target'], radius=.5, expires_at=30.)]),
                  plan([HIGH, GOOD, LOW]), plan([HIGH, GOOD, LOW], time=31.)])
    assert [s['gain_max'] for s in details(out, 'utility_normalization')] == [20., 1000.]
    assert len(details(out, 'blacklist_expired')) == 1
    assert all(t == HIGH['target'] for t in out['states'][-1]['state']['selected_targets'])


@pytest.mark.parametrize('pool', [[GOOD], [GOOD, dict(LOW, raw_gain=20.)],
                                [GOOD, dict(LOW, raw_cost=10.)]])
def test_single_or_equal_score_pools_retain_positive_information(replay, pool):
    out = mission(replay, [plan(pool)])
    assert out['states'][-1]['result'] is True
    assert max(s['utility'] for s in details(out, 'candidate_minimum_utility')) == 1.5


def test_zero_information_is_not_promoted_by_equal_gain_handling(replay):
    out = mission(replay, [plan([dict(GOOD, raw_gain=0.)])])
    assert not out['published']
    assert details(out, 'candidate_minimum_utility')[0]['utility'] == 0.


def collision_pool():
    blocked = candidate([.1, .1, .1], [.75, .1, .1], 1000., 1., 1)
    near = candidate([.8, .1, .1], [.75, .1, .1], 500., 1., 4)
    good = candidate([5.5, 4.1, .1], [3.5, 4.1, .1], 20., 10., 2)
    low = candidate([5.5, 8.1, .1], [3.5, 8.1, .1], 1., 30., 3)
    return [blocked, near, good, low]


def test_new_collision_blacklist_rescores_and_publishes_in_same_cycle(replay):
    pool = collision_pool()
    out = mission(replay, [shell_map(), plan(pool)], position=[3.1, .1, .6])
    assert out['states'][-1]['result'] is True
    assert all(t == pool[2]['target'] for t in out['states'][-1]['state']['selected_targets'])
    scales = details(out, 'utility_normalization')
    assert [s['gain_max'] for s in scales] == [1000., 20.]
    assert [s['pass'] for s in scales] == [0, 1]
    assert len(details(out, 'nbv_collision_rays')) == 1
    assert details(out, 'blacklist_addition')[0]['radius'] == 1.


def test_rescoring_does_not_reset_route_search_budget(replay):
    pool = collision_pool()
    out = mission(replay, [shell_map(), plan(pool), plan(pool, time=1.)],
                  position=[3.1, .1, .6], route_max_candidates=1)
    assert out['states'][-2]['result'] is False
    assert out['states'][-1]['result'] is True
    assert len(details(out, 'nbv_collision_rays')) == 1


def test_raw_views_are_retained_until_mission_eligibility_is_known(replay):
    normal, deferred = sequence(replay, step([.3, .2, 2.]),
                         step([.3, .2, 2.], defer_utility_normalization=True))
    counts = collections.Counter(c['cluster_id'] for c in deferred['candidates'])
    assert max(counts.values()) > 3
    assert len(deferred['candidates']) > len(normal['candidates'])
    assert all(c['raw_gain'] > 0 and c['utility'] == 0 for c in deferred['candidates'])


def test_cluster_shortlist_is_chosen_after_rescoring_all_eligible_views(replay):
    views = [candidate(GOOD['target'], [13., 14. + i * .1, 10.], gain, cost, 2)
             for i, (gain, cost) in enumerate([(20., 30.), (18., 28.), (10., 10.), (8., 9.)])]
    anchor = dict(LOW, raw_cost=50.)
    out = mission(replay, [dict(type='target_history', observed=[HIGH['target']]),
                  plan([HIGH, *views, anchor])])
    scores = [s for s in details(out, 'candidate_minimum_utility') if s['cluster_id'] == 2]
    expected = sorted(views, key=lambda c: (c['raw_gain'] - 1.) / 19. -
                      (c['raw_cost'] - 9.) / 41., reverse=True)[:3]
    assert [s['position'] for s in scores] == [c['position'] for c in expected]
    assert details(out, 'utility_normalization')[0]['raw_candidates'] == 5
