"""Run complete attempt/observation sequences so a valid path cannot hide a loop."""

import json
import math
from pathlib import Path

import pytest

from test_nbv_cache import query, replay  # noqa: F401

INCIDENT = json.loads((Path(__file__).parent / 'fixtures/repeat_target_20261016.json').read_text())
TARGET = INCIDENT['repeated_region'][0]
GOAL = [TARGET[0] + 3, TARGET[1], TARGET[2] + 1]
START = [GOAL[0] + 3, GOAL[1] + 2, GOAL[2]]


def candidate(target=TARGET, goal=GOAL, cluster_id=1):
    yaw = math.atan2(target[1] - goal[1], target[0] - goal[0])
    camera = [goal[0] + .35 * math.cos(yaw) - .3875 * math.sin(yaw),
              goal[1] + .35 * math.sin(yaw) + .3875 * math.cos(yaw), goal[2] - .5]
    return dict(position=camera, target=target, utility=1., cluster_id=cluster_id)


def plan(candidates=None, time=0):
    return dict(type='plan', use_map_collision=True,
                candidates=candidates or [candidate()], time=time)


def frame(time, position, yaw=0, value=1.):
    return [dict(type='mag', value=[math.cos(yaw), -math.sin(yaw), 0]),
            dict(type='odom', position=position, stamp=time),
            dict(type='depth', stamp=time, frame=dict(rows=1, cols=1, values=[value])),
            dict(type='tick', time=time)]


def mission(replay, events, **config):
    return query(replay, dict(op='mission', config=dict(voxel_resolution=.2,
                 max_depth=5., planning_period=100, **config),
                 events=[dict(type='odom', position=START, stamp=0.), dict(type='rgb'),
                         dict(type='candidates', values=[]), *events]))['result']


def filters(out, stage):
    return [e['details'] for e in out['events'] if e.get('stage') == stage]


def test_cancellation_and_expired_cooldowns_cannot_retry_region_forever(replay):
    events = []
    for i in range(6):
        c = candidate(target=INCIDENT['repeated_region'][i % 3], cluster_id=100 + i)
        events += [dict(type='expire', time=i * 31.), plan([c], time=i * 31.), dict(type='cancel')]
    out = mission(replay, events)
    assert len([p for p in out['published'] if p]) == 3
    assert [e['attempt'] for e in filters(out, 'inspection_attempt_started')] == [1, 2, 3]
    assert filters(out, 'inspection_target_suppressed')[-1]['reason'] == 'attempt_budget_exhausted'
    assert len(out['states'][-1]['state']['blacklisted_targets']) == 1
    permanent = [e for e in filters(out, 'blacklist_addition') if not e['expires']]
    assert len(permanent) == 1
    assert not out['completed']


@pytest.mark.parametrize('limit', [2, 3])
@pytest.mark.parametrize('cooldown', [0., 30.])
def test_repeated_failure_blacklist_survives_expiry_and_later_blacklisting(replay, limit, cooldown):
    events = []
    for i in range(limit):
        events += [plan(time=31. * i), dict(type='cancel'),
                   dict(type='expire', time=31. * (i + 1))]
    # Neither a later blacklist call nor passing time may demote the permanent ban.
    events += [dict(type='blacklist'), dict(type='expire', time=1_000_000.)]
    out = mission(replay, events, route_target_max_attempts=limit,
                  route_blacklist_cooldown_s=cooldown)
    assert out['states'][-1]['state']['blacklisted_targets'] == [TARGET]
    assert len([e for e in filters(out, 'blacklist_addition') if not e['expires']]) == 1
    assert len(filters(out, 'blacklist_expired')) == (limit - 1 if cooldown else 0)


def test_last_allowed_attempt_is_not_blacklisted_before_execution_fails(replay):
    events = []
    for i in range(2):
        events += [plan(time=31. * i), dict(type='cancel'),
                   dict(type='expire', time=31. * (i + 1))]
    events += [plan(time=62.), plan(time=63.)]
    out = mission(replay, events)
    assert not out['states'][-1]['state']['blacklisted_targets']
    assert out['states'][-1]['state']['active_trajectory']
    assert [e['attempt'] for e in filters(out, 'inspection_attempt_started')] == [1, 2, 3]


def test_replanning_a_moving_loop_does_not_reset_attempt_deadline(replay):
    events = [dict(type='candidates', values=[candidate()]), plan()]
    for t in range(20, 141, 20):
        # Moving by >.15 m defeats the old displacement-only stall check.
        pos = [START[0], START[1] + (.5 if t % 40 else 0), START[2]]
        events += [dict(type='request'), *frame(float(t), pos)]
    out = mission(replay, events)
    assert len(filters(out, 'inspection_attempt_started')) == 1
    assert filters(out, 'inspection_attempt_timeout')[0]['elapsed_s'] == 120.
    assert not out['states'][-1]['state']['active_trajectory']
    assert not out['completed']  # temporary exclusion is not exploration completion


def test_impossible_goal_checks_have_a_budget_without_any_publication(replay):
    # A search timeout is still bounded, but does not prove all view rays collide.
    events = []
    for i in range(7):
        events += [dict(type='expire', time=31. * i), plan(time=31. * i)]
    out = mission(replay, events, astar_max_steps=1)
    assert not out['published']
    assert [e['attempt'] for e in filters(out, 'inspection_plan_failed')] == [1, 2, 3]
    assert out['states'][-1]['state']['_plan_failure_reason'] == 'target_attempts_exhausted'
    assert out['states'][-1]['state']['blacklisted_targets'] == [TARGET]
    assert any(not e['expires'] for e in filters(out, 'blacklist_addition'))
    assert all(e['reason'] == 'attempt_budget_exhausted' and e['radius'] == .5
               for e in filters(out, 'blacklist_addition'))


def test_failed_preferred_target_is_bounded_even_when_an_alternative_publishes(replay):
    # The nearby alternative fits the search budget; the preferred route does not.
    other = [START[0] + 3, START[1], START[2] - 1]
    other_goal = [START[0] + .2, START[1], START[2]]
    alternative = candidate(other, other_goal)
    alternative['utility'] = .5
    events = []
    events += [plan([candidate(), alternative], time=float(i)) for i in range(6)]
    out = mission(replay, events, astar_max_steps=3)
    assert [e['attempt'] for e in filters(out, 'inspection_plan_failed')] == [1, 2, 3]
    assert len(filters(out, 'inspection_attempt_started')) == 1
    assert out['states'][-1]['result'] is True
    assert filters(out, 'inspection_target_suppressed')


def test_extended_view_is_observed_once_from_actual_depth_pose(replay):
    record = INCIDENT['extended_observation']
    target, goal = record['target'], record['vehicle_goal']
    assert math.dist(target, goal) > 3.  # previous history cutoff missed this visit
    c = candidate(target, goal)
    yaw = math.atan2(target[1] - goal[1], target[0] - goal[0])
    out = mission(replay, [plan([c]), *frame(1., goal, yaw), plan([c], time=2.)])
    assert len(filters(out, 'inspection_target_observed')) == 1
    assert out['states'][-1]['state']['visited_targets'] == [target]
    assert out['states'][-1]['result'] is False
    assert len([p for p in out['published'] if p]) == 1
    assert filters(out, 'inspection_target_suppressed')[-1]['reason'] == 'observed_at_resolved_viewpoint'


def test_reaching_pose_without_facing_target_does_not_mark_observed(replay):
    yaw = math.atan2(TARGET[1] - GOAL[1], TARGET[0] - GOAL[0]) + math.pi
    out = mission(replay, [plan(), *frame(1., GOAL, yaw)])
    assert not filters(out, 'inspection_target_observed')
    assert not out['states'][-1]['state']['visited_targets']


def test_invalid_depth_at_goal_is_not_an_observation(replay):
    yaw = math.atan2(TARGET[1] - GOAL[1], TARGET[0] - GOAL[0])
    out = mission(replay, [plan(), *frame(1., GOAL, yaw, value=None)])
    assert not filters(out, 'inspection_target_observed')
    assert not out['states'][-1]['state']['visited_targets']


def test_only_one_viewpoint_is_executed_for_repeated_cluster(replay):
    c2 = candidate(goal=[GOAL[0], GOAL[1] + 1., GOAL[2]], cluster_id=2)
    c2['utility'] = .5
    out = mission(replay, [plan([candidate(), c2])])
    end = out['states'][-1]['state']['active_trajectory'][-1][:3]
    assert math.dist(end, GOAL) < .001
    assert math.dist(end, c2['position']) > .5
    # A new execution starts at the first actionable waypoint, not its pose anchor.
    wire = out['published'][-1]
    first = [wire[0] - .35, wire[1] - .3875, wire[2] + .5]
    assert math.dist(first, START) > .01


def test_exhausted_target_does_not_block_another_target(replay):
    events = []
    for i in range(3):
        events += [plan(time=31. * i), dict(type='cancel'), dict(type='expire', time=31. * (i + 1))]
    other = [TARGET[0], TARGET[1] + 4, TARGET[2]]
    other_goal = [GOAL[0], GOAL[1] + 4, GOAL[2]]
    events += [plan([candidate(), candidate(other, other_goal)], time=100.)]
    out = mission(replay, events)
    assert out['states'][-1]['result'] is True
    assert all(t == other for t in out['states'][-1]['state']['selected_targets'])


def test_occluded_observation_is_not_marked_complete(replay):
    c = candidate()
    occluder = [(a + b) * .5 for a, b in zip(c['position'], TARGET)]
    yaw = math.atan2(TARGET[1] - GOAL[1], TARGET[0] - GOAL[0])
    out = mission(replay, [plan(), dict(type='map', points=[occluder]), *frame(1., GOAL, yaw)])
    assert not filters(out, 'inspection_target_observed')
    assert not out['states'][-1]['state']['visited_targets']


def test_exhausted_candidates_hold_instead_of_finishing_mission(replay):
    events = []
    for i in range(3):
        events += [plan(time=i * 31.), dict(type='cancel')]
    events += [dict(type='expire', time=100.), dict(type='candidates', values=[candidate()])]
    for t in (101., 102., 103., 104.):
        events += [dict(type='request'), *frame(t, START)]
    out = mission(replay, events)
    assert not out['completed']
    assert not out['states'][-1]['state']['active_trajectory']
    assert len(filters(out, 'inspection_waiting')) >= 3
    bans = [b for e in filters(out, 'candidate_blacklist') for b in e['matching_bans']]
    assert bans and all(b['expires'] is False and b['remaining_s'] is None for b in bans)


@pytest.mark.parametrize('radius', [.2, 1.2])
def test_configured_retry_radius_controls_matching_cooldown_and_permanent_region(replay, radius):
    def shifted(distance):
        target = [TARGET[0], TARGET[1] + distance, TARGET[2]]
        goal = [GOAL[0], GOAL[1] + distance, GOAL[2]]
        return candidate(target=target, goal=goal)

    inside, outside = shifted(.75 * radius), shifted(1.1 * radius)
    out = mission(replay, [plan(), dict(type='cancel'), dict(type='expire', time=31.),
                  plan([inside], time=31.), dict(type='cancel'), dict(type='expire', time=62.),
                  plan([inside, outside], time=62.)],
                  route_blacklist_radius_m=radius, route_target_max_attempts=2)
    bans = filters(out, 'blacklist_addition')
    assert len(bans) == 2 and [b['radius'] for b in bans] == [radius, radius]
    assert [b['expires'] for b in bans] == [True, False]
    attempts = filters(out, 'inspection_attempt_started')
    assert [a['attempt'] for a in attempts] == [1, 2, 1]
    assert all(a['matching_radius_m'] == radius for a in attempts)
    suppression = filters(out, 'inspection_target_suppressed')[-1]
    assert suppression['radius'] == radius and suppression['exclusion_center'] == TARGET
    assert out['states'][-1]['result'] is True
    assert all(t == outside['target'] for t in out['states'][-1]['state']['selected_targets'])


@pytest.mark.parametrize('field', ['route_blacklist_radius_m', 'route_collision_blacklist_radius_m'])
@pytest.mark.parametrize('value', [0., -1., None, 'wide', False])
def test_invalid_blacklist_radii_are_rejected(replay, field, value):
    replay.stdin.write(json.dumps(dict(op='mission', config={field: value}, events=[])) + '\n')
    replay.stdin.flush()
    while True:
        line = replay.stdout.readline()
        assert line
        if line.startswith('{'):
            response = json.loads(line)
            break
    assert field in response['error'] and 'finite and positive' in response['error']
