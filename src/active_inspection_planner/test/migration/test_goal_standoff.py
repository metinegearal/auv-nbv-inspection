"""Blocked NBV endpoints must get a safe, reachable viewing alternative."""

import json
import math
import os

import pytest

from test_nbv_cache import query, replay  # noqa: F401


def run(replay, **case):
    return query(replay, case)['result']


def wall():
    return [[0, y, z] for y in range(-12, 13) for z in range(-12, 13)]


def view(replay, raw=None, **changes):
    raw = wall() if raw is None else raw
    occupied = run(replay, op='clearance_inflate', occupied=raw, radius=.8, resolution=.2)
    case = dict(op='view_standoff', start=[3.1, .1, .1], goal=[.75, .1, .1],
                target=[.1, .1, .1], resolution=.2, occupied=occupied,
                raw_occupied=raw, max_extension=1.5, max_range=5.)
    case.update(changes)
    return run(replay, **case), occupied


def test_blocked_viewpoint_moves_outward_and_keeps_full_clearance(replay):
    result, occupied = view(replay)
    assert result['reason'] == 'success_adjusted_standoff'
    assert result['collision_free']
    end = result['path'][-1]
    assert end[0] > 1.2 and end[1:] == pytest.approx([.1, .1])
    assert math.dist(end, [.75, .1, .1]) <= 1.5
    assert [math.floor(x / .2) for x in end] not in occupied


def test_astar_routes_around_intervening_obstacle(replay):
    raw = wall() + [[20, y, z] for y in range(-1, 2) for z in range(-1, 2)]
    result, _ = view(replay, raw=raw, start=[6.1, .1, .1])
    assert result['reason'] == 'success_adjusted_standoff'
    assert result['collision_free']
    assert any(abs(p[1] - .1) > .8 or abs(p[2] - .1) > .8 for p in result['path'])


@pytest.mark.parametrize('changes,reason', [
    (dict(max_extension=0.), 'goal_occupied'),
    (dict(max_extension=.2), 'no_safe_goal_viewpoint'),
    (dict(max_range=.95), 'no_safe_goal_viewpoint'),
    (dict(start=[.1, .1, .1]), 'start_occupied'),
    (dict(max_steps=1), 'search_limit_reached'),
])
def test_limits_and_blocked_start_are_respected(replay, changes, reason):
    result, _ = view(replay, **changes)
    assert not result['path']
    assert result['reason'] == reason
    if 'max_steps' in changes:
        assert result['expansions'] <= changes['max_steps']


def test_clear_destination_behind_occluder_is_not_an_observation_goal(replay):
    raw = wall() + [[2, y, z] for y in range(-12, 13) for z in range(-12, 13)]
    result, _ = view(replay, raw=raw)
    assert not result['path']
    assert result['reason'] == 'no_safe_goal_viewpoint'


def test_clear_original_goal_preserves_exact_astar_result(replay):
    result, occupied = view(replay, goal=[1.65, .1, .1])
    exact = run(replay, op='astar', start=[3.1, .1, .1], goal=[1.65, .1, .1],
                occupied=occupied, resolution=.2)
    assert result['path'] == exact['path']
    assert result['reason'] == exact['reason'] == 'success'


def test_free_original_goal_must_also_see_the_target(replay):
    raw = wall() + [[2, y, z] for y in range(-12, 13) for z in range(-12, 13)]
    result, occupied = view(replay, raw=raw, goal=[1.95, .1, .1])
    assert [9, 0, 0] not in occupied
    assert not result['path']
    assert result['reason'] == 'goal_view_occluded'


def test_mission_publishes_adjusted_goal_without_blacklisting_target(replay):
    target = [.1, .1, .1]
    result = run(replay, op='mission', config=dict(voxel_resolution=.2,
                 collision_radius=.4, collision_tracking_margin=.4, max_depth=5.),
                 events=[dict(type='odom', position=[3.1, .1, .6], stamp=1.),
                         dict(type='map', points=[[(a + .5) * .2 for a in k] for k in wall()]),
                         dict(type='plan', use_map_collision=True,
                              candidates=[dict(position=[.35, .1, .1], target=target, utility=1.)])])
    assert result['states'][-1]['result'] is True
    assert result['published'] and result['published'][-1]
    state = result['states'][-1]['state']
    assert not state['blacklisted_targets']
    assert all(t == target for t in state['selected_targets'])
    adjusted = [e for e in result['events'] if e.get('stage') == 'nbv_goal_standoff']
    assert len(adjusted) == 1
    assert adjusted[0]['details']['reason'] == 'success_adjusted_standoff'
    assert state['active_trajectory'][-1][0] > 1.2


def test_recorded_distant_nbvs_publish_route_on_saved_map(replay):
    snapshot = os.environ.get('PLANNER_GOAL_MAP')
    iterations = os.environ.get('PLANNER_GOAL_ITERATIONS')
    if not snapshot or not iterations:
        pytest.skip('set PLANNER_GOAL_MAP and PLANNER_GOAL_ITERATIONS for October 16 13:00 run')
    with open(iterations) as source:
        row = next(r for r in map(json.loads, source) if r['iteration'] == 1200)
    eligible = [c for c in row['candidates'] if c['utility'] >= .1]
    assert len(eligible) == 3
    result = run(replay, op='mission', real_map=True, load=snapshot,
                 config=dict(voxel_resolution=.2, collision_radius=.4,
                             collision_tracking_margin=.4, max_depth=5., min_utility_threshold=.1),
                 events=[dict(type='odom', position=row['vehicle_position'], stamp=1.),
                         dict(type='plan', use_map_collision=True, candidates=eligible)])
    assert result['states'][-1]['result'] is True
    assert result['published'][-1]
    state = result['states'][-1]['state']
    assert math.dist(state['active_trajectory'][-1][:3], row['vehicle_position']) > 10.
    adjustments = [e['details'] for e in result['events'] if e.get('stage') == 'nbv_goal_standoff']
    successful = [e for e in adjustments if e['resolved_vehicle_goal'] is not None]
    assert len(successful) == 1
    selected = successful[0]
    assert selected['reason'] in ('success_adjusted_standoff', 'success_surrounding_view')
    # Surrounding views may now resolve either of the formerly occluded rays.
    # Independently reproduce its endpoint and full-map route clearance.
    checked = run(replay, op='view_surround', load=snapshot, clearance=.8, resolution=.2,
                  start=row['vehicle_position'], goal=selected['original_vehicle_goal'],
                  target=selected['target'], offset=[.35, .3875, -.5],
                  max_extension=1.5, max_range=5.)
    assert checked['collision_free'] and checked['path']
    assert checked['path'][-1] == pytest.approx(state['active_trajectory'][-1][:3])
    for target in state['blacklisted_targets']:
        assert any(e['target'] == target and e['reason'] == 'all_nbv_rays_collide'
                   for e in adjustments)


def shell(n=4, open_top=False):
    return [[x, y, z] for x in range(-n, n + 1) for y in range(-n, n + 1)
            for z in range(-n, n + 1)
            if max(abs(x), abs(y), abs(z)) == n and not (open_top and z == n)]


def surround(replay, raw, clearance=.8, **changes):
    occupied = run(replay, op='clearance_inflate', occupied=raw, radius=clearance, resolution=.2)
    case = dict(op='view_surround', start=[3.1, .1, .1], goal=[.75, .1, .1],
                target=[.1, .1, .1], resolution=.2, occupied=occupied,
                raw_occupied=raw, max_extension=1.5, max_range=5.)
    case.update(changes)
    return run(replay, **case)


def test_collision_sweeps_surrounding_views_instead_of_only_extending_one_ray(replay):
    raw = wall() + [[2, y, z] for y in range(-12, 13) for z in range(-12, 13)]
    result = surround(replay, raw)
    assert result['reason'] == 'success_surrounding_view'
    assert result['collision_free'] and result['path']
    assert result['directions_checked'] == 267
    assert result['clear_directions'] > 0


def test_surrounding_sweep_includes_vertical_escape(replay):
    result = surround(replay, shell(open_top=True), clearance=.2)
    assert result['reason'] == 'success_surrounding_view'
    assert result['collision_free']
    assert result['clear_directions'] > 0
    assert max(p[2] for p in result['path']) > 1.1


def test_enclosed_target_reports_all_directions_blocked(replay):
    result = surround(replay, shell())
    assert result['reason'] == 'all_nbv_rays_collide'
    assert result['directions_checked'] == 267
    assert result['clear_directions'] == 0
    assert not result['path']


def test_clear_rays_with_exhausted_astar_budget_are_not_all_colliding(replay):
    occupied = shell(n=5) + [[3, 0, 0]]
    result = surround(replay, [], occupied=occupied, max_extension=0., max_steps=20)
    assert not result['path']
    assert result['reason'] == 'search_limit_reached'
    assert result['directions_checked'] == 267
    assert result['clear_directions'] > 0
    assert result['expansions'] <= 20


def collision_mission(replay, events, **changes):
    config = dict(voxel_resolution=.2, max_depth=5., collision_radius=.4, collision_tracking_margin=.4)
    config.update(changes)
    return run(replay, op='mission', config=config, events=[
               dict(type='odom', position=[3.1, .1, .6], stamp=0.), dict(type='rgb'), *events])


def collision_plan(target=None, camera=None, **changes):
    c = dict(position=camera or [.75, .1, .1], target=target or [.1, .1, .1], utility=1.)
    c.update(changes)
    return dict(type='plan', use_map_collision=True, candidates=[c])


def shell_map():
    return dict(type='map', points=[[(v + .5) * .2 for v in cell] for cell in shell()])


def details(out, stage):
    return [e['details'] for e in out['events'] if e.get('stage') == stage]


@pytest.mark.parametrize('delta', [[1., 0, 0], [.6, .8, 0], [0, 0, 1.], [.7, 0, 0]])
def test_all_colliding_rays_blacklist_one_metre_and_keep_outside_candidate(replay, delta):
    target = [.1, .1, .1]
    nearby = [a + b for a, b in zip(target, delta)]
    outside = [1.11, .1, .1]
    out = collision_mission(replay, [shell_map(), collision_plan(),
             dict(type='expire', time=1_000_000.),
             dict(type='plan', use_map_collision=True, candidates=[
                  collision_plan(nearby, cluster_id=99)['candidates'][0],
                  collision_plan(outside, [3.5, .1, .1])['candidates'][0]])])
    assert out['states'][-1]['result'] is True
    assert all(t == outside for t in out['states'][-1]['state']['selected_targets'])
    assert out['states'][-1]['state']['blacklisted_targets'] == [target]
    bans = details(out, 'blacklist_addition')
    assert len(bans) == 1 and bans[0]['reason'] == 'all_nbv_rays_collide'
    assert bans[0]['radius'] == 1. and bans[0]['expires'] is False
    assert details(out, 'inspection_target_suppressed')[-1]['reason'] == 'collision_blacklist'


def test_neighbour_is_blacklisted_in_the_same_planning_pass(replay):
    near = collision_plan([.8, .1, .1], cluster_id=88, utility=.5)['candidates'][0]
    out = collision_mission(replay, [shell_map(), dict(type='plan', use_map_collision=True,
                         candidates=[collision_plan()['candidates'][0], near])])
    assert not out['published']
    assert len(details(out, 'nbv_collision_rays')) == 1
    assert len(details(out, 'blacklist_addition')) == 1


def test_collision_on_active_route_checks_all_rays_then_blacklists(replay):
    out = collision_mission(replay, [collision_plan(), shell_map(),
             dict(type='candidates', values=collision_plan()['candidates']),
             dict(type='depth', stamp=1., frame=dict(rows=1, cols=1, values=[None])),
             dict(type='tick', time=1.)])
    assert out['published'][0] and out['published'][1] == []
    assert not out['states'][-1]['state']['active_trajectory']
    assert details(out, 'nbv_collision_rays')[0]['reason'] == 'all_nbv_rays_collide'
    assert details(out, 'blacklist_addition')[0]['radius'] == 1.
    assert details(out, 'candidate_blacklist')[-1]['matching_bans'][0]['radius'] == 1.
    assert not out['completed']


def test_collision_with_clear_rays_replans_without_temporary_blacklist(replay):
    out = collision_mission(replay, [collision_plan(),
             dict(type='map', points=[[1.9, .3, .6]]),
             dict(type='candidates', values=collision_plan()['candidates']),
             dict(type='depth', stamp=1., frame=dict(rows=1, cols=1, values=[None])),
             dict(type='tick', time=1.)])
    assert out['published'][0] and out['published'][1] == []
    assert out['published'][-1]
    assert not out['states'][-1]['state']['blacklisted_targets']
    assert details(out, 'nbv_collision_rays')[0]['clear_directions'] > 0
    assert not details(out, 'blacklist_addition')


@pytest.mark.parametrize('radius', [.2, 1.8])
def test_collision_ban_uses_configured_radius_independently_of_retry_radius(replay, radius):
    target = [.1, .1, .1]
    near = [target[0] + .9 * radius, target[1], target[2]]
    far = [4.1, 4.1, .1]
    out = collision_mission(replay, [shell_map(), collision_plan(),
               dict(type='expire', time=1000.),
               dict(type='plan', use_map_collision=True, candidates=[
                    collision_plan(near)['candidates'][0],
                    collision_plan(far, [3.5, 4.1, .1], utility=.5)['candidates'][0]])],
               route_collision_blacklist_radius_m=radius, route_blacklist_radius_m=2.)
    ban = details(out, 'blacklist_addition')[0]
    assert ban['radius'] == radius and ban['expires'] is False
    assert details(out, 'nbv_collision_rays')[0]['blacklist_radius_m'] == radius
    assert details(out, 'inspection_target_suppressed')[-1]['reason'] == 'collision_blacklist'
    assert out['states'][-1]['result'] is True
    assert all(t == far for t in out['states'][-1]['state']['selected_targets'])


def test_all_occluded_views_stop_old_route_even_when_vehicle_path_is_clear(replay):
    points = [[(v + .5) * .2 for v in cell] for cell in shell(n=2)]
    out = run(replay, op='mission', config=dict(voxel_resolution=.2, max_depth=5.,
              collision_radius=.05, collision_tracking_margin=.05), events=[
              dict(type='odom', position=[3.1, .1, .6], stamp=0.), dict(type='rgb'),
              collision_plan(), dict(type='map', points=points),
              dict(type='candidates', values=collision_plan()['candidates']),
              dict(type='depth', stamp=1., frame=dict(rows=1, cols=1, values=[None])),
              dict(type='tick', time=1.)])
    assert out['published'][0] and out['published'][-1] == []
    assert not out['states'][-1]['state']['active_trajectory']
    assert details(out, 'trajectory_cancelled')[0]['reason'] == 'inspection_target_all_rays_blocked'
    assert details(out, 'blacklist_addition')[0]['reason'] == 'all_nbv_rays_collide'
    assert not out['completed']
