"""Safety regressions for occupied-cell geometry and tracking-buffer recovery."""

import math
import os

import pytest

from test_nbv_cache import query, replay  # noqa: F401


def run(replay, **case):
    return query(replay, case)['result']


def wall():
    return [[0, y, z] for y in range(-12, 13) for z in range(-12, 13)]


@pytest.mark.parametrize('point', [[.11, .11, .11], [-.11, -.11, -.11],
                                  [356.71, 830.31, -295.31]])
def test_octomap_collision_cells_are_not_shifted(replay, point):
    cells = run(replay, op='octomap_collision', resolution=.2,
                scans=[dict(origin=[point[0] - 1, point[1], point[2]], points=[point])])
    assert cells == [[math.floor(x / .2) for x in point]]


def test_inflation_protects_whole_cells_including_diagonals(replay):
    blocked = {tuple(k) for k in run(replay, op='clearance_inflate',
                                   occupied=[[0, 0, 0]], resolution=.2, radius=.8)}
    assert (5, 0, 0) in blocked  # faces are only .8 m apart
    assert (4, 3, 0) in blocked  # previously outside center-based inflation
    assert (6, 0, 0) not in blocked
    for x in range(-6, 7):
        for y in range(-6, 7):
            for z in range(-6, 7):
                if (x, y, z) not in blocked:
                    gap = math.sqrt(sum(max(abs(a) - 1, 0)**2 for a in (x, y, z))) * .2
                    assert gap > .8


def test_tracking_buffer_retreat_keeps_body_clearance(replay):
    result = run(replay, op='clearance_retreat', occupied=wall(), resolution=.2,
                 radius=.4, margin=.4, start=[-.55, .1, .1])
    assert result['safe'] and result['end_free']
    assert result['start_clearance'] == pytest.approx(.55)
    assert result['end_clearance'] >= 1.3
    assert result['path'][-1][0] < result['path'][0][0]
    # The current controller may advance its queue .4 m before the endpoint.
    stop_short = result['path'][-1].copy()
    stop_short[0] += .4
    blocked = run(replay, op='clearance_inflate', occupied=wall(), resolution=.2, radius=.8)
    assert [math.floor(x / .2) for x in stop_short] not in blocked


@pytest.mark.parametrize('start', [[-.3, .1, .1], [.1, .1, .1]])
def test_body_collision_never_escapes_through_obstacles(replay, start):
    result = run(replay, op='clearance_retreat', occupied=wall(), resolution=.2,
                 radius=.4, margin=.4, start=start)
    assert result['path'] == []
    assert not result['safe']


@pytest.mark.parametrize('path', [
    [[-1., .1, .1], [1., .1, .1]],  # continuous crossing despite safe endpoints
    [[-1., .1, .1], [-.6, .1, .1]],  # moving toward the wall is not a retreat
    [[-.45, -.45, .1], [.25, -.45, .1]],  # decreasing corner clearance
])
def test_recovery_rejects_crossings_and_approach(replay, path):
    assert not run(replay, op='retreat_safe', occupied=[[0, 0, 0]],
                   resolution=.2, radius=.4, path=path)


def tick(time):
    return [dict(type='depth', stamp=time, frame=dict(rows=1, cols=1, values=[None])),
            dict(type='tick', time=time)]


def mission(replay, extra=(), start=None, move_during_recovery=None):
    initial = [dict(type='mag', value=[0., 1., 0.]),
               dict(type='odom', position=start or [-.55, .1, .1], stamp=1.),
               dict(type='rgb'), dict(type='candidates', values=[]),
               dict(type='map', points=[[(a + .5) * .2 for a in k] for k in wall()])]
    first_tick = tick(1.)
    if move_during_recovery is not None:
        first_tick[-1]['move_during_recovery'] = move_during_recovery
    return run(replay, op='mission', config=dict(voxel_resolution=.2, collision_radius=.4,
               collision_tracking_margin=.4, route_stall_timeout_s=6., planning_period=1),
               events=[*initial, *first_tick, *extra])


def filters(result, name):
    return [event for event in result['events'] if event.get('stage') == name]


def test_recovery_runs_without_nbvs_and_resumes_after_clearance(replay):
    result = mission(replay, [dict(type='odom', stamp=2., position=[-1.75, .1, .1]), *tick(2.)])
    assert result['published'][0]  # retreat despite empty NBV list
    assert result['published'][1] == []  # clear retreat before resuming NBV planning
    assert not result['completed']
    state = result['states'][-1]['state']
    assert state['failed_plan_count'] == 1  # only the subsequent normal NBV pass failed
    assert not state['active_trajectory']
    assert state['_active_route_failure_reason'] is None  # normal route safety ran again


def test_recovery_does_not_cancel_just_after_clearing_buffer(replay):
    result = mission(replay, [dict(type='odom', stamp=2., position=[-1.11, .1, .1]), *tick(2.)])
    assert len(result['published']) == 1
    assert len(result['published'][0]) == 4  # send the destination, not a current-pose hold
    assert result['states'][-1]['state']['active_trajectory']
    assert not filters(result, 'trajectory_cancelled')


def test_new_obstacle_cancels_recovery(replay):
    result = mission(replay, [dict(type='map', points=[[-1., .1, .1]]), *tick(2.)])
    assert len(result['published']) == 2 and result['published'][-1] == []
    assert not result['states'][-1]['state']['active_trajectory']
    assert not result['completed']


def test_stalled_recovery_cancels_and_waits_before_retry(replay):
    result = mission(replay, [*tick(8.), *tick(9.)])
    assert len(result['published']) == 2 and result['published'][-1] == []
    assert result['states'][-1]['state']['failed_plan_count'] == 0
    assert not result['completed']


def test_no_safe_retreat_holds_without_false_completion(replay):
    result = mission(replay, [*tick(2.), *tick(3.), *tick(4.), *tick(8.)], start=[-.3, .1, .1])
    assert not result['published'] and not result['completed']
    assert result['states'][-1]['state']['_plan_failure_reason'] == 'clearance_recovery_blocked'
    assert result['states'][-1]['state']['failed_plan_count'] == 0
    assert len(filters(result, 'clearance_recovery')) == 2  # rate-limited retry


def test_motion_during_recovery_search_is_revalidated(replay):
    result = mission(replay, move_during_recovery=[-.3, .1, .1])
    assert not result['published']
    assert result['states'][-1]['state']['_plan_failure_reason'] == 'clearance_recovery_wire_collision'


def test_latest_saved_map_stuck_position(replay):
    snapshot = os.environ.get('PLANNER_WALL_MAP')
    # Explicit opt-in: a later run overwrites results/final_map_ros2.bt.
    if not snapshot:
        pytest.skip('set PLANNER_WALL_MAP to the October 16 snapshot')
    result = run(replay, op='clearance_retreat', load=snapshot, resolution=.2,
                 radius=.4, margin=.4, start=[356.096, 830.308, -295.366])
    assert result['safe'] and result['end_free'], result
    assert result['end_clearance'] >= 1.3
