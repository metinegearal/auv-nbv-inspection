"""Exercise local geometry refresh and global candidate selection in C++ replay."""

import json
import os
import subprocess

import pytest


@pytest.fixture(scope='module')
def replay():
    executable = os.environ.get('PLANNER_REPLAY', '/tmp/planner-cpp-build/planner_replay')
    process = subprocess.Popen([executable], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               text=True, bufsize=1)
    yield process
    process.stdin.close()
    process.wait(timeout=20)
    assert process.returncode == 0


def query(process, value):
    process.stdin.write(json.dumps(value) + '\n')
    process.stdin.flush()
    while True:
        line = process.stdout.readline()
        assert line, 'C++ replay exited before responding'
        if line.startswith('{'):
            result = json.loads(line)
            assert 'error' not in result, result
            return result


def surface(offset=0.):
    return [[offset + x * .15, y * .15, 0.] for x in range(15) for y in range(15)]


def step(position, points=None, **parameters):
    return dict(points=surface() if points is None else points, position=position,
                heading=[1., 0., 0.],
                parameters=dict(exploration_radius=4., seabed_enabled=False, **parameters))


def sequence(replay, *steps):
    return query(replay, dict(op='nbv_sequence', steps=list(steps)))['result']


def test_distant_clusters_remain_candidates_and_cost_tracks_current_distance(replay):
    near, far, closer = sequence(
        replay, step([.3, .2, 2.]), step([20., .2, 2.]), step([10., .2, 2.]))
    assert near['candidates']
    assert far['candidates']
    assert far['clusters'] == closer['clusters'] == near['clusters']
    assert min(c['raw_cost'] for c in far['candidates']) > min(
        c['raw_cost'] for c in closer['candidates'])
    # The same far-away pose has no destinations without the mission cache.
    stateless = query(replay, dict(op='nbv', **step([20., .2, 2.])))['result']
    assert stateless == []


def test_local_refresh_replaces_geometry_without_accumulating_duplicates(replay):
    before, after = sequence(replay, step([.3, .2, 2.]), step([.3, .2, 2.]))
    assert before['clusters']
    assert before['candidates']
    assert len(after['clusters']) == len(before['clusters'])
    assert {c['id'] for c in before['clusters']}.isdisjoint(
        c['id'] for c in after['clusters'])
    old = [dict(c, cluster_id=0) for c in before['candidates']]
    new = [dict(c, cluster_id=0) for c in after['candidates']]
    assert old == new


def test_local_frontiers_can_disappear_while_distant_clusters_survive(replay):
    world = surface() + surface(20.)
    first, both, refreshed = sequence(
        replay, step([.3, .2, 2.], world), step([20.3, .2, 2.], world),
        step([.3, .2, 2.], world, frontier_angle_threshold_deg=359.))
    assert len(both['clusters']) > len(first['clusters']) > 0
    assert refreshed['candidates']
    assert all(c['centroid'][0] > 15. for c in refreshed['clusters'])
    assert all(c['target'][0] > 15. for c in refreshed['candidates'])
    assert max(c['utility'] for c in refreshed['candidates']) >= .1


@pytest.mark.parametrize('remaining', [[], surface(20.)])
def test_removed_map_support_invalidates_distant_clusters(replay, remaining):
    before, after = sequence(
        replay, step([.3, .2, 2.]), step([40., .2, 2.], remaining))
    assert before['candidates']
    assert after == dict(candidates=[], clusters=[])


def test_seen_distant_clusters_stay_suppressed(replay):
    world = surface()
    initial = sequence(replay, step([.3, .2, 2.], world))[0]
    # Close historical observations satisfy the existing visibility test.
    history = [[[c['centroid'][0], c['centroid'][1], c['centroid'][2] + .3], 0.]
               for c in initial['clusters']]
    populated, far = sequence(
        replay, step([.3, .2, 2.], world),
        dict(step([20., .2, 2.], world), history=history))
    assert populated['candidates']
    assert far['clusters'] == populated['clusters']
    assert far['candidates'] == []
