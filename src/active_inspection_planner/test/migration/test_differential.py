import json
import os
import pathlib
import subprocess
import numpy as np
import pytest

from cases import cases
from reference import clean, run


@pytest.fixture(scope='session')
def replay():
    executable = os.environ.get('PLANNER_REPLAY', '/tmp/planner-cpp-build/planner_replay')
    process = subprocess.Popen([executable], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               text=True, bufsize=1)
    yield process
    process.stdin.close()
    process.wait(timeout=20)
    assert process.returncode == 0


def query(process, value):
    process.stdin.write(json.dumps(clean(value)) + '\n')
    process.stdin.flush()
    while True:
        line = process.stdout.readline()
        assert line, 'C++ replay exited before responding'
        if line.startswith('{'):
            result = json.loads(line)
            assert 'error' not in result, result
            return result


def compare(expected, actual, path='', tolerance=1e-9):
    if isinstance(expected, dict):
        assert set(expected) == set(actual), (path, set(expected) ^ set(actual))
        for key in expected:
            if key in ('duration_ms', 'elapsed_ms', 'evaluation'):
                continue
            compare(expected[key], actual[key], path + '/' + key, tolerance)
    elif isinstance(expected, list):
        assert len(expected) == len(actual), (path, len(expected), len(actual))
        if expected and all(isinstance(x, (float, int)) and not isinstance(x, bool) for x in expected):
            np.testing.assert_allclose(actual, expected, atol=tolerance, rtol=tolerance, err_msg=path)
        else:
            for index, (left, right) in enumerate(zip(expected, actual)):
                compare(left, right, path + '/' + str(index), tolerance)
    elif isinstance(expected, float):
        assert actual == pytest.approx(expected, abs=tolerance, rel=tolerance), (path, expected, actual)
    else:
        assert expected == actual, (path, expected, actual)


# This real-map mission intentionally has new execution semantics (one observed
# target per route). Its original input remains frozen and is checked below
# against both the historical mapping/NBV results and the new safety contract.
CASES = [(name, case) for name, case in cases() if name != 'depth_octomap_mission']


@pytest.mark.parametrize('name,case', CASES, ids=[name for name, _ in CASES])
def test_reference_equivalence(replay, name, case):
    expected = clean(run(case))
    actual = query(replay, case)['result']
    if name == 'defaults':
        # New C++ safety setting; all preserved defaults are still compared.
        expected['route_goal_max_extension_m'] = 1.5
        expected['route_target_max_attempts'] = 3
        expected['route_target_timeout_s'] = 120.
        expected['route_blacklist_radius_m'] = .5
        expected['route_collision_blacklist_radius_m'] = 1.
    tolerance = 1e-9
    if case['op'] == 'optimize':
        tolerance = 2e-6  # <=2 micrometres/radians absolute plus relative rounding allowance
    compare(expected, actual, tolerance=tolerance)


def test_real_map_mission_preserves_mapping_and_executes_one_visible_target(replay):
    case = dict(cases())['depth_octomap_mission']
    expected = clean(run(case))
    actual = query(replay, case)['result']
    compare(expected['inserts'], actual['inserts'])
    compare(expected['writes'], actual['writes'])
    compare(expected['states'][:-1], actual['states'][:-1])
    before, after = expected['states'][-1]['state'], actual['states'][-1]['state']
    for key in before:
        if key not in ('active_trajectory', 'selected_targets'):
            compare(before[key], after[key], path=key)
    historical_nbvs = next(e['candidates'] for e in expected['events'] if e['type'] == 'iteration')
    current_nbvs = next(e['candidates'] for e in actual['events'] if e['type'] == 'iteration')
    compare(historical_nbvs, current_nbvs)
    targets = after['selected_targets']
    assert targets and len({tuple(t) for t in targets}) == 1
    assert len([e for e in actual['events'] if e.get('stage') == 'inspection_attempt_started']) == 1
    route = after['active_trajectory']
    assert len(route) > 1 and actual['published'][-1]
    wire = np.array(actual['published'][-1]).reshape(-1, 4)
    executed = wire - [.35, .3875, -.5, 0.]
    np.testing.assert_allclose(executed, np.array(route)[1:], atol=1e-9)
    assert np.linalg.norm(executed[0, :3] - np.array(after['position'])) > .01
    raw = query(replay, dict(op='octomap_collision', resolution=.2, scans=actual['inserts']))['result']
    occupied = query(replay, dict(op='clearance_inflate', occupied=raw, radius=.2, resolution=.2))['result']
    assert len(occupied) == 2483
    assert query(replay, dict(op='collision', occupied=occupied, resolution=.2,
                             path=[p[:3] for p in route]))['result']
    assert not actual['completed']


def test_reference_sources_unchanged():
    import hashlib
    package = pathlib.Path(__file__).resolve().parents[2]
    manifest = json.loads((package / 'migration_evidence/reference_sha256.json').read_text())
    for relative, expected in manifest.items():
        assert hashlib.sha256((package.parent / relative).read_bytes()).hexdigest() == expected, relative
