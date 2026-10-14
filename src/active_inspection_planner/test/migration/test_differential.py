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


CASES = list(cases())


@pytest.mark.parametrize('name,case', CASES, ids=[name for name, _ in CASES])
def test_reference_equivalence(replay, name, case):
    expected = clean(run(case))
    actual = query(replay, case)['result']
    tolerance = 1e-9
    if case['op'] == 'optimize':
        tolerance = 2e-6  # <=2 micrometres/radians absolute plus relative rounding allowance
    compare(expected, actual, tolerance=tolerance)


def test_reference_sources_unchanged():
    import hashlib
    package = pathlib.Path(__file__).resolve().parents[2]
    manifest = json.loads((package / 'migration_evidence/reference_sha256.json').read_text())
    for relative, expected in manifest.items():
        assert hashlib.sha256((package.parent / relative).read_bytes()).hexdigest() == expected, relative
