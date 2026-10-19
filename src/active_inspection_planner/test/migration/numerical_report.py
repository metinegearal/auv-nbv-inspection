"""Record measured numeric errors; decisions still use the differential assertions."""
import json
import os
import pathlib
import subprocess
import sys

from cases import cases
from reference import clean, run
from test_differential import compare, query


def maximum(a, b):
    if isinstance(a, dict):
        return max((maximum(a[k], b[k]) for k in a), default=0.)
    if isinstance(a, list):
        return max((maximum(x, y) for x, y in zip(a, b)), default=0.)
    if isinstance(a, float):
        return abs(a-b)
    return 0.


worker = subprocess.Popen([os.environ.get('PLANNER_REPLAY', '/tmp/planner-cpp-build/planner_replay')], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
report = {}
try:
    for name, case in cases():
        if case['op'] not in ('smooth', 'pca', 'prepare', 'nbv', 'optimize', 'octomap'):
            continue
        expected = clean(run(case))
        actual = query(worker, case)['result']
        tolerance = 2e-6 if case['op'] == 'optimize' else 1e-9
        compare(expected, actual, tolerance=tolerance)
        report[name] = dict(max_absolute_error=maximum(expected, actual), atol=tolerance, rtol=tolerance)
finally:
    worker.stdin.close(); worker.wait(timeout=30)
pathlib.Path(sys.argv[1]).write_text(json.dumps(report, indent=2) + '\n')
