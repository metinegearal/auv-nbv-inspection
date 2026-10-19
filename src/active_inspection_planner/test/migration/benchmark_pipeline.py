"""Full depth -> OctoMap -> NBV -> route pipeline, including map serialization."""
import collections
import json
import os
import pathlib
import subprocess
import sys
import tempfile

from benchmark import summary
from cases import mission_cases
from test_differential import query

here = pathlib.Path(__file__).parent
case_name = sys.argv[2] if len(sys.argv) > 2 else 'depth_octomap_mission'
case = dict(mission_cases())[case_name]
case.update(realtime=True, write_map=True, real_map=True)
case['config']['octomap_path'] = '/tmp/planner-benchmark-pipeline.bt'
report = dict(case=case_name, description='Synthetic deterministic depth frame, real ColorOcTree. 2 warmups, 15 measured repeats, one math thread. Internal pipeline timers include actual map serialization. The capped-depth case supplies no candidates to isolate depth/map processing. No complete recorded sensor mission was available.', implementations={})
env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1', MPLCONFIGDIR='/tmp/planner-benchmark-mpl')
for name, command in [('python', [sys.executable, str(here / 'reference.py')]), ('cpp', [os.environ.get('PLANNER_REPLAY', '/tmp/planner-cpp-build/planner_replay')])]:
    worker = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, env=env)
    stages = collections.defaultdict(list)
    try:
        for iteration in range(17):
            response = query(worker, case)
            if iteration < 2:
                continue
            stages['replay_total'].extend(response['latency_ms'])
            for event in response['result']['events']:
                if event['type'] == 'stage':
                    stages[event['stage']].append(event['duration_ms'])
        report['implementations'][name] = dict(max_rss_kib=response['max_rss_kib'], stages={k: dict(**summary(v), samples_ms=v) for k, v in stages.items()})
    finally:
        worker.stdin.close()
        worker.wait(timeout=30)
    print(name, report['implementations'][name]['stages']['planning_cycle_total']['median_ms'], flush=True)
pathlib.Path(sys.argv[1]).write_text(json.dumps(report, indent=2) + '\n')
