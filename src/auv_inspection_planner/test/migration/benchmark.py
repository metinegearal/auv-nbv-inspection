"""Same inputs, persistent isolated processes, internal stage timing, fixed threads."""
import argparse
import hashlib
import json
import os
import pathlib
import platform
import statistics
import subprocess
import sys
import time

import numpy as np
from cases import cases
from test_differential import query


def summary(samples):
    return dict(n=len(samples), median_ms=statistics.median(samples),
                mean_ms=statistics.mean(samples), stdev_ms=statistics.stdev(samples) if len(samples)>1 else 0.,
                p95_ms=float(np.percentile(samples, 95)), min_ms=min(samples), max_ms=max(samples),
                throughput_per_s=1000 / statistics.mean(samples))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cpp', default=os.environ.get('PLANNER_REPLAY', '/tmp/planner-cpp-build/planner_replay'))
    parser.add_argument('--repeats', type=int, default=15)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    here = pathlib.Path(__file__).parent
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1', MPLCONFIGDIR='/tmp/planner-benchmark-mpl')
    selected = {'astar_obstacles_0', 'inflate_1.0', 'spline_30_2.0', 'preprocessing_corner',
                'pca_random', 'clusters_plane', 'nbv_corner_0', 'optimizer_10', 'octomap_rays',
                'depth_cap_randomstate', 'unsafe_spline_corner_fallback', 'recorded_map_nbvs',
                'recorded_candidates_248', 'recorded_pose_history', 'recorded_route_248'}
    inputs = {name: case for name, case in cases() if name in selected}
    for name, case in inputs.items():
        case['benchmark'] = True
    report = dict(environment=dict(platform=platform.platform(), python=sys.version,
                                  cpu=next((x.split(':', 1)[1].strip() for x in pathlib.Path('/proc/cpuinfo').read_text().splitlines() if x.startswith('model name')), ''),
                                  threads=1, warmups=2, repeats=args.repeats), cases={})
    # Fresh worker per implementation and fixture isolates peak RSS. Startup,
    # JSON transport, and serialization are excluded from internal latency.
    for name, case in inputs.items():
        record = dict(input_sha256=hashlib.sha256(json.dumps(case, sort_keys=True).encode()).hexdigest())
        for implementation, command in [('python', [sys.executable, str(here / 'reference.py')]), ('cpp', [args.cpp])]:
            process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, env=env)
            try:
                query(process, {**case, 'repeat': 2})
                result = query(process, {**case, 'repeat': args.repeats})
                record[implementation] = dict(**summary(result['latency_ms']), samples_ms=result['latency_ms'], max_rss_kib=result['max_rss_kib'])
            finally:
                process.stdin.close()
                process.wait(timeout=30)
        record['cpp_over_python_median'] = record['cpp']['median_ms']/record['python']['median_ms']
        report['cases'][name] = record
        print(name, f"Python {record['python']['median_ms']:.3f} ms; C++ {record['cpp']['median_ms']:.3f} ms; ratio {record['cpp_over_python_median']:.3f}", flush=True)
        pathlib.Path(args.output).write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
