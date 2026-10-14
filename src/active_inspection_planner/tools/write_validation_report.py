"""Render the measured evidence without discarding raw samples."""
import hashlib
import json
import pathlib

package = pathlib.Path(__file__).resolve().parents[1]
evidence = package / 'migration_evidence'
bench = json.loads((evidence / 'benchmark.json').read_text())
pipeline = json.loads((evidence / 'pipeline_benchmark.json').read_text())
numeric = json.loads((evidence / 'numerical_errors.json').read_text())
lines = ['# Migration validation results', '',
         'Validated on 2026-10-13. Runtime source edits are confined to `active_inspection_planner`.', '',
         '## Build and correctness', '',
         '* Isolated Release colcon build/install: passed.',
         '* Differential tests: **177 passed** (current Python source versus C++).',
         '* File telemetry equivalence: **1 passed**.',
         '* ROS and selectable-launch tests: **5 passed**, including default and shipped configurations for both nodes.',
         '* Existing C++ policy regression: passed. Targeted CTest run: **3/3 passed**.',
         '* Full automatic lint suite was not certified; reference lint imports are unavailable, and vendor code retains upstream formatting.',
         '* Python source hash guard passed. No changes were committed.', '',
         'Logs: `migration_evidence/{build,ctest,differential_tests,telemetry_tests,ros_tests}.txt`.', '',
         '## Numerical errors', '',
         'Discrete decisions, ordering, indices, reasons, and state transitions match in these fixtures. '
         'Ordinary floating-point outputs use absolute and relative tolerances of 1e-9. '
         'Optional optimization uses 2e-6 absolute and relative tolerances (metres/radians), '
         'because finite differences amplify rounding differences.', '',
         '| Component | Largest observed absolute difference |', '| --- | ---: |']
for label, predicate in [('FITPACK spline', lambda k: k.startswith('spline_')),
                         ('PCA', lambda k: k.startswith('pca_')),
                         ('Open3D preprocessing', lambda k: k.startswith('preprocessing_')),
                         ('NBV including recorded map', lambda k: k.startswith('nbv_') or k == 'recorded_map_nbvs'),
                         ('L-BFGS-B trajectory', lambda k: k.startswith('optimizer')),
                         ('OctoMap extraction', lambda k: k.startswith('octomap') or k == 'recorded_map_extraction')]:
    maximum = max((v['max_absolute_error'] for k, v in numeric.items() if predicate(k)), default=0)
    lines.append(f'| {label} | {maximum:.9g} |')
lines += ['', '## Performance method', '',
          f"Host: {bench['environment']['cpu']}; {bench['environment']['platform']}.", '',
          'Python 3.12, NumPy 1.26.4, SciPy 1.16.3, Open3D 0.19.0; GCC/GFortran 13, Release C++17. '
          'Two warmups followed by 15 measurements per workload, one OpenMP/BLAS thread. '
          'Both implementations receive the same serialized fixtures in persistent worker processes. '
          'Each workload gets a fresh worker for independent memory measurements. '
          'Stage measurements use internal monotonic clocks; process startup and pipe transport are excluded. '
          'Operation timings include adapter/replay construction and trace capture. '
          'Peak memory uses Linux `/proc/self/status` VmHWM, avoiding inherited pre-exec `getrusage` peaks. '
          'The machine also ran an existing mission, so CPU contention and variance remain visible. '
          'This is a reproducible microbenchmark, not a controlled full simulator mission comparison.', '',
          '| Workload | Python median ± SD (ms) | C++ median ± SD (ms) | C++/Python | Peak RSS Python / C++ (MiB) |',
          '| --- | ---: | ---: | ---: | ---: |']
for name, row in bench['cases'].items():
    p, c = row['python'], row['cpp']
    lines.append(f"| {name} | {p['median_ms']:.3f} ± {p['stdev_ms']:.3f} | {c['median_ms']:.3f} ± {c['stdev_ms']:.3f} | {row['cpp_over_python_median']:.3f} | {p['max_rss_kib']/1024:.1f} / {c['max_rss_kib']/1024:.1f} |")
lines += ['', 'All per-run samples, p95, min/max, means, standard deviations and throughput values are retained in `migration_evidence/benchmark.json`.', '',
          '## Full synthetic depth/map planning cycle', '',
          'A 30×40 deterministic depth frame runs through real ColorOcTree insertion, extraction, inflation, NBV, routing, validation and actual map-file serialization. '
          'This is synthetic sensor input, not a recorded mission. Timings below are internal stage latencies.', '',
          '| Stage | Python median ± SD (ms) | C++ median ± SD (ms) |', '| --- | ---: | ---: |']
p, c = (pipeline['implementations'][k] for k in ('python', 'cpp'))
for name, row in p['stages'].items():
    if name not in c['stages']: continue
    other = c['stages'][name]
    lines.append(f"| {name} | {row['median_ms']:.3f} ± {row['stdev_ms']:.3f} | {other['median_ms']:.3f} ± {other['stdev_ms']:.3f} |")
ps, cs = p['stages']['planning_cycle_total'], c['stages']['planning_cycle_total']
lines += ['', f"Full-cycle median speedup: **{ps['median_ms']/cs['median_ms']:.2f}×**. Mean-derived throughput: **{ps['throughput_per_s']:.2f} versus {cs['throughput_per_s']:.2f} cycles/s**. "
          f"Peak worker RSS: **{p['max_rss_kib']/1024:.1f} versus {c['max_rss_kib']/1024:.1f} MiB** (Python versus C++).", '',
          '## Regression investigation and remaining limits', '',
          'The capped-depth **mock-map replay** regressed in the final measurement. '
          'That fixture materializes a 4,000-point insertion trace and many JSON state snapshots, unlike the live OctoMap adapter. '
          'It is retained as an observed replay/adapter regression; it is not hidden by averaging other workloads.']
cap_path = evidence / 'capped_depth_benchmark.json'
if cap_path.exists():
    cap = json.loads(cap_path.read_text())['implementations']
    a, b = (cap[k]['stages']['depth_to_octomap'] for k in ('python', 'cpp'))
    lines += ['', f"To investigate, the same capped frame was rerun against **real OctoMap**. The measured `depth_to_octomap` stage was "
              f"{a['median_ms']:.3f} ± {a['stdev_ms']:.3f} ms in Python and {b['median_ms']:.3f} ± {b['stdev_ms']:.3f} ms in C++. "
              'This stage still includes insertion-trace capture, so it does not isolate production projection/insertion from harness serialization overhead. '
              'The capped-depth stage regression remains unresolved despite the faster complete cycle; see `capped_depth_benchmark.json` for all stages and samples.']
lines += ['', 'No raw depth recording or complete mission ROS bag was available. Frozen map/candidate/pose/route output logs and synthetic sensor scenarios were used. '
          'Full recorded-sensor mission equivalence and live-mission performance are therefore unverified. '
          'Unseeded random subsamples differ across runs. Console numeric formatting and JSON whitespace/order are not byte-identical. '
          'The known baseline failures are documented in MIGRATION.md and were not fixed.', '',
          '## Reproduce benchmarks', '', '```bash',
          'PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \\',
          'python3 src/active_inspection_planner/test/migration/benchmark.py \\',
          '  --cpp /tmp/planner-cpp-build/planner_replay --repeats 15 --output /tmp/planner-benchmark.json',
          'PLANNER_REPLAY=/tmp/planner-cpp-build/planner_replay \\',
          'python3 src/active_inspection_planner/test/migration/benchmark_pipeline.py /tmp/planner-pipeline.json',
          'PLANNER_REPLAY=/tmp/planner-cpp-build/planner_replay \\',
          'python3 src/active_inspection_planner/test/migration/benchmark_pipeline.py /tmp/planner-cap.json depth_cap_randomstate',
          '```', '', 'Replace the executable with your installed `lib/active_inspection_planner/planner_replay` path as needed.', '']
(package / 'VALIDATION.md').write_text('\n'.join(lines))
frozen = package / 'test/migration/recorded'
(evidence / 'fixture_sha256.json').write_text(json.dumps({str(p.relative_to(package)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(frozen.iterdir()) if p.is_file()}, indent=2)+'\n')
