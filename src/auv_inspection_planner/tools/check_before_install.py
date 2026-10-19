#!/usr/bin/env python3
"""Build and run the planner regression gate before an optional installation."""

import argparse
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install', action='store_true', help='Install only after all checks pass')
    parser.add_argument('--build-dir', type=Path, help='Use a separately configured planner build')
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[3]
    build = args.build_dir or workspace / 'build/auv_inspection_planner'
    build = build.resolve()
    if not (build / 'CMakeCache.txt').exists():
        parser.error('Configure/build auv_inspection_planner in this workspace first')
    required = {
        'planner_differential', 'planner_telemetry', 'planner_nbv_cache',
        'planner_collision_recovery', 'planner_goal_standoff', 'planner_target_lifecycle',
        'planner_eligible_scoring',
    }
    subprocess.run(['cmake', '--build', str(build), '--target', 'planner_replay',
                    'inspection_planner_node', '-j', '2'], cwd=workspace, check=True)
    listing = subprocess.run(['ctest', '--test-dir', str(build), '--show-only=json-v1'],
                             cwd=workspace, text=True, capture_output=True, check=True)
    registered = {test['name'] for test in json.loads(listing.stdout)['tests']}
    missing = required - registered
    if missing:
        raise SystemExit('Required planner suites missing: ' + ', '.join(sorted(missing)))
    pattern = '^(' + '|'.join(sorted(required)) + ')$'
    subprocess.run(['ctest', '--test-dir', str(build), '-R', pattern,
                    '--no-tests=error', '--output-on-failure'], cwd=workspace, check=True)
    print('Planner regression gate passed.', flush=True)
    if args.install:
        subprocess.run(['cmake', '--install', str(build)], cwd=workspace, check=True)


if __name__ == '__main__':
    main()
