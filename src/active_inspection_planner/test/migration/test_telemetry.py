"""Compare file adapters using actual mission events, including append/reset rules."""
import csv
import json
import subprocess

from cases import mission_cases
from reference import run
from test_differential import compare, query, replay


def test_result_files_match_reference(replay, tmp_path):
    from telemetry_adapters.file_sinks import PlannerFileSink
    from python_planner.adapters.files import open_evaluation_log
    case = dict(mission_cases())['completion_three_passes']
    expected = run(case)
    python_dir, cpp_dir = tmp_path / 'python', tmp_path / 'cpp'
    python_dir.mkdir(); cpp_dir.mkdir()
    for path in [python_dir, cpp_dir]:
        (path / 'evaluation_parameters.txt').write_text('existing evaluation\n')
        (path / 'planner_nbv_iterations.jsonl').write_text('old run\n')
    sink = PlannerFileSink(str(python_dir))
    for event in expected['events']:
        event = dict(event)
        kind = event.pop('type')
        if kind == 'stage': sink.record_stage(**event)
        elif kind == 'filter': sink.record_filter(**event)
        elif kind == 'depth_pose':
            sink.record_depth_pose(event['sequence'], event['depth_time_s'], event['position'], event['roll'], event['pitch'], event['yaw'])
        elif kind == 'iteration': sink.record_iteration(**event)
    with open_evaluation_log(str(python_dir)) as file:
        file.writelines(expected['evaluation'])
    query(replay, {**case, 'files': str(cpp_dir)})
    assert sorted(p.name for p in python_dir.iterdir()) == sorted(p.name for p in cpp_dir.iterdir())
    for reference in python_dir.iterdir():
        actual = cpp_dir / reference.name
        if reference.suffix == '.jsonl':
            compare([json.loads(x) for x in reference.read_text().splitlines()], [json.loads(x) for x in actual.read_text().splitlines()])
        elif reference.suffix == '.csv':
            with reference.open(newline='') as left, actual.open(newline='') as right:
                assert list(csv.reader(left)) == list(csv.reader(right)), reference.name
        else:
            # C++ iostream and Python render exact integer-valued floats differently.
            left, right = reference.read_text().splitlines(), actual.read_text().splitlines()
            assert left[0] == right[0] == 'existing evaluation'
            assert len(left) == len(right)
            for a, b in zip(left[1:], right[1:]):
                assert a.replace('0.0,', '0,') == b
