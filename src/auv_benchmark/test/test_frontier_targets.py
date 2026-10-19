"""Overlay lifecycle and bounded log reads without rerunning planner algorithms."""

import json

import numpy as np

from auv_benchmark.adapters.frontier_targets import FrontierTargets


def event(stage, target=None):
    return (json.dumps({'stage': stage, 'details': {'target': target}}) + '\n').encode()


def test_attach_recovers_latest_target_not_unsuccessful_candidates(tmp_path):
    path = tmp_path / 'filters.jsonl'
    path.write_bytes(event('inspection_attempt_started', [1, 2, 3]) +
                     event('nbv_collision_rays', [99, 99, 99]))
    reader = FrontierTargets(path)
    np.testing.assert_array_equal(reader.poll(), [[1, 2, 3]])
    points = reader.points
    assert reader.poll() is points  # No geometry upload on an unchanged poll.


def test_partial_append_cancel_and_replacement(tmp_path):
    path = tmp_path / 'filters.jsonl'
    partial = event('inspection_attempt_started', [1, 2, 3])
    path.write_bytes(partial[:-10])
    reader = FrontierTargets(path)
    assert not len(reader.poll())
    with path.open('ab') as stream:
        stream.write(partial[-10:])
    np.testing.assert_array_equal(reader.poll(), [[1, 2, 3]])
    with path.open('ab') as stream:
        stream.write(event('trajectory_cancelled'))
    assert not len(reader.poll())
    with path.open('ab') as stream:
        stream.write(event('inspection_attempt_started', [4, 5, 6]))
    np.testing.assert_array_equal(reader.poll(), [[4, 5, 6]])


def test_initial_backwards_search_is_bounded_and_joins_split_rows(tmp_path):
    path = tmp_path / 'filters.jsonl'
    path.write_bytes(event('inspection_attempt_started', [1, 2, 3]) +
                     event('planning_begin') * 15)
    reader = FrontierTargets(path, read_bytes=150)
    assert not len(reader.poll())
    assert reader.back_position == path.stat().st_size - 150
    for _ in range(20):
        reader.poll()
        if not reader.searching:
            break
    np.testing.assert_array_equal(reader.points, [[1, 2, 3]])


def test_new_live_event_supersedes_startup_history_search(tmp_path):
    path = tmp_path / 'filters.jsonl'
    path.write_bytes(event('inspection_attempt_started', [1, 2, 3]) +
                     event('planning_begin') * 30)
    reader = FrontierTargets(path, read_bytes=150)
    reader.poll()
    with path.open('ab') as stream:
        stream.write(event('inspection_attempt_started', [4, 5, 6]))
    np.testing.assert_array_equal(reader.poll(), [[4, 5, 6]])
    assert not reader.searching
    np.testing.assert_array_equal(reader.poll(), [[4, 5, 6]])


def test_reset_on_rotation_truncation_and_same_length_rewrite(tmp_path):
    path = tmp_path / 'filters.jsonl'
    path.write_bytes(event('inspection_attempt_started', [1, 2, 3]))
    reader = FrontierTargets(path)
    reader.poll()
    path.write_bytes(event('inspection_attempt_started', [4, 5, 6]))
    np.testing.assert_array_equal(reader.poll(), [[4, 5, 6]])
    path.write_bytes(b'')
    assert not len(reader.poll())
    replacement = tmp_path / 'replacement'
    replacement.write_bytes(event('inspection_attempt_started', [7, 8, 9]))
    replacement.replace(path)
    np.testing.assert_array_equal(reader.poll(), [[7, 8, 9]])
    path.unlink()
    assert not len(reader.poll())


def test_malformed_targets_and_completed_goals(tmp_path):
    path = tmp_path / 'filters.jsonl'
    path.write_bytes(event('inspection_attempt_started', [1, 2, 3]) + b'broken json\n' +
                     event('inspection_attempt_started', [float('nan'), 2, 3]) + b'[]\n')
    reader = FrontierTargets(path)
    np.testing.assert_array_equal(reader.poll(), [[1, 2, 3]])
    for stage in ('inspection_target_observed', 'mission_completion'):
        with path.open('ab') as stream:
            stream.write(event(stage))
        assert not len(reader.poll())
