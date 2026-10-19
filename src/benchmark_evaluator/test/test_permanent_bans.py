"""Permanent exclusions survive recording without becoming stale across runs."""

import json
from pathlib import Path

import pytest

from benchmark_evaluator.adapters.permanent_bans import (
    BAN_SPHERES_FILENAME, PermanentBanRecorder, load_ban_spheres, target_is_banned,
)


def event(stage='blacklist_addition', center=None, **details):
    return dict(stage=stage, iteration=4, frame_id=10,
                details=dict(target=center or [10., 20., -30.], **details))


def append(path, *events):
    with path.open('a') as stream:
        for value in events:
            stream.write(json.dumps(value) + '\n')


def test_records_permanent_spheres_and_ignores_temporary_bans(tmp_path):
    source = tmp_path / 'planner_nbv_filters.jsonl'
    append(source, event(expires=True, radius=.5, cooldown_s=30.),
           event(expires=False, radius=1., reason='all_nbv_rays_collide'))
    recorder = PermanentBanRecorder(tmp_path)
    assert recorder.poll()
    assert not recorder.poll()
    spheres = load_ban_spheres(tmp_path / BAN_SPHERES_FILENAME)
    assert len(spheres) == 1
    assert spheres[0]['center'] == [10., 20., -30.]
    assert spheres[0]['radius_m'] == 1.
    assert spheres[0]['reason'] == 'all_nbv_rays_collide'
    assert spheres[0]['iteration'] == 4


def test_retry_exhaustion_uses_original_attempt_center_despite_target_jitter(tmp_path):
    source = tmp_path / 'planner_nbv_filters.jsonl'
    append(source, event('inspection_attempt_started', attempt=1),
           event('inspection_attempt_started', center=[10.2, 20., -30.], attempt=2),
           event('inspection_target_suppressed', center=[10.3, 20., -30.],
                 reason='attempt_budget_exhausted'),
           event('inspection_target_suppressed', center=[10.4, 20., -30.],
                 reason='attempt_budget_exhausted'),
           event('inspection_target_suppressed', center=[50., 20., -30.],
                 reason='observed_at_resolved_viewpoint'))
    recorder = PermanentBanRecorder(tmp_path)
    recorder.poll()
    assert len(recorder.spheres) == 1
    assert recorder.spheres[0]['center'] == [10., 20., -30.]
    assert recorder.spheres[0]['radius_m'] == .5


def test_partial_appends_deduplicate_and_allow_radius_upgrade(tmp_path):
    source = tmp_path / 'planner_nbv_filters.jsonl'
    value = json.dumps(event(expires=False, radius=.5))
    source.write_text(value[:30])
    recorder = PermanentBanRecorder(tmp_path)
    recorder.poll()
    assert not recorder.spheres
    with source.open('a') as stream:
        stream.write(value[30:] + '\n')
    recorder.poll()
    append(source, event(expires=False, radius=.5), event(expires=False, radius=1.))
    recorder.poll()
    assert len(recorder.spheres) == 1 and recorder.spheres[0]['radius_m'] == 1.
    archive = recorder.save_snapshot(tmp_path / 'permanent_ban_spheres_run.json')
    assert load_ban_spheres(archive) == recorder.spheres


@pytest.mark.parametrize('replacement', ['truncate', 'rotate', 'rewrite_larger'])
def test_new_planner_log_clears_previous_mission_bans(tmp_path, replacement):
    source = tmp_path / 'planner_nbv_filters.jsonl'
    append(source, event(expires=False, radius=1.))
    recorder = PermanentBanRecorder(tmp_path)
    recorder.poll()
    if replacement == 'rotate':
        source.rename(tmp_path / 'old.jsonl')
    source.write_text('')
    if replacement == 'rewrite_larger':
        append(source, *[event('depth_range', sample=i) for i in range(10)])
    recorder.poll()
    assert load_ban_spheres(recorder.output) == []
    append(source, event(center=[100., 200., -30.], expires=False, radius=.5))
    recorder.poll()
    assert [s['center'] for s in recorder.spheres] == [[100., 200., -30.]]


def test_malformed_diagnostics_do_not_drop_later_valid_bans(tmp_path):
    source = tmp_path / 'planner_nbv_filters.jsonl'
    source.write_text('invalid json\n42\n[]\n')
    append(source, event(center=[1, 2], expires=False, radius=1.),
           event(expires=False, radius=-1.), event(expires=False, radius=1.))
    recorder = PermanentBanRecorder(tmp_path)
    recorder.poll()
    assert len(recorder.spheres) == 1


def test_radius_boundary_uses_3d_map_coordinates(tmp_path):
    source = tmp_path / 'planner_nbv_filters.jsonl'
    append(source, event(expires=False, radius=1.))
    recorder = PermanentBanRecorder(tmp_path)
    recorder.poll()
    spheres = load_ban_spheres(recorder.output)
    assert target_is_banned([10.6, 20.8, -30.], spheres)
    assert target_is_banned([10., 20., -29.], spheres)
    assert not target_is_banned([10., 20., -28.99], spheres)
    assert load_ban_spheres(tmp_path / 'missing.json') == []


def test_viewer_wire_sphere_has_exact_saved_center_and_radius(tmp_path, monkeypatch):
    import numpy as np

    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[3]))
    import test_env_2

    geometries = []
    test_env_2.add_ban_spheres(geometries, [dict(center=[10., 20., -30.], radius_m=1.)])
    wire = geometries[0]
    vertices = np.asarray(wire.points)
    np.testing.assert_allclose(vertices.min(axis=0), [9., 19., -31.])
    np.testing.assert_allclose(vertices.max(axis=0), [11., 21., -29.])
    assert len(wire.lines) > 0


def test_viewer_refreshes_active_run_bans_but_respects_explicit_snapshot(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[3]))
    import test_env_2

    source = tmp_path / 'planner_nbv_filters.jsonl'
    append(source, event(expires=False, radius=1.))
    path, spheres = test_env_2.load_viewer_bans(tmp_path / 'final_map_ros2.bt')
    assert path == tmp_path / BAN_SPHERES_FILENAME
    assert len(spheres) == 1
    frozen = PermanentBanRecorder(tmp_path).save_snapshot(tmp_path / 'frozen.json')
    append(source, event(center=[100., 200., -30.], expires=False, radius=.5))
    assert test_env_2.load_viewer_bans(tmp_path / 'final_map_ros2.bt', frozen)[1] == []
    assert len(test_env_2.load_viewer_bans(tmp_path / 'final_map_ros2.bt')[1]) == 2


@pytest.mark.parametrize('radius', [.2, 1.2])
def test_configured_retry_radius_and_canonical_center_survive_recording(tmp_path, radius):
    source = tmp_path / 'planner_nbv_filters.jsonl'
    center = [10., 20., -30.]
    jitter = [center[0] + radius * .75, center[1], center[2]]
    append(source, event('inspection_attempt_started', region_center=center,
                        matching_radius_m=radius, attempt=1),
           event('inspection_target_suppressed', center=jitter,
                 reason='attempt_budget_exhausted', exclusion_center=center, radius=radius))
    recorder = PermanentBanRecorder(tmp_path)
    recorder.poll()
    sphere = load_ban_spheres(recorder.output)[0]
    assert sphere['center'] == center and sphere['radius_m'] == radius
    assert target_is_banned(jitter, [sphere])
    assert not target_is_banned([center[0] + radius * 1.1, center[1], center[2]], [sphere])
