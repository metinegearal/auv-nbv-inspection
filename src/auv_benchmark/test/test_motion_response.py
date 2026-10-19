"""Verify measured response semantics and evaluator report integration."""
import csv
import math
from types import SimpleNamespace

from auv_contracts.values import ArrayEvent, Flag, PoseSample, Quaternion, Stamp, Vector3
from auv_benchmark.adapters.reports import FileReportWriter
from auv_benchmark.domain.response import ResponseMonitor
from auv_benchmark.use_cases.motion_response import MotionResponseEvaluation
from auv_benchmark.use_cases.trajectory_evaluation import TrajectoryEvaluation
import numpy as np
import pytest


class Clock:
    now = 0.0

    def time(self):
        return self.now

    def perf_counter(self):
        return self.now


class Diagnostics:

    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)

    def warning(self, message):
        self.messages.append(message)


def pose(x, y=0., z=0.):
    return PoseSample(Vector3(x, y, z), Quaternion(0., 0., 0., 1.), Stamp(0, 0))


def test_first_order_reference_has_expected_measured_times():
    monitor = ResponseMonitor([0., 0., 0.], [1., 0., 0.])
    for elapsed in np.arange(.01, 6.01, .01):
        monitor.update(float(elapsed), [1 - math.exp(-elapsed), 0., 0.])
    values = monitor.metrics()
    assert values['rise_time_s'] == pytest.approx(math.log(9), abs=1e-4)
    assert values['t63_s'] == pytest.approx(1., abs=1e-4)
    assert values['settling_time_s'] == pytest.approx(-math.log(.02), abs=.01)
    assert values['settling_dwell_met']
    assert values['overshoot_pct'] == 0.
    assert values['peak_time_s'] is None
    assert values['iae_m_s'] == pytest.approx(1 - math.exp(-6), abs=1e-4)
    assert values['ise_m2_s'] == pytest.approx((1 - math.exp(-12)) / 2, abs=1e-4)


def test_settling_requires_full_error_and_is_revoked_on_exit():
    monitor = ResponseMonitor([0., 0., 0.], [1., 0., 0.])
    monitor.update(1., [1., .1, 0.])
    assert monitor.metrics()['settling_time_s'] is None
    monitor.update(2., [1., 0., 0.])
    monitor.update(3., [1., 0., 0.])
    assert monitor.metrics()['settling_time_s'] == 2.
    assert monitor.metrics()['settling_dwell_met']
    monitor.update(4., [1.1, 0., 0.])
    values = monitor.metrics()
    assert values['settling_time_s'] is None
    assert not values['settling_dwell_met']
    assert values['overshoot_pct'] == pytest.approx(10.)
    assert values['peak_time_s'] == 4.
    monitor.update(5., [1., 0., 0.])
    assert monitor.metrics()['settling_time_s'] == 5.
    assert not monitor.metrics()['settling_dwell_met']


def test_bounded_history_keeps_crossings_and_integrals():
    monitor = ResponseMonitor([0., 0., 0.], [10., 0., 0.], max_samples=2)
    monitor.update(1., [5., 0., 0.])
    monitor.update(2., [10., 0., 0.])
    monitor.update(3., [10., 0., 0.])
    values = monitor.metrics()
    assert values['rise_time_s'] == pytest.approx(1.6)
    assert values['iae_m_s'] == 10.
    assert values['ise_m2_s'] == 75.
    assert values['retained_samples'] == 2
    assert values['sample_count'] == 4


def test_zero_step_and_unreached_crossings_are_not_zero_response_times():
    monitor = ResponseMonitor([1., 2., 3.], [1., 2., 3.])
    assert not monitor.metrics()['valid_step']
    for key in ('rise_time_s', 't63_s', 'settling_time_s', 'overshoot_pct', 'peak_time_s'):
        assert monitor.metrics()[key] is None
    monitor = ResponseMonitor([0., 0., 0.], [1., 0., 0.])
    monitor.update(1., [.2, 0., 0.])
    assert monitor.metrics()['rise_time_s'] is None
    assert monitor.metrics()['t63_s'] is None


def test_reversed_or_invalid_samples_do_not_mutate_monitor():
    monitor = ResponseMonitor([0., 0., 0.], [1., 0., 0.])
    monitor.update(1., [.5, 0., 0.])
    previous = monitor.metrics()
    for elapsed, position in ((.5, [.5, 0., 0.]), (2., [np.nan, 0., 0.]),
                              (np.inf, [.5, 0., 0.])):
        with pytest.raises(ValueError):
            monitor.update(elapsed, position)
        assert monitor.metrics() == previous


def test_actual_target_deduplication_replanning_and_pending_odometry():
    clock, diagnostics = Clock(), Diagnostics()
    evaluator = MotionResponseEvaluation(clock, diagnostics)
    evaluator.waypoint_callback(ArrayEvent((1., 0., 0., .5, 99., 99., 99., 0.)))
    assert evaluator.monitor is None
    evaluator.odom_callback(pose(0.))
    assert evaluator.episode_id == 1
    clock.now = 1.
    evaluator.odom_callback(pose(.5))
    evaluator.waypoint_callback(ArrayEvent((1., 0., 0., 1.5)))
    assert evaluator.episode_id == 1  # Yaw-only change is not a position step.
    evaluator.waypoint_callback(ArrayEvent((2., 0., 0., 0.)))
    assert evaluator.episode_id == 2
    assert evaluator.records[0]['end_reason'] == 'target_changed'
    assert evaluator.records[0]['rise_time_s'] is None
    assert evaluator.monitor.initial_distance == 1.5
    evaluator.waypoint_callback(ArrayEvent((np.nan, 0., 0.)))
    assert evaluator.episode_id == 2
    evaluator.close('mission_finished')
    evaluator.close()
    evaluator.odom_callback(pose(10.))
    assert len(evaluator.records) == 2
    assert any('tr(10-90%)' in message for message in diagnostics.messages)


def test_every_odometry_sample_reaches_saved_snapshot_and_reports(tmp_path, monkeypatch):
    monkeypatch.setenv('MPLBACKEND', 'Agg')
    clock, diagnostics, writes = Clock(), Diagnostics(), []
    evaluation = TrajectoryEvaluation(
        SimpleNamespace(record_trajectory=lambda *args: None),
        SimpleNamespace(write=lambda *args: writes.append(args)), clock, diagnostics)
    evaluation.odom_callback(pose(0.))
    evaluation.motion_response.waypoint_callback(ArrayEvent((1., 0., 0., 0.)))
    for elapsed, x in ((.1, .1), (.5, .5), (.9, .9), (1., 1.), (2., 1.)):
        clock.now = elapsed
        evaluation.odom_callback(pose(x))
    # The separate 0.5 s trajectory logger never ran, but the response is sampled.
    evaluation.finish_callback(Flag(True))
    evaluation.close()
    assert len(writes) == 1
    snapshot, total_time = writes[0]
    assert snapshot.response_log[0]['rise_time_s'] == pytest.approx(.8)
    assert snapshot.response_log[0]['settling_dwell_met']
    assert len(snapshot.response_samples) == 6
    evaluation.motion_response.records[0]['final_error_m'] = 99.
    assert snapshot.response_log[0]['final_error_m'] == 0.
    FileReportWriter(tmp_path / 'nested', diagnostics).write(snapshot, total_time)
    output = tmp_path / 'nested'
    with (output / 'motion_response_metrics.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    assert float(rows[0]['rise_time_s']) == pytest.approx(.8)
    assert rows[0]['peak_time_s'] == ''
    assert rows[0]['end_reason'] == 'mission_finished'
    assert (output / 'motion_control_response.png').stat().st_size > 0
    assert 'provisional' in (output / 'motion_response_summary.txt').read_text()


def test_shutdown_without_targets_exports_empty_reports(tmp_path):
    evaluation = TrajectoryEvaluation(
        None, FileReportWriter(tmp_path, Diagnostics()), Clock(), Diagnostics())
    evaluation.close()
    with (tmp_path / 'motion_response_metrics.csv').open() as stream:
        assert list(csv.DictReader(stream)) == []
    assert not (tmp_path / 'motion_control_response.png').exists()
