"""Verify real mission strike policy through repeated planning passes."""
import io
import time
from types import SimpleNamespace

import numpy as np
import pytest

from python_planner.domain.configuration import load_configuration
from python_planner.use_cases.mission import InspectionMission
from python_planner.domain.nbv_parameters import MIN_NBV_CLOUD_POINTS


class Sink:
    def __getattr__(self, name):
        return lambda *args, **kwargs: None


def mission(point_count=MIN_NBV_CLOUD_POINTS):
    overrides = {'planning_period': 1, 'nbv_filter_logging': False}
    config = load_configuration(lambda name, default: overrides.get(name, default))
    planner = InspectionMission(
        config, Sink(), Sink(), io.StringIO(), Sink(), time, Sink(),
        lambda *args, **kwargs: [], lambda *args, **kwargs: None, 'test',
        SimpleNamespace(camera_pose=lambda: (np.zeros(3), np.array([1., 0., 0.]))))
    planner.odom_received = planner.rgb_received = True
    planner.depth_frame = np.ones((2, 2))
    planner._accumulate_depth = lambda: None
    planner._occupied_points = lambda: np.column_stack((
        np.arange(point_count, dtype=float), np.zeros(point_count), np.ones(point_count)))
    return planner


def tick(planner):
    planner.depth_sequence += 1
    planner.timer_callback()


def test_strike_three_finishes_mission_and_preserves_abort_reason():
    planner = mission()
    planner.planning_enabled = True
    for strike in range(1, 4):
        tick(planner)
        assert planner.failed_plan_count == strike
        assert planner.finished_mapping is (strike == 3)
    assert planner.failure_reason == 'nbv_generation_returned_no_candidates'
    assert planner.active_trajectory.shape == (0, 4)
    tick(planner)
    assert planner.failed_plan_count == 3


def test_mapping_without_an_action_does_not_consume_strikes():
    planner = mission()
    for _ in range(4):
        tick(planner)
    assert planner.failed_plan_count == 0
    assert not planner.finished_mapping


@pytest.mark.parametrize('point_count', [0, 1, MIN_NBV_CLOUD_POINTS - 1])
def test_startup_waits_for_a_usable_map_without_strikes(point_count):
    planner = mission(point_count)
    planner.planning_enabled = True
    planner.frame_period = 100
    calls = []
    planner.nbv_calculator = lambda *args, **kwargs: calls.append(args) or []
    for _ in range(5):
        tick(planner)
    assert not planner.finished_mapping and not planner.map_ready
    assert planner.failed_plan_count == 0
    assert planner.planning_state == 'WaitingForPointCloud'
    assert planner.force_replan
    assert calls == []
    # The pending action starts planning on the next sufficient frame, even
    # though the periodic planning interval has not elapsed.
    planner._occupied_points = mission()._occupied_points
    tick(planner)
    assert planner.map_ready and planner.failed_plan_count == 1
    assert len(calls) == 1


@pytest.mark.parametrize('sensor,state', [('odom_received', 'WaitingForOdometry'),
                                        ('rgb_received', 'WaitingForRGB'),
                                        ('depth_frame', 'WaitingForDepth')])
def test_missing_startup_sensor_reports_waiting_without_strikes(sensor, state):
    planner = mission()
    planner.planning_enabled = True
    setattr(planner, sensor, None if sensor == 'depth_frame' else False)
    for _ in range(4):
        tick(planner)
    assert planner.planning_state == state
    assert planner.failed_plan_count == 0 and not planner.finished_mapping
