"""Run with /usr/bin/python3 -m pytest; no ROS, Torch or simulator startup."""
import ast
from contextlib import contextmanager
from dataclasses import FrozenInstanceError
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

SOURCE = Path(__file__).resolve().parents[3]
for package in ('auv_contracts', 'auv_stereo_depth', 'auv_planner_reference', 'auv_sim_bridge',
                'auv_benchmark', 'auv_experiments', 'auv_bringup'):
    sys.path.insert(0, str(SOURCE / package))

from auv_contracts.values import Configuration, CameraCalibration, Trajectory, ArrayEvent
from auv_contracts.mission_results import distance_record, distance_from_records
from auv_stereo_depth.use_cases.depth import StereoDepthUseCase
from auv_sim_bridge.use_cases.simulation import SimulationRunner
from auv_sim_bridge.domain.simulation import Command
from auv_sim_bridge.adapters.sensor_dispatch import SensorDispatch
from auv_benchmark.use_cases.map_evaluation import EvaluateMap
from auv_benchmark.use_cases.trajectory_evaluation import TrajectoryEvaluation
from auv_experiments.trial_policy import TrialObjective
from auv_bringup.configuration import apply_trial_overrides


class Diagnostics:
    def __init__(self):
        self.events = []

    def __getattr__(self, level):
        return lambda message, **options: self.events.append((level, message))


def test_configuration_and_trajectory_take_immutable_snapshots():
    original = {'nested': {'weights': [1., 2.]}}
    config = Configuration(original)
    original['nested']['weights'][0] = 99.
    assert config.nested['weights'] == (1., 2.)
    with pytest.raises(TypeError):
        config.nested['weights'] = ()
    points = [[1., 2., 3., 4.]]
    trajectory = Trajectory(points)
    points[0][0] = 99.
    assert trajectory.points == ((1., 2., 3., 4.),)
    with pytest.raises(FrozenInstanceError):
        trajectory.points = ()


def test_depth_keeps_invalid_disparity_mask_and_center_sample():
    disparity = np.array([[-2., 0., .1], [1., 2., 4.]], dtype=np.float32)
    values = iter([10., 10.3, 10.4])
    estimator = SimpleNamespace(estimate=lambda left, right, started:
                                (disparity.copy(), 100., 100., 10.2))
    use_case = StereoDepthUseCase(estimator, SimpleNamespace(perf_counter=lambda: next(values)))
    depth = use_case.compute('left', 'right', CameraCalibration(256., .5))
    np.testing.assert_array_equal(depth, [[0., 0., 0.], [128., 64., 32.]])
    timings = use_case.timings()
    assert timings['center_depth_m'] == 64.
    assert timings['postprocessing_ms'] == pytest.approx(100.)
    assert timings['total_compute_ms'] == pytest.approx(400.)
    timings.clear()
    assert use_case.timings()['center_depth_m'] == 64.


def test_simulation_preserves_step_dispatch_pump_order_and_limit():
    events = []
    command = Command(tuple(range(8)))

    class Sessions:
        @contextmanager
        def open(self):
            events.append('open')
            yield SimpleNamespace(step=lambda received:
                                  events.append(('step', received)) or {'sensor': 1})
            events.append('close')

    runner = SimulationRunner(Sessions(), SimpleNamespace(dispatch=lambda state, session:
                              events.append(('dispatch', state))), lambda: command,
                              lambda: events.append('pump'), Diagnostics(), step_limit=2)
    runner.run()
    assert events == ['open', ('step', command), ('dispatch', {'sensor': 1}), 'pump',
                      ('step', command), ('dispatch', {'sensor': 1}), 'pump', 'close']


def test_sensor_dispatch_preserves_order_and_continues_after_sensor_failure():
    events = []

    class Outputs:
        def __getattr__(self, name):
            def publish(value):
                events.append(name)
                if name == '_publish_dvl':
                    raise ValueError('failed sensor')
            return publish

    state = dict(FrontCamera=np.zeros((1, 1, 3)), DownCamera=np.zeros((1, 1, 3)),
                 LocationSensor=[1., 2., -3.])
    diagnostics = Diagnostics()
    sensors = SensorDispatch(Outputs(), diagnostics, lambda location: [4., 5., 6.])
    session = SimpleNamespace(set_ocean_currents=lambda agent, vector:
                              events.append((agent, vector)))
    sensors.dispatch(state, session)
    assert events == ['_publish_camera', '_publish_camera_down', '_publish_range_finder',
                      '_publish_dvl', '_publish_imu', '_publish_depth', '_publish_mag',
                      ('auv0', [4., 5., 6.]), '_publish_location']
    assert any('dvl failed' in message for level, message in diagnostics.events)


def test_map_scoring_keeps_strict_coverage_threshold():
    geometry = SimpleNamespace(prepare=lambda: None,
        map_distances=lambda: [.1, .3], ground_truth_distances=lambda: [.149, .15, .2],
        visualize=lambda distances: pytest.fail('headless evaluation must not visualize'))
    coverage, rmse = EvaluateMap(geometry, lambda message: None).execute(headless=True)
    assert coverage == pytest.approx(100. / 3.)
    assert rmse == pytest.approx(np.sqrt(.05))


def test_trajectory_metrics_and_save_remain_idempotent():
    writes = []
    evaluation = TrajectoryEvaluation(None, SimpleNamespace(write=lambda *args: writes.append(args)),
                                      SimpleNamespace(time=lambda: 10.), Diagnostics())
    evaluation.total_distance = 5.
    evaluation.diagnostic_log = [dict(planned_x_m=0., planned_y_m=0., planned_z_m=0.,
        x_m=3., y_m=4., z_m=0., heading_error_deg=-10., speed_mps=2.)]
    evaluation.save_benchmark_data(10.)
    evaluation.close()
    assert len(writes) == 1
    snapshot, elapsed = writes[0]
    assert snapshot.metrics['trajectory_rmse_m'] == 5.
    assert snapshot.metrics['mean_abs_heading_error_deg'] == 10.
    assert snapshot.metrics['peak_speed_mps'] == 2.
    evaluation.diagnostic_log[0]['x_m'] = 99.
    assert snapshot.diagnostic_log[0]['x_m'] == 3.
    assert elapsed == 10.


@pytest.mark.parametrize('distance,rmse,expected', [(7., .41, (40., 7.)), (7., .4, (80., 7.)),
                                                  (None, .1, (0., 999.))])
def test_trial_objective_preserves_penalties(distance, rmse, expected):
    sampled = []
    trial = SimpleNamespace(number=1, suggest_float=lambda name, low, high:
                            sampled.append((name, low, high)) or low)
    objective = TrialObjective(SimpleNamespace(run=lambda parameters: distance),
                               lambda **options: (80., rmse), lambda message: None)
    assert objective(trial) == expected
    assert len(sampled) == 6


def test_result_protocol_and_config_translation_keep_existing_wire_format():
    assert distance_record(12.345) == 'Total Distance Traveled: 12.35 meters\n'
    assert distance_from_records(['heading\n', distance_record(12.345)]) == 12.35
    assert distance_from_records([]) == 999.
    params = {'planner_node': {'ros__parameters': {'untouched': True}},
              'run_baselines': {'ros__parameters': {}}}
    assert apply_trial_overrides(params, {'nbv_standoff': 2}, 'map.bt', 'results') is params
    assert params['planner_node']['ros__parameters'] == {
        'untouched': True, 'nbv_standoff': 2., 'octomap_path': 'map.bt', 'results_dir': 'results'}
    assert params['run_baselines']['ros__parameters']['results_dir'] == 'results'


def test_python_policy_modules_have_no_framework_or_io_imports():
    forbidden = {'rclpy', 'rclcpp', 'sensor_msgs', 'nav_msgs', 'std_msgs', 'cv_bridge',
                 'torch', 'torchvision', 'open3d', 'pyoctomap', 'holoocean', 'cv2',
                 'matplotlib', 'subprocess', 'os', 'pathlib'}
    paths = list((SOURCE / 'auv_contracts/auv_contracts').glob('*.py'))
    for package in ('auv_planner_reference', 'auv_stereo_depth', 'auv_sim_bridge', 'auv_benchmark'):
        for layer in ('domain', 'use_cases'):
            paths.extend((SOURCE / package / package / layer).glob('*.py'))
    paths.append(SOURCE / 'auv_experiments/auv_experiments/trial_policy.py')
    for path in paths:
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.Import):
                names = [alias.name.split('.')[0] for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [(node.module or '').split('.')[0]]
                assert 'adapters' not in (node.module or '').split('.'), path
            else:
                continue
            assert not forbidden.intersection(names), path
