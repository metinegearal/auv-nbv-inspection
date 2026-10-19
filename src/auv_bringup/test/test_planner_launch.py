"""Check auv_bringup composition without starting the simulator or ROS nodes."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


def load_launch(filename, monkeypatch):
    package = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location('tested_launch', package / 'launch' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(package))
    return module.generate_launch_description()


def expand(description, context):
    nodes = []
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
        elif isinstance(action, OpaqueFunction):
            nodes.extend(action.execute(context))
        elif isinstance(action, Node):
            nodes.append(action)
    return nodes


@pytest.mark.parametrize('filename', ['mapper.launch.py', 'planner.launch.py'])
@pytest.mark.parametrize('implementation', [None, 'cpp', 'python'])
def test_bringup_selects_one_planner_and_keeps_parameter_precedence(
        filename, implementation, monkeypatch, tmp_path):
    description = load_launch(filename, monkeypatch)
    context = LaunchContext()
    nbv = tmp_path / 'nbv.yaml'
    main = tmp_path / 'main.yaml'
    nbv.write_text('planner_node:\n  ros__parameters:\n    nbv_standoff: 1.5\n')
    main.write_text('planner_node:\n  ros__parameters:\n    nbv_standoff: 2.0\n')
    context.launch_configurations.update(
        params_file=str(main), nbv_params_file=str(nbv), holo_config_file='/custom/scenario.json')
    if implementation is not None:
        context.launch_configurations['planner_implementation'] = implementation
    nodes = expand(description, context)
    chosen = implementation or 'cpp'
    assert context.launch_configurations['planner_implementation'] == chosen
    package, executable = (
        ('auv_inspection_planner', 'inspection_planner_node') if chosen == 'cpp'
        else ('auv_planner_reference', 'planner_node'))
    planners = [node for node in nodes
                if node.node_package in ('auv_inspection_planner', 'auv_planner_reference')]
    assert len(planners) == 1
    planner = planners[0]
    assert planner.node_package == package
    assert planner.node_executable == executable
    # Inspect normalized resources without executing the launch action.
    resources = evaluate_parameters(context, planner._Node__parameters)
    assert [str(path) for path in resources] == [str(nbv), str(main)]
    if filename == 'mapper.launch.py':
        assert len(nodes) == 7
        assert {node.node_package for node in nodes} == {
            package, 'auv_sim_bridge', 'auv_stereo_depth', 'auv_motion_control', 'auv_benchmark'}
    else:
        assert len(nodes) == 1


@pytest.mark.parametrize('filename', ['mapper.launch.py', 'planner.launch.py'])
def test_bringup_rejects_unknown_planner(filename, monkeypatch):
    description = load_launch(filename, monkeypatch)
    context = LaunchContext()
    context.launch_configurations['planner_implementation'] = 'invalid'
    with pytest.raises((RuntimeError, ValueError), match='planner_implementation'):
        expand(description, context)


@pytest.mark.parametrize('enabled', ['true', 'false'])
def test_live_viewer_can_be_disabled_without_affecting_planner(enabled, monkeypatch):
    context = LaunchContext()
    context.launch_configurations['live_viewer'] = enabled
    nodes = expand(load_launch('mapper.launch.py', monkeypatch), context)
    viewer = next(node for node in nodes if node.node_executable == 'live_viewer')
    assert viewer.node_package == 'auv_benchmark'
    assert viewer.condition.evaluate(context) == (enabled == 'true')
    assert len([node for node in nodes if node.node_package == 'auv_inspection_planner']) == 1


def test_standalone_viewer_uses_custom_bringup_parameters(monkeypatch):
    context = LaunchContext()
    context.launch_configurations['params_file'] = '/custom/viewer.yaml'
    nodes = expand(load_launch('live_viewer.launch.py', monkeypatch), context)
    assert len(nodes) == 1
    assert nodes[0].node_executable == 'live_viewer'
    assert [str(path) for path in evaluate_parameters(context, nodes[0]._Node__parameters)] == [
        '/custom/viewer.yaml']
