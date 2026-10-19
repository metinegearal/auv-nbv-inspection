"""Use the existing bringup description with a selectable inspection planner."""
import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    implementation = LaunchConfiguration('planner_implementation').perform(context)
    if implementation not in ('python', 'cpp'):
        raise ValueError('planner_implementation must be python or cpp')
    path = os.path.join(get_package_share_directory('bringup'), 'launch', 'mapper.launch.py')
    spec = importlib.util.spec_from_file_location('_reference_mapper_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    actions = []
    replaced = 0
    for action in description.entities:
        if isinstance(action, Node) and action.node_package == 'python_planner':
            replaced += 1
            action = Node(
                package='active_inspection_planner' if implementation == 'cpp' else 'python_planner',
                executable='inspection_planner_node' if implementation == 'cpp' else 'planner_node',
                name='planner_node',
                parameters=[LaunchConfiguration('nbv_params_file'), LaunchConfiguration('params_file')])
        actions.append(action)
    if replaced != 1:
        raise RuntimeError(f'Expected one reference planner in bringup; found {replaced}')
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('planner_implementation', default_value='cpp',
                              choices=['python', 'cpp'],
                              description='Select implementation for differential validation'),
        OpaqueFunction(function=_setup),
    ])
