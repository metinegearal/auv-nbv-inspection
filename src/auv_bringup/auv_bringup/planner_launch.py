"""Compose either inspection planner using auv_bringup-owned parameters."""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def planner_implementation_argument():
    return DeclareLaunchArgument(
        'planner_implementation', default_value='cpp', choices=['cpp', 'python'],
        description='Inspection planner implementation; Python is the migration reference')


def launch_planner(context, output=None):
    implementation = LaunchConfiguration('planner_implementation').perform(context)
    implementations = {
        'cpp': ('auv_inspection_planner', 'inspection_planner_node'),
        'python': ('auv_planner_reference', 'planner_node'),
    }
    if implementation not in implementations:
        raise ValueError('planner_implementation must be cpp or python')
    package, executable = implementations[implementation]
    options = {} if output is None else {'output': output}
    return [Node(
        package=package, executable=executable, name='planner_node',
        # Keep main/custom configuration overrides authoritative.
        parameters=[LaunchConfiguration('nbv_params_file'), LaunchConfiguration('params_file')],
        **options)]
