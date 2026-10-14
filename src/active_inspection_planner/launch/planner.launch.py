"""Run only the selected planner with the same parameter precedence as bringup."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    implementation = LaunchConfiguration('planner_implementation').perform(context)
    return [Node(package='active_inspection_planner' if implementation == 'cpp' else 'python_planner',
                 executable='inspection_planner_node' if implementation == 'cpp' else 'planner_node',
                 name='planner_node', output='screen',
                 parameters=[LaunchConfiguration('nbv_params_file'), LaunchConfiguration('params_file')])]


def generate_launch_description():
    config = os.path.join(get_package_share_directory('bringup'), 'config')
    return LaunchDescription([
        DeclareLaunchArgument('planner_implementation', default_value='cpp', choices=['python', 'cpp']),
        DeclareLaunchArgument('params_file', default_value=os.path.join(config, 'params.yaml')),
        DeclareLaunchArgument('nbv_params_file', default_value=os.path.join(config, 'nbv_params.yaml')),
        OpaqueFunction(function=_setup),
    ])
