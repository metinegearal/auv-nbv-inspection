"""Run only the selected planner with the same parameter precedence as auv_bringup."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from auv_bringup.planner_launch import launch_planner, planner_implementation_argument


def generate_launch_description():
    config = os.path.join(get_package_share_directory('auv_bringup'), 'config')
    return LaunchDescription([
        planner_implementation_argument(),
        DeclareLaunchArgument('params_file', default_value=os.path.join(config, 'params.yaml')),
        DeclareLaunchArgument('nbv_params_file', default_value=os.path.join(config, 'nbv_params.yaml')),
        OpaqueFunction(function=launch_planner, kwargs={'output': 'screen'}),
    ])
