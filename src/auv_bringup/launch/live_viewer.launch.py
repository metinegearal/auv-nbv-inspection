"""Attach the benchmark viewer to an existing mapping run."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('auv_bringup'), 'config', 'params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        Node(package='auv_benchmark', executable='live_viewer', name='live_viewer',
             parameters=[LaunchConfiguration('params_file')], output='screen'),
    ])
