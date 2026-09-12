import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Find the parameter file
    bringup_dir = get_package_share_directory('bringup')
    config_file = os.path.join(bringup_dir, 'config', 'params.yaml')
    holo_config_file = os.path.join(bringup_dir, 'config', 'holoocean_config.json')

    return LaunchDescription([
        Node(
            package='sim_bridge',
            executable='holoocean_node',
            name='holoocean_node',
            parameters=[config_file, {'holoocean_config_path': holo_config_file}]
        ),
        Node(
            package='perception',
            executable='depth_estimation',
            name='depth_estimation',
            parameters=[config_file]
        ),
        Node(
            package='motion_control',
            executable='waypoint_control_node',
            name='waypoint_control_node',
            parameters=[config_file]
        ),
        Node(
            package='motion_control',
            executable='point_iterator',
            name='point_iterator',
            parameters=[config_file]
        ),
        Node(
            package='active_inspection_planner',
            executable='nbv_utility_node',
            name='nbv_utility_node',
            parameters=[config_file]
        ),
        Node(
            package='benchmark_evaluator',
            executable='run_baselines',
            name='run_baselines',
            parameters=[config_file]
        )
    ])