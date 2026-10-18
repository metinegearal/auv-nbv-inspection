import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from bringup.planner_launch import launch_planner, planner_implementation_argument

def generate_launch_description():
    # 1. Set up the default paths (used if no arguments are passed)
    bringup_dir = get_package_share_directory('bringup')
    default_config_file = os.path.join(bringup_dir, 'config', 'params.yaml')
    default_nbv_config_file = os.path.join(bringup_dir, 'config', 'nbv_params.yaml')
    default_holo_config_file = os.path.join(bringup_dir, 'config', 'holoocean_config_sub.json')

    # 2. Create LaunchConfigurations to hold the runtime values
    params_file = LaunchConfiguration('params_file')
    holo_config = LaunchConfiguration('holo_config_file')

    # 3. Declare the launch arguments so they are exposed to the CLI
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_config_file,
        description='Full path to the ROS 2 parameters YAML file to use'
    )

    declare_holo_config_cmd = DeclareLaunchArgument(
        'holo_config_file',
        default_value=default_holo_config_file,
        description='Full path to the HoloOcean configuration JSON file to use'
    )
    declare_nbv_params_file_cmd = DeclareLaunchArgument(
        'nbv_params_file',
        default_value=default_nbv_config_file,
        description='Full path to NBV geometry, sampling, and scoring parameters YAML'
    )

    return LaunchDescription([
        # Add the declarations to the LaunchDescription first
        declare_params_file_cmd,
        declare_nbv_params_file_cmd,
        declare_holo_config_cmd,
        planner_implementation_argument(),
        DeclareLaunchArgument('live_viewer', default_value='true',
                              description='Open the live map and planned trajectory viewer'),
        
        # Now pass the LaunchConfigurations into the nodes instead of the hardcoded strings
        Node(
            package='sim_bridge',
            executable='holoocean_node',
            name='holoocean_node',
            parameters=[params_file, {'holoocean_config_path': holo_config}]
        ),
        Node(
            package='perception',
            executable='depth_estimation',
            name='depth_estimation',
            parameters=[params_file]
        ),
        Node(
            package='motion_control',
            executable='waypoint_control_node',
            name='waypoint_control_node',
            parameters=[params_file]
        ),
        Node(
            package='motion_control',
            executable='point_iterator',
            name='point_iterator',
            parameters=[params_file]
        ),
        OpaqueFunction(function=launch_planner),
        Node(
            package='benchmark_evaluator',
            executable='run_baselines',
            name='run_baselines',
            parameters=[params_file]
        ),
        Node(
            package='benchmark_evaluator', executable='live_viewer', name='live_viewer',
            parameters=[params_file], condition=IfCondition(LaunchConfiguration('live_viewer')),
            output='screen'
        ),
    ])
