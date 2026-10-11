import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from bringup.transforms import camera_transform_arguments


def camera_transform_node(context, default_file, params_file, enabled):
    # The standalone publisher requires CLI frame IDs and overrides YAML
    # parameters internally, so resolve the configured mount into CLI flags.
    return [Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='camera_static_transform', condition=IfCondition(enabled),
        arguments=camera_transform_arguments(default_file, params_file.perform(context)))]

def generate_launch_description():
    # 1. Set up the default paths (used if no arguments are passed)
    bringup_dir = get_package_share_directory('bringup')
    default_config_file = os.path.join(bringup_dir, 'config', 'params.yaml')
    default_nbv_config_file = os.path.join(bringup_dir, 'config', 'nbv_params.yaml')
    default_holo_config_file = os.path.join(bringup_dir, 'config', 'holoocean_config.json')

    # 2. Create LaunchConfigurations to hold the runtime values
    params_file = LaunchConfiguration('params_file')
    nbv_params_file = LaunchConfiguration('nbv_params_file')
    holo_config = LaunchConfiguration('holo_config_file')
    publish_sim_tf = LaunchConfiguration('publish_sim_tf')
    publish_camera_tf = LaunchConfiguration('publish_camera_tf')

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
        DeclareLaunchArgument('publish_sim_tf', default_value='true',
                              description='Bridge simulator odometry to TF with identity map to odom'),
        DeclareLaunchArgument('publish_camera_tf', default_value='true',
                              description='Publish configured camera mount; disable for URDF/dynamic TF'),

        # HoloOcean odometry is already in global map coordinates.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='map_to_odom', condition=IfCondition(publish_sim_tf),
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom']
        ),
        OpaqueFunction(function=camera_transform_node,
                       args=[default_config_file, params_file, publish_camera_tf]),
        
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
        Node(
            package='python_planner',
            executable='planner_node',
            name='planner_node',
            # Keep legacy combined/custom params_file overrides authoritative.
            parameters=[{'publish_vehicle_tf': ParameterValue(publish_sim_tf, value_type=bool)},
                        nbv_params_file, params_file]
        ),
        Node(
            package='benchmark_evaluator',
            executable='run_baselines',
            name='run_baselines',
            parameters=[params_file]
        )
    ])
