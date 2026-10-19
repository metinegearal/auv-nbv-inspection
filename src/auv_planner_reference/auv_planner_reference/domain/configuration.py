from auv_contracts.values import Configuration
from .nbv_parameters import DEFAULT_NBV_PARAMETERS

def load_configuration(read_parameter):
    values = {}
    values['resolution'] = read_parameter('voxel_resolution', 0.1)
    values['max_depth'] = read_parameter('max_depth', 3.0)
    values['min_depth'] = read_parameter('min_depth', 0.5)
    values['z_min_target'] = read_parameter(
        'z_min_target', -3000.0)
    values['depth_stride'] = read_parameter('depth_stride', 8)
    values['depth_scale'] = read_parameter('depth_scale', 1.0)
    values['frame_period'] = read_parameter('planning_period', 10)
    values['min_utility'] = read_parameter(
        'min_utility_threshold', 0.005)
    values['route_max_candidates'] = read_parameter('route_max_candidates', 5)
    candidate_limit = values['route_max_candidates']
    if (isinstance(candidate_limit, bool) or not isinstance(candidate_limit, int)
            or candidate_limit < 1):
        raise ValueError('route_max_candidates must be a positive integer')
    values['collision_radius'] = read_parameter(
        'collision_radius', 0.6)
    values['collision_tracking_margin'] = read_parameter(
        'collision_tracking_margin', 0.4)
    values['route_stall_timeout_s'] = read_parameter('route_stall_timeout_s', 10.0)
    values['route_stall_min_progress_m'] = read_parameter('route_stall_min_progress_m', 0.15)
    for name in ('route_stall_timeout_s', 'route_stall_min_progress_m'):
        if not 0.0 < float(values[name]) < float('inf'):
            raise ValueError(f'{name} must be finite and positive')
    values['blacklist_cooldown_s'] = read_parameter(
        'route_blacklist_cooldown_s', 30.0)
    values['trajectory_points'] = read_parameter(
        'trajectory_points', 300)
    values['waypoint_spacing'] = read_parameter(
        'waypoint_spacing', 0.5)
    values['map_path'] = read_parameter(
        'octomap_path', 'results/final_map_ros2.bt')
    values['visit_radius'] = read_parameter('visit_radius', 0.25)
    values['results_dir'] = read_parameter(
        'results_dir', 'results')
    values['nbv_filter_logging'] = read_parameter(
        'nbv_filter_logging', True)
    values['_nbv_empty_reason'] = None
    values['_plan_failure_reason'] = None
    values['use_incremental_map_cache'] = read_parameter(
        'use_incremental_map_cache', True)
    values['nbv_parameters'] = {
        name: read_parameter(
            f'nbv_{name}', default)
        for name, default in DEFAULT_NBV_PARAMETERS.items()
        if name != 'z_min_target'
    }
    # The baseline NBV function filtered at -31.5 independently of mapping.
    values['nbv_parameters']['z_min_target'] = read_parameter(
        'nbv_z_min_target', DEFAULT_NBV_PARAMETERS['z_min_target'])
    values['astar_max_steps'] = read_parameter(
        'astar_max_steps', 50000)
    values['spline_smoothing'] = read_parameter(
        'spline_smoothing', 2.0)
    values['collision_check_spacing_factor'] = read_parameter(
        'collision_check_spacing_factor', 0.5)
    values['arrival_target_radius'] = read_parameter(
        'arrival_target_radius', 1.5)
    values['replan_on_target_arrival'] = read_parameter(
        'replan_on_target_arrival', False)
    values['optimize_motion'] = read_parameter(
        'trajectory_optimization_enabled', False)
    values['velocity_history_window'] = read_parameter(
        'velocity_history_window', 1.0)
    values['trajectory_history_window'] = read_parameter(
        'trajectory_history_window', 5.0)
    values['motion_cost_parameters'] = {
        name: read_parameter(f'trajectory_{name}', default)
        for name, default in {
            'continuity_weight': 8.0, 'reversal_weight': 4.0,
            'history_weight': 2.0, 'history_radius': 0.5,
        }.items()
    }
    values['optimizer_max_deviation'] = read_parameter(
        'trajectory_max_deviation', 0.5)
    values['optimizer_max_iterations'] = read_parameter(
        'trajectory_optimizer_max_iterations', 40)

    return Configuration(values)
