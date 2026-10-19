"""Deployment-owned translation of experiment overrides to the ROS YAML schema."""


def apply_trial_overrides(parameters, overrides, map_path, results_dir):
    planner = parameters['planner_node']['ros__parameters']
    for key, value in overrides.items():
        planner[key] = float(value)
    planner['octomap_path'] = str(map_path)
    planner['results_dir'] = str(results_dir)
    parameters['run_baselines']['ros__parameters']['results_dir'] = str(results_dir)
    if 'live_viewer' in parameters:
        parameters['live_viewer']['ros__parameters']['map_path'] = str(map_path)
    return parameters
