import optuna
import subprocess
import time
import signal
import os
from pathlib import Path
import yaml

from benchmark_evaluator import evaluate_map_accuracy

ROS_WORKSPACE = Path(__file__).resolve().parents[3]
BASE_PARAMS_PATH = ROS_WORKSPACE / 'src' / 'bringup' / 'config' / 'params.yaml'
TEMP_PARAMS_PATH = ROS_WORKSPACE / 'temp_optuna_params.yaml'
MAP_PATH = ROS_WORKSPACE / 'results' / 'final_map_ros2.bt'
MISSION_STATS_PATH = ROS_WORKSPACE / 'results' / 'optuna_mission_stats.txt'


def write_params_yaml(params_dict):
    """Write the current ROS config with the active planner overrides."""
    with BASE_PARAMS_PATH.open() as params_file:
        params = yaml.safe_load(params_file)

    planner_params = params['planner_node']['ros__parameters']
    
    # Dynamically inject all Optuna suggestions into the ROS 2 params
    for key, value in params_dict.items():
        planner_params[key] = float(value)

    planner_params['octomap_path'] = str(MAP_PATH)
    planner_params['results_dir'] = str(ROS_WORKSPACE / 'results')
    params['run_baselines']['ros__parameters']['results_dir'] = str(
        ROS_WORKSPACE / 'results'
    )

    with TEMP_PARAMS_PATH.open('w') as params_file:
        yaml.safe_dump(params, params_file, sort_keys=False)

def objective(trial):
    # Sample all 6 parameters based on the requested ranges
    params_dict = {
        'min_utility_threshold': trial.suggest_float('min_utility_threshold', 0.05, 0.35),
        'nbv_standoff': trial.suggest_float('nbv_standoff', 1.0, 2.5),
        'nbv_distance_weight': trial.suggest_float('nbv_distance_weight', 0.5, 2.0),
        'nbv_rotation_weight': trial.suggest_float('nbv_rotation_weight', 0.1, 1.0),
        'nbv_collision_weight': trial.suggest_float('nbv_collision_weight', 1.0, 5.0),
        'nbv_min_raw_gain': trial.suggest_float('nbv_min_raw_gain', 0.2, 1.0),
    }

    print(f"\n--- Starting Trial {trial.number} ---")
    print(f"Params: {params_dict}")

    # Write to YAML so ROS 2 can read it
    write_params_yaml(params_dict)

    # Clean up old map files so we don't evaluate the previous run
    for output_path in (MAP_PATH, MISSION_STATS_PATH):
        output_path.unlink(missing_ok=True)

    # Launch the ROS 2 system
    cmd = (
        f"source /opt/ros/jazzy/setup.bash && "
        f"source {ROS_WORKSPACE}/install/setup.bash && "
        f"ros2 launch bringup mapper.launch.py params_file:={TEMP_PARAMS_PATH}"
    )

    print("Launching ROS 2 stack...")
    process = subprocess.Popen(
        cmd,
        shell=True,
        executable='/bin/bash',
        preexec_fn=os.setsid
    )

    # Wait for the mission to finish (Timeout after 300 seconds)
    timeout = 300 
    start_time = time.time()
    mission_success = False

    while (time.time() - start_time) < timeout:
        if MISSION_STATS_PATH.exists():
            mission_success = True
            break
        time.sleep(2.0)

    # Kill the ENTIRE process group cleanly
    print("Terminating simulation run...")
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.wait()
    except ProcessLookupError:
        pass 

    time.sleep(3.0) 
    mission_success = mission_success or MISSION_STATS_PATH.exists()

    # Evaluate Fitness
    if not mission_success:
        print("Trial timed out before results were saved. Penalizing heavily.")
        return 0.0, 999.0 

    total_distance = 999.0
    with MISSION_STATS_PATH.open() as f:
        for line in f:
            if "Total Distance Traveled:" in line:
                total_distance = float(line.split(":")[1].replace("meters", "").strip())

    coverage, rmse = evaluate_map_accuracy(headless=True)
    
    if rmse > 0.4:
        coverage = coverage * 0.5 

    print(f"Trial {trial.number} Results -> Coverage: {coverage:.2f}%, Distance: {total_distance:.2f}m")
    return coverage, total_distance


if __name__ == "__main__":
    study_name = "auv_nbv_optimization"
    
    # Robust storage with a 20-second timeout queue to prevent SQLite locks
    storage = optuna.storages.RDBStorage(
        url="sqlite:///optuna_auv_study.db",
        engine_kwargs={"connect_args": {"timeout": 20.0}}
    )
    
    study = optuna.create_study(
        study_name=study_name, 
        storage=storage, 
        directions=["maximize", "minimize"], 
        load_if_exists=True 
    )
    
    print(f"\nOptimization results will be saved to: sqlite:///optuna_auv_study.db")
    study.optimize(objective, n_trials=60)
    
    print("\n=======================================")
    print("Optimization Complete!")
    print("Best Trials on the Pareto Front:")
    for trial in study.best_trials:
        print(f"Trial {trial.number}: Coverage {trial.values[0]:.2f}%, Distance {trial.values[1]:.2f}m")
        print(f"  Params: {trial.params}")
        
    import pandas as pd
    df = study.trials_dataframe()
    df.to_csv("optuna_results_summary.csv")
    print("Saved summary to optuna_results_summary.csv")