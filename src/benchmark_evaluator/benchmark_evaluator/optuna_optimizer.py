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


def write_params_yaml(min_utility):
    """Write the current ROS config with the active planner override."""
    with BASE_PARAMS_PATH.open() as params_file:
        params = yaml.safe_load(params_file)

    planner_params = params['planner_node']['ros__parameters']
    planner_params['min_utility_threshold'] = float(min_utility)
    planner_params['octomap_path'] = str(MAP_PATH)
    planner_params['results_dir'] = str(ROS_WORKSPACE / 'results')
    params['run_baselines']['ros__parameters']['results_dir'] = str(
        ROS_WORKSPACE / 'results'
    )

    with TEMP_PARAMS_PATH.open('w') as params_file:
        yaml.safe_dump(params, params_file, sort_keys=False)

def objective(trial):
    # The active planner uses this threshold to reject low-value viewpoints.
    min_utility = trial.suggest_float('min_utility_threshold', 0.005, 0.16)

    print(f"\n--- Starting Trial {trial.number} ---")
    print(f"Params: MinUtil={min_utility:.4f}")

    # 2. Write to YAML so ROS 2 can read it
    write_params_yaml(min_utility)

    # 3. Clean up old map files so we don't evaluate the previous run
    for output_path in (MAP_PATH, MISSION_STATS_PATH):
        output_path.unlink(missing_ok=True)

    # 4. Launch the ROS 2 system
    # Chain the sourcing commands with the launch command
    cmd = (
        f"source /opt/ros/jazzy/setup.bash && "
        f"source {ROS_WORKSPACE}/install/setup.bash && "
        f"ros2 launch bringup mapper.launch.py params_file:={TEMP_PARAMS_PATH}"
    )

    print("Launching ROS 2 stack...")
    # preexec_fn=os.setsid creates a new process group so we can kill ALL child nodes cleanly
    process = subprocess.Popen(
        cmd,
        shell=True,
        executable='/bin/bash',
        preexec_fn=os.setsid
    )

    # 5. Wait for the mission to finish (Timeout after 300 seconds)
    timeout = 300 
    start_time = time.time()
    mission_success = False

    while (time.time() - start_time) < timeout:
        if MISSION_STATS_PATH.exists():
            mission_success = True
            break
        time.sleep(2.0)

    # 6. Kill the ENTIRE process group cleanly
    print("Terminating simulation run...")
    try:
        # Send SIGINT to the entire process group (bash + ros2 launch + nodes)
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        # Force kill if any node hangs on shutdown
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.wait()
    except ProcessLookupError:
        pass # Process already exited

    time.sleep(3.0) # Settle simulation before the next trial
    # The benchmark logger saves partial results during graceful shutdown.
    mission_success = mission_success or MISSION_STATS_PATH.exists()

    # 7. Evaluate Fitness
    if not mission_success:
        print("Trial timed out before results were saved. Penalizing heavily.")
        return 0.0, 999.0 # 0% coverage, 999m distance

    # Read distance from the logger's text file
    total_distance = 999.0
    with MISSION_STATS_PATH.open() as f:
        for line in f:
            if "Total Distance Traveled:" in line:
                total_distance = float(line.split(":")[1].replace("meters", "").strip())

    # Run the evaluator in Headless Mode (returns floats instead of opening GUI)
    # Make sure to update your evaluate_map_accuracy to return (coverage_percentage, rmse)
    coverage, rmse = evaluate_map_accuracy(headless=True)
    
    # If RMSE is terrible (> 0.4m), penalize the coverage heavily so Optuna learns not to do this
    if rmse > 0.4:
        coverage = coverage * 0.5 

    print(f"Trial {trial.number} Results -> Coverage: {coverage:.2f}%, Distance: {total_distance:.2f}m")
    
    # Return Multi-Objective: Maximize Coverage, Minimize Distance
    return coverage, total_distance


if __name__ == "__main__":
    # Define the name of the database file and the study name
    study_name = "auv_nbv_optimization"
    storage_name = "sqlite:///optuna_auv_study.db"
    
    # Create or load the study using the SQLite database
    study = optuna.create_study(
        study_name=study_name, 
        storage=storage_name, 
        directions=["maximize", "minimize"], 
        load_if_exists=True  # Resumes where it left off if you restart the script
    )
    
    print(f"\nOptimization results will be saved to: {storage_name}")
    
    # Run 60 trials
    study.optimize(objective, n_trials=60)
    
    print("\n=======================================")
    print("Optimization Complete!")
    print("Best Trials on the Pareto Front:")
    for trial in study.best_trials:
        print(f"Trial {trial.number}: Coverage {trial.values[0]:.2f}%, Distance {trial.values[1]:.2f}m")
        print(f"  Params: {trial.params}")
        
    # Optional: You can also export the final results to a CSV
    import pandas as pd
    df = study.trials_dataframe()
    df.to_csv("optuna_results_summary.csv")
    print("Saved summary to optuna_results_summary.csv")