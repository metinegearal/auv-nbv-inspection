import optuna
import subprocess
import time
import os
import signal
import yaml

# Import your evaluator (we will modify it slightly to return values)
from benchmark_evaluator import evaluate_map_accuracy

def write_params_yaml(info_gain, sparsity, min_utility):
    """Writes the Optuna suggestions to a temporary ROS 2 params file."""
    params = {
        'nbv_utility_node': {
            'ros__parameters': {
                'info_gain_threshold': float(info_gain),
                'sparsity_multiplier': float(sparsity),
                'min_utility_threshold': float(min_utility),
                'voxel_resolution': 0.1,
                'max_depth': 4.0,
                'min_depth': 0.5,
                'z_min_target': -32.0
            }
        },
        'waypoint_control_node': {
            'ros__parameters': {
                'kp_yaw': 0.6,
                'kp_dist': 0.75,
                'max_yaw_thrust': 3.0,
                'absolute_max_motor_signal': 40.0,
                'yaw_deadband_deg': 15.0,
                'dvl_active': True
            }
        },
        'point_extract': {
            'ros__parameters': {
                'look_ahead_pixels': 100,
                'angle_weight': 0.5
            }
        }
    }
    with open('temp_optuna_params.yaml', 'w') as f:
        yaml.dump(params, f)

def objective(trial):
    # 1. Optuna suggests hyperparameter values
    # We know values > 0.18 cause immediate aborts.
    info_gain = trial.suggest_float('info_gain_threshold', 0.05, 0.17)
    
    # We know multipliers > 1.3 make frontiers impossible to find.
    sparsity = trial.suggest_float('sparsity_multiplier', 0.5, 1.2)
    
    # We know low utility (< 0.05) causes AUV to travel too far for bad points.
    min_utility = trial.suggest_float('min_utility_threshold', 0.05, 0.16)
    
    print(f"\n--- Starting Trial {trial.number} ---")
    print(f"Params: InfoGain={info_gain:.3f}, Sparsity={sparsity:.3f}, MinUtil={min_utility:.4f}")

    # 2. Write to YAML so ROS 2 can read it
    write_params_yaml(info_gain, sparsity, min_utility)

    # 3. Clean up old map files so we don't evaluate the previous run
    map_path = "/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/NBV_paper_ros2/final_map_ros2.bt"
    if os.path.exists(map_path):
        os.remove(map_path)
    if os.path.exists("optuna_mission_stats.txt"):
        os.remove("optuna_mission_stats.txt")

    # 4. Launch the ROS 2 system
    ws_path = "/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/NBV_paper_ros2"
    params_path = os.path.abspath("temp_optuna_params.yaml")
    
    # Chain the sourcing commands with the launch command
    cmd = (
        f"source /opt/ros/jazzy/setup.bash && "
        f"source {ws_path}/install/setup.bash && "
        f"ros2 launch bringup mapper.launch.py params_file:={params_path}"
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
        if os.path.exists("optuna_mission_stats.txt"):
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

    # 7. Evaluate Fitness
    if not mission_success:
        print("Trial timed out! Penalizing heavily.")
        return 0.0, 999.0 # 0% coverage, 999m distance

    # Read distance from the logger's text file
    total_distance = 999.0
    with open("optuna_mission_stats.txt", "r") as f:
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