import subprocess
import time
import signal
import os
from pathlib import Path
import yaml
from auv_bringup.configuration import apply_trial_overrides
from auv_contracts.mission_results import MISSION_STATS_FILENAME, distance_from_records


ROS_WORKSPACE = Path(__file__).resolve().parents[3]
BASE_PARAMS_PATH = ROS_WORKSPACE / 'src' / 'auv_bringup' / 'config' / 'params.yaml'
TEMP_PARAMS_PATH = ROS_WORKSPACE / 'temp_optuna_params.yaml'
MAP_PATH = ROS_WORKSPACE / 'results' / 'final_map_ros2.bt'
MISSION_STATS_PATH = ROS_WORKSPACE / 'results' / MISSION_STATS_FILENAME


def write_params_yaml(params_dict):
    """Write the current ROS config with the active planner overrides."""
    with BASE_PARAMS_PATH.open() as params_file:
        params = yaml.safe_load(params_file)

    apply_trial_overrides(params, params_dict, MAP_PATH, ROS_WORKSPACE / 'results')

    with TEMP_PARAMS_PATH.open('w') as params_file:
        yaml.safe_dump(params, params_file, sort_keys=False)


class RosTrialRunner:
    def run(self, params_dict):
        # Write to YAML so ROS 2 can read it
        write_params_yaml(params_dict)

        # Clean up old map files so we don't evaluate the previous run
        for output_path in (MAP_PATH, MISSION_STATS_PATH):
            output_path.unlink(missing_ok=True)

        # Launch the ROS 2 system
        cmd = (
            f"source /opt/ros/jazzy/setup.bash && "
            f"source {ROS_WORKSPACE}/install/setup.bash && "
            f"ros2 launch auv_bringup mapper.launch.py params_file:={TEMP_PARAMS_PATH}"
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
            return None 

        with MISSION_STATS_PATH.open() as f:
            return distance_from_records(f)
