# import sys
# sys.path.append()

import math
import numpy as np
from scipy.interpolate import splprep, splev

from Tools.search_algo import astar_3d

def rotate_around_xy_line(point, angle):
    x, y, z = point
    x=np.sqrt(x**2+y**2)
    y=z
    angle_rad = math.radians(angle)
    rotatedX = x * np.cos(angle_rad) - y * np.sin(angle_rad)
    rotatedY = x * np.sin(angle_rad) + y * np.cos(angle_rad)
    pointX=point[0]*(rotatedX/x)
    pointY=point[1]*(rotatedX/x)
    pointZ=z+rotatedY
    return (pointX, pointY, pointZ)

def plan_multi_waypoint_route(current_location, candidates, occ_set, res):
    """
    1. Sequences the viewpoints using Greedy Nearest-Neighbor.
    2. Runs A* between each point.
    3. Returns the concatenated path and the raw target focus for each point.
    """
    if not candidates:
        return [], []

    unvisited = candidates.copy()
    ordered_candidates = []
    curr_pos = current_location
    
    # 1. Sequence the viewpoints (Greedy TSP)
    while unvisited:
        # Find the closest next viewpoint using Euclidean distance (fast proxy for A*)
        best_idx = np.argmin([np.linalg.norm(curr_pos - c[0]) for c in unvisited])
        best_candidate = unvisited.pop(best_idx)
        ordered_candidates.append(best_candidate)
        curr_pos = best_candidate[0]

    # 2. Build the continuous A* route
    full_path = []
    target_tracking = [] # Keeps track of what the camera should look at
    
    start_pos = current_location
    for vp, target_frontier in ordered_candidates:
        segment = astar_3d(start_pos, vp, occ_set, res)
        
        if not segment:
            print("Warning: A candidate viewpoint is unreachable. Skipping.")
            continue
            
        # Prevent duplicate points at the overlapping joints
        if full_path and np.allclose(full_path[-1], segment[0]):
            segment = segment[1:]
            
        full_path.extend(segment)
        # Assign this segment's target focus to every voxel in the segment
        target_tracking.extend([target_frontier] * len(segment))
        
        start_pos = vp

    return np.array(full_path), np.array(target_tracking)


def generate_seamless_mission_trajectory(full_path, target_tracking, num_points=200, smoothing=2.0):
    """
    Generates a C2 continuous spline for X, Y, Z, and Yaw.
    The camera will pan beautifully between targets as the AUV drives.
    """
    if len(full_path) < 4:
        return []

    # 1. Calculate the Raw Yaw for every single A* step
    raw_yaw = np.arctan2(target_tracking[:, 1] - full_path[:, 1], 
                         target_tracking[:, 0] - full_path[:, 0])
    
    # 2. Unwrap the yaw so it doesn't spin wildly at the -Pi/Pi boundary!
    unwrapped_yaw = np.unwrap(raw_yaw)

    # 3. Fit a 4-Dimensional Spline [X, Y, Z, Yaw]
    # We pass all 4 arrays into splprep. It smooths them together over time.
    tck, u = splprep([
        full_path[:, 0], 
        full_path[:, 1], 
        full_path[:, 2], 
        unwrapped_yaw
    ], s=smoothing, k=3)
    
    # 4. Generate the dense, smooth trajectory for the controller
    u_fine = np.linspace(0.0, 1.0, num_points)
    x_smooth, y_smooth, z_smooth, yaw_smooth = splev(u_fine, tck)
    
    # Re-wrap the yaw back to [-Pi, Pi] for standard ROS 2 IMU/Odometry constraints
    yaw_smooth = (yaw_smooth + np.pi) % (2 * np.pi) - np.pi

    # Stack into a final Nx4 array
    trajectory = np.vstack((x_smooth, y_smooth, z_smooth, yaw_smooth)).T
    
    return trajectory

# Example usage:
if __name__ == "__main__":
    x = 1
    y = 1
    z = 0
    angle = 45  # Rotate by 45 degrees

    rotated_point = rotate_around_xy_line((x, y, z), angle)
    print(rotated_point)
