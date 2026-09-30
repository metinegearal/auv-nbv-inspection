import cv2
import numpy as np
import math
import os
from datetime import datetime
import pyoctomap
import open3d as o3d
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree

from Tools.mapping3D import generate_seamless_mission_trajectory, plan_multi_waypoint_route
from Tools.search_algo import astar_3d, check_line_of_sight

def generate_global_point_cloud(depth_image, color_image, mask, fov_degrees, location, rotation_matrix, max_depth=5.0):
    height, width = depth_image.shape
    fx = (width / 2.0) / math.tan(math.radians(fov_degrees) / 2.0)
    fy = fx
    cx, cy = width / 2.0, height / 2.0

    u, v = np.meshgrid(np.arange(width), np.arange(height))
    u, v, Z = u.flatten(), v.flatten(), depth_image.flatten()
    
    # M = mask.flatten() > 0

    valid = (Z > 0.5) & (Z < max_depth) # & M 
    u, v, Z = u[valid], v[valid], Z[valid]

    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    
    pitch_degrees = 45.0 
    pitch_rad = math.radians(pitch_degrees)
    
    # ---------------------------------------------------------
    # FIXED: Camera +X is "Right". Vehicle +Y is "Left". 
    # Therefore, Vehicle Y must be -X.
    # ---------------------------------------------------------
    local_x = Z * math.cos(pitch_rad) - Y * math.sin(pitch_rad)
    local_y = -X  
    local_z = -Z * math.sin(pitch_rad) - Y * math.cos(pitch_rad)
    
    local_points = np.vstack((local_x, local_y, local_z)).T
    
    # ==========================================
    # FIXED: Y is Left, so the offset is Positive
    # X (Front) = 0.35
    # Y (Left)  = 0.1125 + 0.275
    # Z (Up)    = 0.0
    # ==========================================
    # camera_offset = np.array([0.35, (0.1125 + 0.275), -0.5])
    # local_points = local_points + camera_offset
    
    # Rotate the translated points into the global world frame
    global_points = np.dot(local_points, rotation_matrix.T) + location

    rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    colors = rgb_image.reshape(-1, 3) / 255.0  
    
    valid_colors = colors[valid]

    return global_points, valid_colors

# ==========================================
# 2. Stateful Real-Time Mapper Class
# ==========================================
class RealTimeStereoMapper:
    def __init__(self, fov_degrees=90.0, focal_length_px=256.0, baseline_meters=0.5, voxel_resolution=0.05):

        self.explore_file = open("Notes/exploration_parameters.txt","w")
        self.previous_leaves = 0
        # Camera Params
        self.fov_degrees = fov_degrees
        self.focal_length_px = focal_length_px
        self.baseline_meters = baseline_meters
        
        # 1. Persistent Map Storage
        self.tree = pyoctomap.ColorOcTree(voxel_resolution)
        # self.counting_tree = pyoctomap.CountingOcTree(voxel_resolution)
        
        # 2. Non-Blocking Open3D Visualizer Setup
        # self.vis = o3d.visualization.Visualizer()
        # self.vis.create_window(window_name="HoloOcean Real-Time Map")
        
        # Create persistent geometry objects
        self.map_pcd = o3d.geometry.PointCloud()
        self.vehicle_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
        self.vehicle_marker.paint_uniform_color([1.0, 0.0, 0.0]) # Red
        
        # Add them to the visualizer
        # self.vis.add_geometry(self.map_pcd)
        # self.vis.add_geometry(self.vehicle_marker)
        
        # Keep track of the last known location so we can move the marker accurately
        self.last_location = np.array([0.0, 0.0, 0.0])

    def process_frame(self, location, yaw_radians, roll_radians, pitch_radians, frame1, depth_map):
        if frame1 is None:
            return
        location+= np.array([0.35, 0.1125+0.275, -0.5]) # front, left, down


        # height, width = frame1.shape[:2]
        # dynamic_focal_length = (width / 2.0) / math.tan(math.radians(self.fov_degrees) / 2.0)
        # print(f"Dynamic Focal Length: {dynamic_focal_length:.2f} px (Image Size: {width}x{height})")

        cr, sr = math.cos(roll_radians), math.sin(roll_radians)
        cp, sp = math.cos(pitch_radians), math.sin(pitch_radians)
        cy, sy = math.cos(yaw_radians), math.sin(yaw_radians)

        rotation_matrix = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [  -sp,            cp*sr,            cp*cr]
        ])

        # 1. Compute Data

        # Normalize the depth map to 0-255 for visualization
        # depth_norm = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        # Apply a vibrant color map
        # depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

        # Save the colorful depth map
        # cv2.imshow("frame1", frame1)
        # cv2.imshow("frame2", frame2)
        # cv2.imshow("depth_color", depth_color)
        # cv2.waitKey(1)

        frame1 = frame1[:, :, :3]
        # mask, overlay = detect_mask_and_overlay(frame1)
        mask=None
        overlay=None

        # 3. Pass the mask into the point cloud generator
        raw_points, raw_colors = generate_global_point_cloud(
            depth_map, frame1, mask, self.fov_degrees, location, rotation_matrix, max_depth=4.0
        )

        # 2. Accumulate into the Persistent OctoMap
        if len(raw_points) > 0:
            points_f64 = raw_points.astype(np.float64)
            colors_f64 = raw_colors.astype(np.float64) # PyOctoMap requires float64 colors
            location_f64 = location.astype(np.float64)
            
            # Pass the colors array into the tree insertion
            self.tree.insertPointCloud(
                points_f64, 
                sensor_origin=location_f64, 
                max_range=4.0, # Should match your max_depth
                colors=colors_f64
            )

            # res = self.tree.getResolution()
            # voxels, hit_counts = np.unique(np.floor(points_f64 / res).astype(int), axis=0, return_counts=True)
            
            # for voxel, hits in zip(voxels, hit_counts):
            #     coord = (voxel + 0.5) * res
            #     node = self.counting_tree.updateNode(coord.astype(np.float64))
            #     if hits > 1:
            #         # updateNode adds 1, so we add the remainder of the hits
            #         node.setCount(node.getCount() + hits - 1)

            current_leaves = self.tree.getNumLeafNodes()
            exploration_score = current_leaves - self.previous_leaves
            self.previous_leaves = current_leaves
            self.explore_file.write(f"Exploration Score: {exploration_score}, Total Leaves: {current_leaves}\n")
        
        # 3. Update Visuals
        # now=datetime.now()
        # self.update_visualization(location)
        # print(datetime.now()-now)

    def update_visualization(self, current_location):
        # Extract the accumulated map
        occupied_points, _ = self.tree.extractPointCloud()
        
        if len(occupied_points) > 0:
            # Update Map Points
            self.map_pcd.points = o3d.utility.Vector3dVector(occupied_points)
            
            # Recompute Colors based on depth
            z_vals = occupied_points[:, 2]
            z_norm = (z_vals - z_vals.min()) / (z_vals.max() - z_vals.min() + 1e-6)
            colors = plt.get_cmap('ocean')(z_norm)[:, :3]
            self.map_pcd.colors = o3d.utility.Vector3dVector(colors)
            
            # Tell Open3D the map data changed
            self.vis.update_geometry(self.map_pcd)

        # Update Vehicle Position dynamically
        translation_vector = current_location - self.last_location
        self.vehicle_marker.translate(translation_vector)
        self.vis.update_geometry(self.vehicle_marker)
        self.last_location = current_location.copy()

        # 4. Render the frame without blocking Python
        self.vis.poll_events()
        self.vis.update_renderer()

    def save_map(self, filename="underwater_map.bt"):
        """Call this when your simulation ends to save the accumulated map."""
        self.tree.write(filename)
        # count_filename = filename.replace(".bt", "_counts.bt")
        # self.counting_tree.write(count_filename)
        # print(f"Hit-count map saved to {count_filename}")
        print(f"Final map saved to {filename}")
        # self.vis.destroy_window()

    def generate_candidate_viewpoints(self, current_position):
        """
        Phase 1: Generates viewpoints evaluated by an Information-to-Cost Utility Ratio.
        Returns a list of tuples: (viewpoint_coordinates, target_frontier_coordinates)
        and the smoothed 4D trajectory array.
        """
        from scipy.spatial import cKDTree
        
        current_position += np.array([0.35, 0.1125+0.275, -0.5])
        
        occupied_points, empty_points, _ = self.tree.extractPointCloud()
        if len(occupied_points) == 0:
            return [], []

        res = self.tree.getResolution()
        occ_idx = np.floor(occupied_points / res).astype(int)
        occ_set = set(map(tuple, occ_idx))

        # ==========================================
        # 0. CONFIGURATION SPACE (C-SPACE) INFLATION
        # ==========================================
        auv_radius_meters = 0.6  # Adjust this to your AUV's actual physical radius
        inflation_voxels = int(np.ceil(auv_radius_meters / res))
        
        # Create a spherical offset kernel
        x, y, z = np.mgrid[-inflation_voxels:inflation_voxels+1, 
                           -inflation_voxels:inflation_voxels+1, 
                           -inflation_voxels:inflation_voxels+1]
        kernel = (x**2 + y**2 + z**2) <= inflation_voxels**2
        offsets = np.argwhere(kernel) - inflation_voxels
        
        # Fast memory-efficient inflation
        inflated_occ_set = set()
        for offset in offsets:
            shifted_idx = occ_idx + offset
            inflated_occ_set.update(map(tuple, shifted_idx))
            
        print(f"Inflated C-Space: {len(inflated_occ_set)} danger voxels (Original: {len(occ_set)}).")

        # ==========================================
        # 1. Calculate Information Gain (Surface Sparsity)
        # ==========================================
        print("Calculating Surface Sparsity Frontiers...")
        z_min_target = -32.0  
        
        valid_occ = occupied_points[occupied_points[:, 2] > z_min_target]
        frontier_centers = []
        frontier_scores = [] # We must retain the scores to calculate Utility

        if len(valid_occ) > 10:
            occ_tree = cKDTree(valid_occ)
            distances, _ = occ_tree.query(valid_occ, k=11)
            rho = np.mean(distances[:, 1:], axis=1)
            
            rho_min, rho_max = rho.min(), rho.max()
            base_scores = (rho - rho_min) / (rho_max - rho_min + 1e-6)

            sparse_mask = (base_scores > 0.15) & (rho > (res * 1.2))
            
            candidate_centers = valid_occ[sparse_mask]
            candidate_scores = base_scores[sparse_mask]

            if len(candidate_centers) > 0:
                if len(empty_points) > 0:
                    free_tree = cKDTree(empty_points)
                    search_radius = res * 3.0
                    free_neighbors = free_tree.query_ball_point(candidate_centers, r=search_radius)
                    free_counts = np.array([len(indices) for indices in free_neighbors])
                    damping_factor = np.exp(-0.05 * free_counts)
                    adjusted_scores = candidate_scores * damping_factor
                else:
                    adjusted_scores = candidate_scores

                # Keep a very low baseline filter just to drop absolute zeros
                cutoff_threshold = 0.05
                final_mask = adjusted_scores >= cutoff_threshold
                
                frontier_centers = candidate_centers[final_mask].tolist()
                frontier_scores = adjusted_scores[final_mask].tolist()

        # ==========================================
        # 2. Evaluate Utility & Generate Viewpoints
        # ==========================================
        candidates = []
        
        # Tuning parameters for U(v)
        alpha_cost = 1.0           # Weight of distance cost
        min_utility_threshold = 0.02 # Minimum acceptable Information-to-Cost ratio

        # Zip the centers with their Information Gain scores
        for f, info_gain in zip(frontier_centers, frontier_scores):
            f = np.array(f)
            vec = f - current_position[:3] 
            vec[2] = 0
            norm = np.linalg.norm(vec)
            dir_xy = np.array([1.0, 0.0, 0.0]) if norm < 1e-5 else vec / norm
            
            for dist in [2.0, 1.5, 3.0, 1.0]:
                for pitch_deg in [45.0, 60.0, 30.0, 75.0, 15.0]:
                    pitch = np.radians(pitch_deg)
                    
                    viewpoint = f - dir_xy * (dist * np.cos(pitch)) + np.array([0, 0, dist * np.sin(pitch)])
                    
                    if check_line_of_sight(viewpoint, f, occ_set, res):

                        viewpoint_voxel = tuple(np.floor(viewpoint / res).astype(int))
                        if viewpoint_voxel not in inflated_occ_set:
                            # --- THE UTILITY CALCULATION ---
                            # Cost(x -> v) approximated by Euclidean distance + small epsilon to prevent div/0
                            travel_cost = np.linalg.norm(viewpoint - current_position[:3]) + 0.1 
                            
                            # U(v) = Information / (alpha * Cost)
                            utility = info_gain / (alpha_cost * travel_cost)
                            
                            if utility >= min_utility_threshold:
                                candidates.append((viewpoint, f))
                            else:
                                # Uncomment to see how many points get rejected for being "not worth it"
                                # print(f"Rejected viewpoint: Utility {utility:.3f} < {min_utility_threshold} (Cost: {travel_cost:.1f}m)")
                                pass
                            
                        break # Found the best LoS for this frontier, move to next frontier
                else:
                    continue
                break


        if len(candidates) > 0:
            print(f"Pre-culling waypoints: {len(candidates)}")
            
            fov_rad = np.radians(80.0) # Slightly tighter than your actual FOV to guarantee overlap
            max_view_range = 3.0       # Max reliable depth range
            
            covered_frontiers = set()
            optimized_candidates = []
            
            for vp, target in candidates:
                # Round target to 3 decimal places to use as a unique dictionary/set key
                target_tup = tuple(np.round(target, 3))
                
                # If a previous camera position already covers this target, skip it!
                if target_tup in covered_frontiers:
                    continue 
                    
                # Keep this viewpoint
                optimized_candidates.append((vp, target))
                
                # Simulate the camera's forward optical axis
                vp_arr = np.array(vp)
                forward_vec = np.array(target) - vp_arr
                dist_to_target = np.linalg.norm(forward_vec)
                
                if dist_to_target < 1e-5:
                    continue
                forward_dir = forward_vec / dist_to_target
                
                # Check WHICH OTHER frontiers this single camera position can see
                for f in frontier_centers:
                    f_tup = tuple(np.round(f, 3))
                    
                    if f_tup in covered_frontiers:
                        continue
                        
                    vec_to_f = np.array(f) - vp_arr
                    dist_to_f = np.linalg.norm(vec_to_f)
                    
                    # Is it within range?
                    if dist_to_f <= max_view_range:
                        dir_to_f = vec_to_f / dist_to_f
                        
                        # Calculate angle between camera forward axis and the frontier
                        cos_angle = np.clip(np.dot(forward_dir, dir_to_f), -1.0, 1.0)
                        angle = np.arccos(cos_angle)
                        
                        # If it falls within the FOV cone, mark it as covered!
                        if angle <= (fov_rad / 2.0):
                            # (Optional for speed: We skip the raycast check here assuming 
                            # local frontiers are mostly unoccluded if they share the same surface)
                            covered_frontiers.add(f_tup)
            
            print(f"FOV Culling: Reduced {len(candidates)} waypoints down to {len(optimized_candidates)} optimized stops!")
            candidates = optimized_candidates
        # ==========================================
        # 3. Plan Multi-Waypoint Mission
        # ==========================================
        smooth_traj = []
        print(f"Evaluated {len(frontier_centers)} frontiers -> Kept {len(candidates)} high-utility viewpoints.")
        
        if len(candidates) > 0:
            full_path, target_tracking = plan_multi_waypoint_route(
                current_position, candidates, inflated_occ_set, res
            )
            
            if len(full_path) > 0:
                smooth_traj = generate_seamless_mission_trajectory(full_path, target_tracking, num_points=200)
        else:
            print("Exploration Complete or Remaining Frontiers lack sufficient Utility!")

        return candidates, smooth_traj

    
    def get_path_to_viewpoint(self, current_location, target_viewpoint):
        """
        Phase 2: Calculate the A* path from the AUV's current location to the chosen viewpoint.
        """
        occupied_points, _, _ = self.tree.extractPointCloud()
        res = self.tree.getResolution()
        occ_set = set(map(tuple, np.floor(occupied_points / res).astype(int)))
        
        path = astar_3d(current_location, target_viewpoint, occ_set, res)
        return path

    def validate_current_path(self, current_path, target_viewpoint, target_frontier):
        """
        Phase 3: Call this in your main loop every tick while moving.
        Fast O(1) checks directly against the OctoTree memory.
        Returns False if a new obstacle blocks the path OR the line of sight.
        """
        res = self.tree.getResolution()
        
        # 1. Check if the path is still safe
        for p in current_path:
            node = self.tree.search(p.astype(np.float64))
            if node is not None and self.tree.isNodeOccupied(node):
                print("WARNING: Path obstructed by newly mapped obstacle!")
                return False
                
        # 2. Check if the view is still clear (we need to rebuild a quick occ_set for the thick raycast)
        # To keep it fast, we only extract the tree if the path check passed
        occupied_points, _, _ = self.tree.extractPointCloud()
        occ_set = set(map(tuple, np.floor(occupied_points / res).astype(int)))
        
        if not check_line_of_sight(target_viewpoint, target_frontier, occ_set, res):
            print("WARNING: Target viewpoint line-of-sight is now blocked!")
            return False
            
        return True

# ==========================================
# How to use this in your HoloOcean Script
# ==========================================
if __name__ == "__main__":
    # 1. Initialize ONCE before your loop
    mapper = RealTimeStereoMapper(voxel_resolution=0.1)
    
    # 2. Simulate your HoloOcean Loop
    print("Starting simulation loop... (Close the window to stop)")
    
    for step in range(50): # Replace with your while loop
        # Fake vehicle moving forward
        sim_location = np.array([step * 0.1, 0.0, 0.0]) 
        sim_yaw = 0.0
        
        # Load frames (replace with holoocean tick data)
        frame1 = cv2.imread('frame1.png')
        frame2 = cv2.imread('frame2.png')
        
        # 3. Call this EVERY tick. It will accumulate data and update instantly.
        mapper.process_frame(sim_location, sim_yaw, frame1, frame2)
        
    # 4. Save the map when done
    mapper.save_map()