import numpy as np
import open3d as o3d
import pyoctomap
import matplotlib.pyplot as plt
import copy

def evaluate_map_accuracy(headless=False):
    # File Paths
    octomap_file = "/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/NBV_paper_ros2/final_map_ros2.bt"
    # octomap_file = "/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/Notes/3d_map_lawnmover/final_map.bt"
    # octomap_file = "/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/Notes/3d_map_lawnmover/short_v/final_map.bt"
    # ground_truth_file = "/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/Tools/3D_scan/GT/objects/SM_MERGED_pipe_full.OBJ"
    ground_truth_file = "/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/Tools/3D_scan/GT/objects/SM_MERGED_pipe_m_l100_390.OBJ"
    
    # Global Coordinates from Odometry/Simulation
    obj_center_x = -13.0
    obj_center_y = -84.75
    surface_z = -32.0

    print("=======================================")
    print("  AUV MAPPING BENCHMARK EVALUATOR")
    print("=======================================\n")

    # ==========================================
    # 1. LOAD AND PREPARE OCTOMAP
    # ==========================================
    print(f"[1/5] Loading OctoMap: {octomap_file.split('/')[-1]}")
    tree = pyoctomap.ColorOcTree(0.1) # Must match your MapperConfig resolution
    tree=tree.read(octomap_file)
    
    occupied_points, _, _ = tree.extractPointCloud()
    
    # Filter Map: Remove seafloor and stray noise below Z = -31.9 (giving a 10cm buffer)
    valid_map_points = occupied_points[occupied_points[:, 2] > (surface_z + 0.1)]
    
    map_pcd = o3d.geometry.PointCloud()
    map_pcd.points = o3d.utility.Vector3dVector(valid_map_points)
    
    # Statistical Outlier Removal (Denoising)
    map_pcd, _ = map_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.5)
    print(f"      -> Map points after filtering: {len(map_pcd.points)}")

    # ==========================================
    # 2. LOAD AND PREPARE GROUND TRUTH MESH
    # ==========================================
    print(f"[2/5] Loading Ground Truth: {ground_truth_file.split('/')[-1]}")
    gt_mesh = o3d.io.read_triangle_mesh(ground_truth_file)
    
    # Scale Unreal Engine cm to ROS meters
    gt_mesh.scale(0.01, center=(0, 0, 0))
    gt_pcd = gt_mesh.sample_points_uniformly(number_of_points=1000000)
    
    gt_points = np.asarray(gt_pcd.points)
    
    # Fix Unreal Engine Left-Handed Y-Axis Mirroring
    gt_points[:, 1] = -gt_points[:, 1]
    
    # Filter GT: Remove the seafloor from the Unreal export
    gt_valid = gt_points[gt_points[:, 2] > (surface_z + 0.1)]
    gt_pcd.points = o3d.utility.Vector3dVector(gt_valid)
    print(f"      -> GT points after filtering: {len(gt_pcd.points)}")

    # ==========================================
    # 3. ALIGNMENT (INITIAL GUESS + ICP)
    # ==========================================
    print("[3/5] Aligning Coordinate Systems...")
    
    # Shift GT center to the known AUV global coordinates
    gt_current_center = gt_pcd.get_center()
    gt_pcd.translate(-gt_current_center) # Move to 0,0,0 temporarily
    
    # Move to the user's known global center
    # Note: Z is shifted slightly up so the pipe rests on the -32 surface
    gt_pcd.translate([obj_center_x, obj_center_y, surface_z + 1.0]) 

    # Run Fine Iterative Closest Point (ICP) for millimeter perfect matching
    print("      -> Running Point-to-Point ICP...")
    icp_result = o3d.pipelines.registration.registration_icp(
        map_pcd, gt_pcd, max_correspondence_distance=2.0,
        init=np.eye(4),
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200)
    )
    
    # Apply the mathematical alignment to the map
    map_pcd.transform(icp_result.transformation)

    # ==========================================
    # 4. CALCULATE ACADEMIC METRICS
    # ==========================================
    print("[4/5] Calculating Paper Metrics...")
    
    # A. ACCURACY (How wrong is the map?) -> Distance from Map to GT
    map_to_gt_dists = np.asarray(map_pcd.compute_point_cloud_distance(gt_pcd))
    mean_error = np.mean(map_to_gt_dists)
    rmse = np.sqrt(np.mean(map_to_gt_dists**2))
    
    # B. COMPLETENESS / COVERAGE (How much of the pipe did we see?) -> Distance from GT to Map
    gt_to_map_dists = np.asarray(gt_pcd.compute_point_cloud_distance(map_pcd))
    
    # If a GT point is within 15cm of a mapped point, we consider it "Covered"
    coverage_threshold = 0.15 
    covered_points = np.sum(gt_to_map_dists < coverage_threshold)
    coverage_percentage = (covered_points / len(gt_to_map_dists)) * 100.0

    print("\n=======================================")
    print("          BENCHMARK RESULTS")
    print("=======================================")
    print(f"Mean Error (Accuracy):  {mean_error:.4f} m")
    print(f"RMSE (Accuracy):        {rmse:.4f} m")
    print(f"Surface Coverage:       {coverage_percentage:.2f} % (at {coverage_threshold}m threshold)")
    print("=======================================\n")

    if headless:
        return coverage_percentage, rmse

    # ==========================================
    # 5. HEATMAP COLORIZATION & VISUALIZATION
    # ==========================================
    print("[5/5] Generating Error Heatmap...")
    
    # Color the Ground Truth a faint transparent grey
    gt_pcd.paint_uniform_color([0.7, 0.7, 0.7])
    
    # Normalize Map distances for the Heatmap: Green (0.0m error) -> Red (0.2m error)
    max_error_viz = 0.2 
    normalized_errors = np.clip(map_to_gt_dists / max_error_viz, 0, 1)
    
    cmap = plt.get_cmap("RdYlGn_r") # Red-Yellow-Green (Reversed)
    error_colors = cmap(normalized_errors)[:, :3]
    map_pcd.colors = o3d.utility.Vector3dVector(error_colors)

    print("Opening 3D Visualizer. Close the window to exit.")
    print("LEGEND: Green = Perfect Match | Red = Noisy/Inaccurate | Grey = Ground Truth")
    
    o3d.visualization.draw_geometries([gt_pcd, map_pcd], 
                                      window_name="AUV Mapping Benchmark",
                                      width=1280, height=720)

if __name__ == "__main__":
    evaluate_map_accuracy()