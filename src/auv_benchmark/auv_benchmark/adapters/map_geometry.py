import numpy as np
import open3d as o3d
import pyoctomap
import matplotlib.pyplot as plt

class Open3DMapEvaluation:
    def __init__(self, config):
        self.config = config

    def prepare(self):
        print("=======================================")
        print("  AUV MAPPING BENCHMARK EVALUATOR")
        print("=======================================\n")

        # ==========================================
        # 1. LOAD AND PREPARE OCTOMAP
        # ==========================================
        print(f"[1/5] Loading OctoMap: {self.config.octomap_file.split('/')[-1]}")
        tree = pyoctomap.ColorOcTree(0.1) # Must match your MapperConfig resolution
        tree=tree.read(self.config.octomap_file)

        occupied_points, _, _ = tree.extractPointCloud()

        # Filter Map: Remove seafloor and stray noise below Z = -31.9 (giving a 10cm buffer)
        valid_map_points = occupied_points[occupied_points[:, 2] > (self.config.surface_z + 0.1)]
        # valid_map_points = occupied_points[
        #     (occupied_points[:, 2] > (self.config.surface_z + 0.1)) & 
        #     (occupied_points[:, 1] >= -84.75)
        # ]

        map_pcd = o3d.geometry.PointCloud()
        map_pcd.points = o3d.utility.Vector3dVector(valid_map_points)

        # Statistical Outlier Removal (Denoising)
        map_pcd, _ = map_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.5)
        print(f"      -> Map points after filtering: {len(map_pcd.points)}")

        # ==========================================
        # 2. LOAD AND PREPARE GROUND TRUTH MESH
        # ==========================================
        print(f"[2/5] Loading Ground Truth: {self.config.ground_truth_file.split('/')[-1]}")
        gt_mesh = o3d.io.read_triangle_mesh(self.config.ground_truth_file)

        # Scale Unreal Engine cm to ROS meters
        gt_mesh.scale(0.01, center=(0, 0, 0))
        gt_pcd = gt_mesh.sample_points_uniformly(number_of_points=1000000)

        gt_points = np.asarray(gt_pcd.points)

        # Fix Unreal Engine Left-Handed Y-Axis Mirroring
        gt_points[:, 1] = -gt_points[:, 1]

        # Filter GT: Remove the seafloor from the Unreal export
        gt_valid = gt_points[gt_points[:, 2] > (self.config.surface_z + 0.1)]
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
        gt_pcd.translate([self.config.obj_center_x, self.config.obj_center_y, self.config.surface_z + 1.0]) 

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


        self.map_pcd, self.gt_pcd = map_pcd, gt_pcd

    def map_distances(self):
        return self.map_pcd.compute_point_cloud_distance(self.gt_pcd)
    def ground_truth_distances(self):
        return self.gt_pcd.compute_point_cloud_distance(self.map_pcd)
    def visualize(self, distances):
        map_pcd, gt_pcd = self.map_pcd, self.gt_pcd
        # ==========================================
        # 5. HEATMAP COLORIZATION & VISUALIZATION
        # ==========================================
        print("[5/5] Generating Error Heatmap...")

        # Color the Ground Truth a faint transparent grey
        gt_pcd.paint_uniform_color([0.7, 0.7, 0.7])

        # Normalize Map distances for the Heatmap: Green (0.0m error) -> Red (0.2m error)
        max_error_viz = 0.2 
        normalized_errors = np.clip(distances / max_error_viz, 0, 1)

        cmap = plt.get_cmap("RdYlGn_r") # Red-Yellow-Green (Reversed)
        error_colors = cmap(normalized_errors)[:, :3]
        map_pcd.colors = o3d.utility.Vector3dVector(error_colors)

        print("Opening 3D Visualizer. Close the window to exit.")
        print("LEGEND: Green = Perfect Match | Red = Noisy/Inaccurate | Grey = Ground Truth")

        o3d.visualization.draw_geometries([gt_pcd, map_pcd], 
                                          window_name="AUV Mapping Benchmark",
                                          width=1280, height=720)

