import numpy as np
import open3d as o3d
from scipy.spatial import KDTree
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
import pyoctomap as octomap

# ==========================================
# 1. SYNTHETIC ENVIRONMENT (WITH HOLE & AUV)
# ==========================================
def load_real_octomap(filepath, auv_pos):
    """Loads a real .bt OctoMap file and prepares it for NBV math."""
    print(f"Loading OctoMap from: {filepath}")
    
    # Load the OctoMap using the python bindings from your previous script
    tree = octomap.ColorOcTree(0.1)
    tree =tree.read(filepath)
    occupied_points, empty_points, colors = tree.extractPointCloud()
    
    if len(occupied_points) == 0:
        raise ValueError("OctoMap is empty! Check the filepath.")
        
    points = np.asarray(occupied_points)
    
    # 1. Filter out the seafloor (e.g., Z < -31.5) so we only evaluate the pipe
    z_min_target = -31.5
    pipe_points = points[points[:, 2] > z_min_target]
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pipe_points)
    
    # 2. Clean up floating noise (critical to prevent fake frontiers)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=15, std_ratio=1.5)
    
    # 3. Estimate Normals
    # We use a slightly larger radius (0.4m) to smooth out the blocky OctoMap voxels
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.4, max_nn=30))
    
    # 4. Orient Normals Outward
    # By giving Open3D the AUV's position, it ensures normals face the cameras
    pcd.orient_normals_towards_camera_location(auv_pos)
    
    return pcd

def generate_synthetic_pipe_with_hole(radius=1.0, length=5.0, density=3000):
    theta = np.random.uniform(0, 2*np.pi, density)
    z = np.random.uniform(0, length, density)
    
    valid_mask = ~((z > 2.0) & (z < 3.0) & (theta > 0) & (theta < np.pi/2))
    theta = theta[valid_mask]
    z = z[valid_mask]
    
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    
    noise = np.random.normal(0, 0.02, (len(x), 3))
    points = np.column_stack((x, y, z)) + noise
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))
    
    # Force normals to point outward (critical for incidence angle math)
    normals = np.asarray(pcd.normals)
    for i in range(len(points)):
        outward_vec = np.array([points[i,0], points[i,1], 0.0])
        if np.dot(normals[i], outward_vec) < 0:
            normals[i] = -normals[i]
    pcd.normals = o3d.utility.Vector3dVector(normals)
    
    return pcd

AUV_POSITION = np.array([3.0, 2.0, 2.5])

# ==========================================
# 2. FRONTIER DETECTION & COMPLEXITY
# ==========================================
def calculate_pca_complexity(pcd, k=20):
    points = np.asarray(pcd.points)
    tree = KDTree(points)
    complexities = np.zeros(len(points))
    
    for i, pt in enumerate(points):
        _, idxs = tree.query(pt, k=k)
        neighbors = points[idxs]
        centered = neighbors - np.mean(neighbors, axis=0)
        cov = np.cov(centered.T)
        eigenvalues = np.sort(np.linalg.eigvalsh(cov))[::-1]
        
        if np.sum(eigenvalues) > 0:
            complexities[i] = eigenvalues[2] / np.sum(eigenvalues)
    return complexities

def detect_frontiers(pcd, k=30, angle_threshold_deg=120):
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    tree = KDTree(points)
    
    frontier_indices = []
    angle_threshold = np.radians(angle_threshold_deg)
    
    for i in range(len(points)):
        pt = points[i]
        n = normals[i]
        _, neighbor_idxs = tree.query(pt, k=k)
        neighbors = points[neighbor_idxs]
        
        u = np.cross(n, np.array([0, 0, 1]))
        if np.linalg.norm(u) < 1e-4:
            u = np.cross(n, np.array([0, 1, 0]))
        u = u / np.linalg.norm(u)
        v = np.cross(n, u)
        
        angles = []
        for n_pt in neighbors:
            vec = n_pt - pt
            dist = np.linalg.norm(vec)
            if dist < 1e-5: continue
            angles.append(np.arctan2(np.dot(vec, v), np.dot(vec, u)))
            
        if not angles: continue
        angles.sort()
        angles.append(angles[0] + 2 * np.pi)
        
        if np.max(np.diff(angles)) > angle_threshold:
            frontier_indices.append(i)
            
    return frontier_indices

# ==========================================
# 3. CLUSTERING (NOW RETURNS RICH DATA)
# ==========================================
def cluster_frontiers(points, normals, frontier_indices, fov_radius=1.0, max_angle_deg=45, min_samples=5):
    """
    Visibility-Aware Clustering.
    Groups frontiers ONLY if they fit within a camera's footprint AND face the same direction.
    Prevents the DBSCAN 'chaining effect' where giant continuous frontiers become one impossible target.
    """
    if not frontier_indices:
        return []

    frontier_pts = points[frontier_indices]
    frontier_norms = normals[frontier_indices]

    unassigned = list(range(len(frontier_pts)))
    clusters = []
    cluster_id = 0

    max_angle_rad = np.radians(max_angle_deg)

    while unassigned:
        # Pick the first available point as a seed for a new cluster
        seed_idx = unassigned[0]
        seed_pt = frontier_pts[seed_idx]
        seed_norm = frontier_norms[seed_idx]

        cluster_mask = []
        new_unassigned = []

        # Check every remaining point against the seed's Field of View
        for idx in unassigned:
            pt = frontier_pts[idx]
            norm = frontier_norms[idx]

            # 1. Spatial Check: Does it physically fit inside the camera's view footprint?
            dist = np.linalg.norm(pt - seed_pt)

            # 2. Angular Check: Is it facing the same direction as the seed?
            cos_theta = np.clip(np.dot(seed_norm, norm), -1.0, 1.0)
            angle = np.arccos(cos_theta)

            if dist <= fov_radius and angle <= max_angle_rad:
                cluster_mask.append(idx)
            else:
                new_unassigned.append(idx)

        # Move forward with the points that didn't fit in this cluster
        unassigned = new_unassigned

        # Filter out noise (clusters that are too small to waste a waypoint on)
        if len(cluster_mask) < min_samples:
            continue

        # Extract the accepted cluster data
        c_pts = frontier_pts[cluster_mask]
        c_norms = frontier_norms[cluster_mask]
        c_indices = np.array(frontier_indices)[cluster_mask]

        centroid = np.mean(c_pts, axis=0)

        # Calculate the average normal of the cluster boundary
        avg_normal = np.mean(c_norms, axis=0)
        avg_normal = avg_normal / np.linalg.norm(avg_normal)

        clusters.append({
            'id': cluster_id,
            'centroid': centroid,
            'avg_normal': avg_normal,
            'points': c_pts,
            'normals': c_norms,
            'global_indices': c_indices
        })
        cluster_id += 1

    return clusters

# ==========================================
# 4. VIEWPOINT GENERATION & EVALUATION
# ==========================================
def generate_cone_viewpoints(target_pt, normal, standoff=1.5, polar_angles_deg=[15, 30], azimuth_steps=8):
    """
    Generates a 3D dome of candidate viewpoints around the target's normal.
    Uses spherical coordinates to sample a perfect cone of cameras.
    """
    normal = normal / np.linalg.norm(normal)
    
    # 1. Create a local orthonormal basis (tangent plane)
    u = np.cross(normal, np.array([0, 0, 1]))
    if np.linalg.norm(u) < 1e-4:
        u = np.cross(normal, np.array([0, 1, 0]))
    u = u / np.linalg.norm(u)
    v = np.cross(normal, u)
    
    candidates = []
    
    # 2. Add the direct "straight-on" view (0 degrees polar)
    candidates.append({
        'position': target_pt + (normal * standoff),
        'polar': 0.0,
        'azimuth': 0.0
    })
    
    # 3. Generate rings of viewpoints
    # azimuth_steps=8 means it will check 0, 45, 90, 135, 180, 225, 270, 315 degrees around the circle
    azimuth_angles = np.linspace(0, 360, azimuth_steps, endpoint=False)
    
    for polar_deg in polar_angles_deg:
        polar_rad = np.radians(polar_deg)
        sin_p = np.sin(polar_rad)
        cos_p = np.cos(polar_rad)
        
        for az_deg in azimuth_angles:
            az_rad = np.radians(az_deg)
            
            # Spherical to Cartesian mapping in the local frame
            x_local = sin_p * np.cos(az_rad)
            y_local = sin_p * np.sin(az_rad)
            z_local = cos_p
            
            # Project back to global 3D space
            ray_dir = (x_local * u) + (y_local * v) + (z_local * normal)
            vp = target_pt + (ray_dir * standoff)
            
            candidates.append({
                'position': vp, 
                'polar': polar_deg, 
                'azimuth': az_deg
            })
            
    return candidates

def evaluate_viewpoint(vp_pos, cluster_pts, cluster_normals, cluster_complexities):
    """
    Math: Score = Sum( Incidence_Angle * (1 + Complexity_Weight * PCA_Complexity) )
    This rewards looking directly at highly complex/rough areas.
    """
    total_score = 0.0
    
    for pt, norm, comp in zip(cluster_pts, cluster_normals, cluster_complexities):
        ray = pt - vp_pos
        dist = np.linalg.norm(ray)
        if dist < 1e-4: continue
        ray_dir = ray / dist
        
        # 1. Incidence Angle (Camera looking at surface normal)
        # We want the ray to be exactly opposite to the normal for a perfect view (cos = 1)
        cos_theta = np.clip(np.dot(-ray_dir, norm), 0.0, 1.0)
        
        # 2. Geometric Complexity Multiplier
        # If the point is rough/complex (comp > 0), it becomes more valuable to look at
        complexity_weight = 15.0 
        info_value = 1.0 + (complexity_weight * comp)
        
        # Add to total cluster score
        total_score += (cos_theta * info_value)
        
    return total_score

# ==========================================
# 5. KINEMATIC COST & UTILITY SCORING
# ==========================================
def calculate_kinematic_cost(auv_pos, auv_heading, vp_pos, target_pos, pcd_tree):
    """
    Calculates the physical cost of moving the AUV to a viewpoint.
    Cost = (Path Length) + (Orientation Time/Energy) + (Collision Risk)
    """
    # 1. Path Length (Translational Energy)
    travel_vec = vp_pos - auv_pos
    dist = np.linalg.norm(travel_vec)
    
    # 2. Reorientation Cost (Rotational Energy/Time)
    if dist > 1e-4:
        travel_dir = travel_vec / dist
        cos_turn1 = np.clip(np.dot(auv_heading, travel_dir), -1.0, 1.0)
        turn1_angle = np.arccos(cos_turn1)
    else:
        turn1_angle = 0.0
        
    view_vec = target_pos - vp_pos
    view_dist = np.linalg.norm(view_vec)
    
    if view_dist > 1e-4 and dist > 1e-4:
        view_dir = view_vec / view_dist
        cos_turn2 = np.clip(np.dot(travel_dir, view_dir), -1.0, 1.0)
        turn2_angle = np.arccos(cos_turn2)
    else:
        turn2_angle = 0.0
        
    total_rotation = turn1_angle + turn2_angle
    
    # 3. Collision Risk (Proximity to the nearest pipe surface)
    nearest_dist, _ = pcd_tree.query(vp_pos, k=1)
    # Penalize heavily if we get within 0.5 meters of the pipe
    collision_risk = 1.0 / (nearest_dist + 0.1) if nearest_dist < 0.8 else 0.0
    
    # Weighted Sum of Costs (Tune these weights based on AUV battery vs speed priorities)
    w_dist, w_rot, w_col = 1.0, 0.5, 2.0
    raw_cost = (w_dist * dist) + (w_rot * total_rotation) + (w_col * collision_risk)
    
    return raw_cost, dist, np.degrees(total_rotation), collision_risk

def normalize_array(arr):
    """Min-Max Normalization to scale arrays perfectly between 0.0 and 1.0."""
    arr = np.array(arr)
    val_min, val_max = arr.min(), arr.max()
    if val_max - val_min < 1e-6:
        return np.zeros_like(arr)
    return (arr - val_min) / (val_max - val_min)

# ==========================================
# MAIN EXECUTION & VISUALIZATION
# ==========================================
if __name__ == "__main__":
    print("Generating environment...")
    # Real AUV coordinates at the start of the pipeline
    AUV_POSITION = np.array([-14.054,-82.086,-29.828])
    
    print("Loading real environment...")
    map_path = "/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/NBV_paper_ros2/results/final_map_ros2.bt"
    pcd = load_real_octomap(map_path, AUV_POSITION)
    
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    
    print(f"Loaded {len(points)} valid pipe voxels.")
    
    print("Calculating PCA complexity...")
    complexities = calculate_pca_complexity(pcd)
    
    print("Detecting frontiers...")
    # You might need to tweak k=40 depending on how dense your OctoMap resolution is (0.1m)
    frontier_indices = detect_frontiers(pcd, k=40, angle_threshold_deg=130)
    
    print("Clustering frontiers...")
    clusters = cluster_frontiers(points, normals, frontier_indices, fov_radius=1.0, max_angle_deg=45)
    
    # --- Geometries for Visualization ---
    pcd.paint_uniform_color([0.5, 0.5, 0.5]) # Gray pipe
    geometries = [pcd]
    
    # AUV Position
    auv_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
    auv_sphere.translate(AUV_POSITION)
    auv_sphere.paint_uniform_color([0, 0.5, 1]) # Blue AUV
    geometries.append(auv_sphere)
    
    # Draw Frontier points
    if frontier_indices:
        frontier_pcd = o3d.geometry.PointCloud()
        frontier_pcd.points = o3d.utility.Vector3dVector(points[frontier_indices])
        frontier_pcd.paint_uniform_color([1, 0, 0]) # Red frontiers
        geometries.append(frontier_pcd)

    print(f"\n--- EVALUATING {len(clusters)} CLUSTERS ---")
    
    # AUV Current State
    AUV_HEADING = np.array([1.0, 0.0, 0.0])
    pcd_tree = KDTree(points) 
    
    all_candidates = []
    
    # --- PHASE 1: Generate & Raw Score EVERY Single Candidate ---
    MIN_RAW_GAIN = 2.0 
    
    for cluster in clusters:
        c_complexities = complexities[cluster['global_indices']]
        candidates = generate_cone_viewpoints(cluster['centroid'], cluster['avg_normal'], standoff=2.0)
        
        for cand in candidates:
            # 1. Raw Information Gain
            gain = evaluate_viewpoint(cand['position'], cluster['points'], cluster['normals'], c_complexities)
            
            # CRITICAL FIX: Instantly filter out garbage viewpoints
            if gain < MIN_RAW_GAIN:
                continue 
            
            # 2. Raw Kinematic Cost
            cost, dist, rot_deg, risk = calculate_kinematic_cost(
                AUV_POSITION, AUV_HEADING, cand['position'], cluster['centroid'], pcd_tree
            )
            
            all_candidates.append({
                'cluster_id': cluster['id'],
                'target': cluster['centroid'],
                'position': cand['position'],
                'polar': cand['polar'],
                'azimuth': cand['azimuth'],
                'raw_gain': gain,
                'raw_cost': cost,
                'dist': dist,
                'rot_deg': rot_deg,
                'risk': risk
            })
            
            # Visualize evaluated candidate (Magenta)
            cand_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.04)
            cand_sphere.translate(cand['position'])
            cand_sphere.paint_uniform_color([1, 0, 1])
            geometries.append(cand_sphere)

    # --- PHASE 2: Global Normalization & Utility Calculation ---
    # We normalize across the ENTIRE pool so a "high cost" is relative to all possible moves
    gains = [c['raw_gain'] for c in all_candidates]
    costs = [c['raw_cost'] for c in all_candidates]
    
    norm_gains = normalize_array(gains)
    norm_costs = normalize_array(costs)
    
    # Assign utilities and find the best candidate per cluster
    cluster_winners = {}
    
    for i, cand in enumerate(all_candidates):
        cand['norm_gain'] = norm_gains[i]
        cand['norm_cost'] = norm_costs[i]
        cand['utility'] = norm_gains[i] - norm_costs[i] # U = I - C
        
        cid = cand['cluster_id']
        # If this is the first candidate for this cluster, or it has a better utility than the current winner
        if cid not in cluster_winners or cand['utility'] > cluster_winners[cid]['utility']:
            cluster_winners[cid] = cand

    # --- PHASE 3: Sequence Winners & Find Global Best ---
    print("\n--- CLUSTER WINNERS (UTILITY SEQUENCED) ---")
    
    global_best_utility = -999.0
    global_best_winner = None
    
    for cid, w in cluster_winners.items():
        print(f"Cluster {cid:>2} | Polar: {w['polar']:>3.0f}°, Azimuth: {w['azimuth']:>3.0f}° | Util: {w['utility']:>5.2f} "
              f"(NormGain: {w['norm_gain']:.2f}, NormCost: {w['norm_cost']:.2f}) | "
              f"Path: {w['dist']:.1f}m, Turn: {w['rot_deg']:.0f}°, Risk: {w['risk']:.2f}")
        
        # Visualize cluster winners (Yellow)
        win_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.08)
        win_sphere.translate(w['position'])
        win_sphere.paint_uniform_color([1, 1, 0])
        geometries.append(win_sphere)
        
        if w['utility'] > global_best_utility:
            global_best_utility = w['utility']
            global_best_winner = w

    print("\n==================================================")
    print(f"🌟 GLOBAL BEST TARGET: Cluster {global_best_winner['cluster_id']} (Polar: {global_best_winner['polar']:.0f}°, Azimuth: {global_best_winner['azimuth']:.0f}°) 🌟")
    print(f"Utility Score: {global_best_winner['utility']:.3f}")
    print(f"Coordinates:   [{global_best_winner['position'][0]:.2f}, "
          f"{global_best_winner['position'][1]:.2f}, {global_best_winner['position'][2]:.2f}]")
    print("==================================================\n")

    # Visualize Global Best (Cyan) and Line of Sight
    best_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
    best_sphere.translate(global_best_winner['position'])
    best_sphere.paint_uniform_color([0, 1, 1]) 
    geometries.append(best_sphere)
    
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector([AUV_POSITION, global_best_winner['position']])
    line_set.lines = o3d.utility.Vector2iVector([[0, 1]])
    line_set.colors = o3d.utility.Vector3dVector([[0, 1, 1]])
    geometries.append(line_set)

    print("Opening Open3D Visualization...")
    print("Gray: Surface | Red: Frontiers | Blue: AUV | Magenta: Evaluated Cone Points")
    print("Yellow: Cluster Winners | CYAN: Global Best Utility Next-Best-View")
    o3d.visualization.draw_geometries(geometries)