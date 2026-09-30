import numpy as np
import open3d as o3d
from scipy.spatial import KDTree
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt

# ==========================================
# 1. SYNTHETIC ENVIRONMENT (WITH HOLE & AUV)
# ==========================================
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
def cluster_frontiers(points, normals, frontier_indices, eps=0.3, min_samples=5):
    """Clusters frontiers and returns a rich dictionary for each cluster."""
    if not frontier_indices:
        return []
        
    frontier_pts = points[frontier_indices]
    frontier_norms = normals[frontier_indices]
    
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(frontier_pts)
    labels = clustering.labels_
    
    clusters = []
    for label in set(labels):
        if label == -1: continue # Skip noise
        
        mask = (labels == label)
        c_pts = frontier_pts[mask]
        c_norms = frontier_norms[mask]
        c_indices = np.array(frontier_indices)[mask]
        
        centroid = np.mean(c_pts, axis=0)
        
        # Calculate the average normal of the cluster boundary
        avg_normal = np.mean(c_norms, axis=0)
        avg_normal = avg_normal / np.linalg.norm(avg_normal)
        
        clusters.append({
            'id': label,
            'centroid': centroid,
            'avg_normal': avg_normal,
            'points': c_pts,
            'normals': c_norms,
            'global_indices': c_indices
        })
        
    return clusters

# ==========================================
# 4. VIEWPOINT GENERATION & EVALUATION
# ==========================================
def generate_cone_viewpoints(target_pt, normal, standoff=1.5, angles_deg=[-30, -15, 0, 15, 30]):
    """Generates an arc of candidate viewpoints around the target's normal."""
    normal = normal / np.linalg.norm(normal)
    tangent = np.cross(normal, np.array([0, 0, 1]))
    if np.linalg.norm(tangent) < 1e-4:
        tangent = np.cross(normal, np.array([0, 1, 0]))
    tangent = tangent / np.linalg.norm(tangent)
    
    candidates = []
    for angle in angles_deg:
        rad = np.radians(angle)
        rotated_dir = normal * np.cos(rad) + tangent * np.sin(rad)
        vp = target_pt + (rotated_dir * standoff)
        candidates.append({'position': vp, 'angle': angle})
        
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
    pcd = generate_synthetic_pipe_with_hole()
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    
    print("Calculating PCA complexity...")
    complexities = calculate_pca_complexity(pcd)
    
    print("Detecting frontiers...")
    frontier_indices = detect_frontiers(pcd, k=40, angle_threshold_deg=130)
    
    print("Clustering frontiers...")
    clusters = cluster_frontiers(points, normals, frontier_indices, eps=0.25)
    
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
    
    # AUV Current State (Assuming it starts pointing down the X-axis)
    AUV_HEADING = np.array([1.0, 0.0, 0.0])
    pcd_tree = KDTree(points) # For collision checking
    
    cluster_winners = []
    
    # --- PHASE 1: Find the Best Local Viewpoint for Each Cluster ---
    for cluster in clusters:
        c_complexities = complexities[cluster['global_indices']]
        candidates = generate_cone_viewpoints(cluster['centroid'], cluster['avg_normal'], standoff=2.0)
        
        best_gain = -1.0
        best_vp = None
        
        for cand in candidates:
            # We ONLY calculate Information Gain here
            gain = evaluate_viewpoint(cand['position'], cluster['points'], cluster['normals'], c_complexities)
            
            # Visualize all candidates as small magenta spheres
            cand_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.04)
            cand_sphere.translate(cand['position'])
            cand_sphere.paint_uniform_color([1, 0, 1])
            geometries.append(cand_sphere)
            
            if gain > best_gain:
                best_gain = gain
                best_vp = cand
                
        # Calculate Kinematic Cost for the winning local viewpoint
        raw_cost, dist, rot_deg, risk = calculate_kinematic_cost(
            AUV_POSITION, AUV_HEADING, best_vp['position'], cluster['centroid'], pcd_tree
        )
        
        cluster_winners.append({
            'cluster_id': cluster['id'],
            'position': best_vp['position'],
            'target': cluster['centroid'],
            'raw_gain': best_gain,
            'raw_cost': raw_cost,
            'dist': dist,
            'rot_deg': rot_deg,
            'risk': risk
        })

    # --- PHASE 2: Normalize and Calculate Global Utility ---
    print("\n--- GLOBAL UTILITY SEQUENCING ---")
    
    gains = [w['raw_gain'] for w in cluster_winners]
    costs = [w['raw_cost'] for w in cluster_winners]
    
    norm_gains = normalize_array(gains)
    norm_costs = normalize_array(costs)
    
    global_best_utility = -999.0
    global_best_winner = None
    
    for i, w in enumerate(cluster_winners):
        # U = I - C
        utility = norm_gains[i] - norm_costs[i]
        w['utility'] = utility
        
        print(f"Cluster {w['cluster_id']:>2} | Util: {utility:>5.2f} "
              f"(NormGain: {norm_gains[i]:.2f}, NormCost: {norm_costs[i]:.2f}) | "
              f"Path: {w['dist']:.1f}m, Turn: {w['rot_deg']:.0f}°, Risk: {w['risk']:.2f}")
        
        # Visualize standard winners as yellow spheres
        win_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.08)
        win_sphere.translate(w['position'])
        win_sphere.paint_uniform_color([1, 1, 0]) # Yellow
        geometries.append(win_sphere)
        
        if utility > global_best_utility:
            global_best_utility = utility
            global_best_winner = w

    # --- PHASE 3: Highlight the Absolute Best Waypoint ---
    print("\n==================================================")
    print(f"🌟 GLOBAL BEST TARGET: Cluster {global_best_winner['cluster_id']} 🌟")
    print(f"Utility Score: {global_best_winner['utility']:.3f}")
    print(f"Coordinates:   [{global_best_winner['position'][0]:.2f}, "
          f"{global_best_winner['position'][1]:.2f}, {global_best_winner['position'][2]:.2f}]")
    print("==================================================\n")

    # Make the absolute best waypoint a massive Cyan Sphere
    best_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
    best_sphere.translate(global_best_winner['position'])
    best_sphere.paint_uniform_color([0, 1, 1]) # Cyan
    geometries.append(best_sphere)
    
    # Draw a line from the AUV directly to the Global Best Waypoint
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector([AUV_POSITION, global_best_winner['position']])
    line_set.lines = o3d.utility.Vector2iVector([[0, 1]])
    line_set.colors = o3d.utility.Vector3dVector([[0, 1, 1]]) # Cyan Line
    geometries.append(line_set)

    print("Opening Open3D Visualization...")
    print("Gray: Surface | Red: Frontiers | Blue: AUV | Magenta: Evaluated Cone Points")
    print("Yellow: Cluster Winners | CYAN: Global Best Utility Next-Best-View")
    o3d.visualization.draw_geometries(geometries)