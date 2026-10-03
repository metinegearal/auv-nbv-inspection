import numpy as np
import open3d as o3d
from rclpy.logging import get_logger
from scipy.spatial import KDTree


LOGGER = get_logger('nbv_waypoint_calculator')

DEFAULT_NBV_PARAMETERS = {
    'z_min_target': -31.5,
    'pca_neighbors': 20,
    'frontier_neighbors': 40,
    'frontier_angle_threshold_deg': 130.0,
    'cluster_fov_radius': 1.0,
    'cluster_max_angle_deg': 45.0,
    'cluster_min_samples': 5,
    'voxel_downsample_size': 0.25,
    'outlier_neighbors': 15,
    'outlier_std_ratio': 1.5,
    'normal_radius': 0.4,
    'normal_max_neighbors': 30,
    'min_raw_gain': 0.5,
    'standoff': 1.5,
    'polar_angles_deg': [15.0, 30.0],
    'azimuth_steps': 8,
    'visited_min_distance': 0.1,
    'visited_max_distance': 3.0,
    'visited_yaw_threshold_deg': 30.0,
    'complexity_weight': 15.0,
    'distance_weight': 1.0,
    'rotation_weight': 0.5,
    'collision_weight': 2.0,
    'collision_distance_threshold': 0.8,
    'collision_distance_offset': 0.1,
}

# ==========================================
# NBV MATHEMATICAL HELPER FUNCTIONS
# ==========================================

def calculate_pca_complexity(pcd, k=20):
    points = np.asarray(pcd.points)
    n_points = len(points)
    
    if n_points < 3:
        return np.zeros(n_points)
        
    tree = KDTree(points)
    complexities = np.zeros(n_points)
    
    # Cap k so we never ask for more neighbors than actually exist
    k_actual = min(k, n_points)
    
    for i, pt in enumerate(points):
        _, idxs = tree.query(pt, k=k_actual)
        neighbors = points[idxs]
        centered = neighbors - np.mean(neighbors, axis=0)
        cov = np.cov(centered.T)
        eigenvalues = np.sort(np.linalg.eigvalsh(cov))[::-1]
        
        if np.sum(eigenvalues) > 0:
            complexities[i] = eigenvalues[2] / np.sum(eigenvalues)
    return complexities

def detect_frontiers(pcd, k=40, angle_threshold_deg=130):
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

def cluster_frontiers(points, normals, frontier_indices, fov_radius=1.0, max_angle_deg=45, min_samples=5):
    if not frontier_indices:
        return []

    frontier_pts = points[frontier_indices]
    frontier_norms = normals[frontier_indices]

    unassigned = list(range(len(frontier_pts)))
    clusters = []
    cluster_id = 0
    max_angle_rad = np.radians(max_angle_deg)

    while unassigned:
        seed_idx = unassigned[0]
        seed_pt = frontier_pts[seed_idx]
        seed_norm = frontier_norms[seed_idx]

        cluster_mask = []
        new_unassigned = []

        for idx in unassigned:
            pt = frontier_pts[idx]
            norm = frontier_norms[idx]
            dist = np.linalg.norm(pt - seed_pt)
            cos_theta = np.clip(np.dot(seed_norm, norm), -1.0, 1.0)
            angle = np.arccos(cos_theta)

            if dist <= fov_radius and angle <= max_angle_rad:
                cluster_mask.append(idx)
            else:
                new_unassigned.append(idx)

        unassigned = new_unassigned

        if len(cluster_mask) < min_samples:
            continue

        c_pts = frontier_pts[cluster_mask]
        c_norms = frontier_norms[cluster_mask]
        c_indices = np.array(frontier_indices)[cluster_mask]

        centroid = np.mean(c_pts, axis=0)
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

def generate_cone_viewpoints(
        target_pt, normal, standoff=1.5, polar_angles_deg=None,
        azimuth_steps=8):
    if polar_angles_deg is None:
        polar_angles_deg = [15, 30]
    normal = normal / np.linalg.norm(normal)
    u = np.cross(normal, np.array([0, 0, 1]))
    if np.linalg.norm(u) < 1e-4:
        u = np.cross(normal, np.array([0, 1, 0]))
    u = u / np.linalg.norm(u)
    v = np.cross(normal, u)
    
    candidates = []
    candidates.append({'position': target_pt + (normal * standoff)})
    
    azimuth_angles = np.linspace(0, 360, azimuth_steps, endpoint=False)
    for polar_deg in polar_angles_deg:
        polar_rad = np.radians(polar_deg)
        sin_p, cos_p = np.sin(polar_rad), np.cos(polar_rad)
        for az_deg in azimuth_angles:
            az_rad = np.radians(az_deg)
            x_local, y_local, z_local = sin_p * np.cos(az_rad), sin_p * np.sin(az_rad), cos_p
            ray_dir = (x_local * u) + (y_local * v) + (z_local * normal)
            candidates.append({'position': target_pt + (ray_dir * standoff)})
            
    return candidates

def evaluate_viewpoint(vp_pos, cluster_pts, cluster_normals, cluster_complexities):
    total_score = 0.0
    for pt, norm, comp in zip(cluster_pts, cluster_normals, cluster_complexities):
        ray = pt - vp_pos
        dist = np.linalg.norm(ray)
        if dist < 1e-4: continue
        ray_dir = ray / dist
        cos_theta = np.clip(np.dot(-ray_dir, norm), 0.0, 1.0)
        info_value = 1.0 + (15.0 * comp)
        total_score += (cos_theta * info_value)
    return total_score

def calculate_kinematic_cost(
    auv_pos, auv_heading, vp_pos, target_pos, pcd_tree,
    distance_weight=1.0, rotation_weight=0.5, collision_weight=2.0,
    collision_distance_threshold=0.8, collision_distance_offset=0.1):
    travel_vec = vp_pos - auv_pos
    dist = np.linalg.norm(travel_vec)
    
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
    nearest_dist, _ = pcd_tree.query(vp_pos, k=1)
    collision_risk = (
        1.0 / (nearest_dist + collision_distance_offset)
        if nearest_dist < collision_distance_threshold else 0.0)
    raw_cost = (
        (distance_weight * dist)
        + (rotation_weight * total_rotation)
        + (collision_weight * collision_risk))
    return raw_cost

def normalize_array(arr):
    arr = np.array(arr)
    val_min, val_max = arr.min(), arr.max()
    if val_max - val_min < 1e-6:
        return np.zeros_like(arr)
    return (arr - val_min) / (val_max - val_min)

# ==========================================
# MAIN NBV PIPELINE FUNCTION
# ==========================================
def calculate_nbv_waypoints(
    occupied_points, auv_position, auv_heading, observation_history=None,
    parameters=None):
    """
    Takes raw OctoMap occupied points and returns a sorted list of the best 
    Next-Best-View (NBV) waypoints for the AUV trajectory planner.
    """
    LOGGER.info('Calculating NBV waypoints...')
    parameters = {**DEFAULT_NBV_PARAMETERS, **(parameters or {})}
    if observation_history is None:
        observation_history = []
    points_raw = np.asarray(occupied_points)
    if len(points_raw) == 0:
        return []
        
    # 1. Pre-Processing (Filter seabed & clean noise)
    z_min_target = float(parameters['z_min_target'])
    pipe_points = points_raw[points_raw[:, 2] > z_min_target]
    if len(pipe_points) == 0:
        return []
        
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pipe_points)
    pcd = pcd.voxel_down_sample(
        voxel_size=float(parameters['voxel_downsample_size']))
    pcd, _ = pcd.remove_statistical_outlier(
        nb_neighbors=int(parameters['outlier_neighbors']),
        std_ratio=float(parameters['outlier_std_ratio']))
    
    if len(pcd.points) < 40:
        return []
        
    # Normal generation (oriented towards AUV)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=float(parameters['normal_radius']),
        max_nn=int(parameters['normal_max_neighbors'])))
    pcd.orient_normals_towards_camera_location(auv_position)
    
    cleaned_points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    pcd_tree = KDTree(cleaned_points)
    
    # 2. Mathematical Pipeline
    complexities = calculate_pca_complexity(
        pcd, k=int(parameters['pca_neighbors']))
    frontier_indices = detect_frontiers(
        pcd,
        k=int(parameters['frontier_neighbors']),
        angle_threshold_deg=float(parameters['frontier_angle_threshold_deg']))
    clusters = cluster_frontiers(
        cleaned_points,
        normals,
        frontier_indices,
        fov_radius=float(parameters['cluster_fov_radius']),
        max_angle_deg=float(parameters['cluster_max_angle_deg']),
        min_samples=int(parameters['cluster_min_samples']))
    
    if not clusters:
        return []

    # 3. Candidate Generation & Raw Evaluation
    all_candidates = []
    min_raw_gain = float(parameters['min_raw_gain'])
    
    for cluster in clusters:
        already_visited = False
        for obs_pos, obs_yaw in observation_history:
            vec = cluster['centroid'] - obs_pos
            dist = np.linalg.norm(vec)
            
            # 1. Was it within the camera's physical range?
            if (parameters['visited_min_distance'] < dist
                    < parameters['visited_max_distance']):
                # 2. Was the AUV physically facing it?
                angle_to_target = np.arctan2(vec[1], vec[0])
                
                # Normalize the angle difference
                yaw_diff = np.abs(np.arctan2(
                    np.sin(angle_to_target - obs_yaw), 
                    np.cos(angle_to_target - obs_yaw)
                ))
                
                if yaw_diff < np.radians(
                    parameters['visited_yaw_threshold_deg']):
                    LOGGER.info(
                        f"Cluster {cluster['id']} was already observed "
                        f"(distance: {dist:.2f} m, "
                        f"yaw_diff: {np.degrees(yaw_diff):.2f} deg).")
                    already_visited = True
                    break
                else:
                    LOGGER.info(
                        f"Cluster {cluster['id']} was in range but outside "
                        f"camera FOV (yaw_diff: "
                        f"{np.degrees(yaw_diff):.2f} deg).")
            else:
                LOGGER.info(
                    f"Cluster {cluster['id']} is out of range from previous "
                    f"observation (distance: {dist:.2f} m).")
                
        if already_visited:
            continue # Drop this cluster, we have confirmed visual coverage

        # =========================================================
        # THE MISSING MATH BLOCK: Generate & evaluate viewpoints
        # =========================================================
        c_complexities = complexities[cluster['global_indices']]
        candidates = generate_cone_viewpoints(
            cluster['centroid'],
            cluster['avg_normal'],
            standoff=float(parameters['standoff']),
            polar_angles_deg=parameters['polar_angles_deg'],
            azimuth_steps=int(parameters['azimuth_steps']))
        
        for cand in candidates:
            gain = evaluate_viewpoint(cand['position'], cluster['points'], cluster['normals'], c_complexities)
            
            # Apply the threshold
            if gain < min_raw_gain:
                continue
                
            cost = calculate_kinematic_cost(
                auv_position,
                auv_heading,
                cand['position'],
                cluster['centroid'],
                pcd_tree,
                distance_weight=float(parameters['distance_weight']),
                rotation_weight=float(parameters['rotation_weight']),
                collision_weight=float(parameters['collision_weight']),
                collision_distance_threshold=float(
                    parameters['collision_distance_threshold']),
                collision_distance_offset=float(
                    parameters['collision_distance_offset']))
            
            all_candidates.append({
                'cluster_id': cluster['id'],
                'target': cluster['centroid'],
                'position': cand['position'],
                'raw_gain': gain,
                'raw_cost': cost
            })

    # If all frontiers were either visited or below the gain threshold, exploration is done
    if not all_candidates:
        return []

    # 4. Global Utility Normalization
    gains = [c['raw_gain'] for c in all_candidates]
    costs = [c['raw_cost'] for c in all_candidates]
    
    norm_gains = normalize_array(gains)
    norm_costs = normalize_array(costs)
    
    cluster_winners = {}
    for i, cand in enumerate(all_candidates):
        cand['utility'] = norm_gains[i] - norm_costs[i]
        cid = cand['cluster_id']
        
        if cid not in cluster_winners or cand['utility'] > cluster_winners[cid]['utility']:
            cluster_winners[cid] = cand

    # 5. Format and Sort Output
    final_waypoints = []
    for cid, winner in cluster_winners.items():
        final_waypoints.append({
            'cluster_id': cid,
            'position': winner['position'],
            'target': winner['target'],
            'utility': winner['utility'],
            'raw_gain': winner['raw_gain'],
            'raw_cost': winner['raw_cost']
        })
        
    final_waypoints.sort(key=lambda x: x['utility'], reverse=True)
    return final_waypoints