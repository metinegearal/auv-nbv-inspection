import numpy as np
from scipy.spatial import KDTree




from ..domain.nbv_parameters import DEFAULT_NBV_PARAMETERS

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
    k_actual = min(int(k), n_points)
    if k_actual < 2:
        return complexities
    # Batch queries and eigensolves; cap temporary storage for large maps.
    for start in range(0, n_points, 1024):
        _, idxs = tree.query(points[start:start + 1024], k=k_actual)
        neighbors = points[idxs]
        centered = neighbors - neighbors.mean(axis=1, keepdims=True)
        covariance = np.einsum('nki,nkj->nij', centered, centered) / (k_actual - 1)
        eigenvalues = np.linalg.eigvalsh(covariance)
        totals = eigenvalues.sum(axis=1)
        complexities[start:start + len(idxs)] = np.divide(
            eigenvalues[:, 0], totals, out=np.zeros_like(totals), where=totals > 0)
    return complexities

def detect_frontiers(pcd, k=40, angle_threshold_deg=130):
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals, dtype=np.float64)
    tree = KDTree(points)

    frontier_indices = []
    angle_threshold = np.radians(angle_threshold_deg)
    if len(points) < 2:
        return frontier_indices
    k_actual = min(max(2, int(k)), len(points))
    for start in range(0, len(points), 1024):
        batch = points[start:start + 1024]
        batch_normals = normals[start:start + 1024]
        _, neighbor_idxs = tree.query(batch, k=k_actual)
        vectors = points[neighbor_idxs] - batch[:, None, :]
        u = np.cross(batch_normals, [0, 0, 1])
        vertical = np.linalg.norm(u, axis=1) < 1e-4
        u[vertical] = np.cross(batch_normals[vertical], [0, 1, 0])
        u /= np.maximum(np.linalg.norm(u, axis=1, keepdims=True), 1e-12)
        v = np.cross(batch_normals, u)
        valid = (np.linalg.norm(vectors, axis=2) >= 1e-5) & (
            np.linalg.norm(batch_normals, axis=1)[:, None] > 0)
        angles = np.arctan2(np.einsum('nki,ni->nk', vectors, v),
                            np.einsum('nki,ni->nk', vectors, u))
        angles[~valid] = np.inf
        angles.sort(axis=1)
        counts = valid.sum(axis=1)
        has_neighbors = counts > 0
        interior = np.arange(k_actual - 1)[None, :] < (counts - 1)[:, None]
        gaps = np.subtract(angles[:, 1:], angles[:, :-1],
                           out=np.zeros((len(batch), k_actual - 1)), where=interior)
        last = angles[np.arange(len(batch)), np.maximum(counts - 1, 0)]
        wrap = np.zeros(len(batch))
        wrap[has_neighbors] = (angles[has_neighbors, 0] + 2 * np.pi
                               - last[has_neighbors])
        largest = np.maximum(gaps.max(axis=1), wrap)
        frontier_indices.extend((start + np.flatnonzero(
            has_neighbors & (largest > angle_threshold))).tolist())
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

        indices = np.asarray(unassigned)
        distances = np.linalg.norm(frontier_pts[indices] - seed_pt, axis=1)
        angles = np.arccos(np.clip(frontier_norms[indices] @ seed_norm, -1.0, 1.0))
        accepted = (distances <= fov_radius) & (angles <= max_angle_rad)
        cluster_mask = indices[accepted]
        unassigned = indices[~accepted].tolist()

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

def select_upward_view_axis(target_pt, auv_position, occupied_tree,
                            standoff=1.5, azimuth_steps=36):
    """Choose yaw by occupied-point clearance with elevation locked to +45 degrees.

    Sample the target-to-viewpoint rays after the first 20% of standoff, where
    departure from the target surface is expected. Prefer maximum minimum ray
    clearance, then mean clearance; ties favor yaw toward the AUV. Absence of
    occupied samples is not proof of observed free space. Route collision checks
    still determine whether the resulting viewpoints can be reached safely.
    """
    if standoff <= 0.0 or azimuth_steps < 1:
        raise ValueError('standoff and view-axis azimuth_steps must be positive')
    delta = np.asarray(auv_position) - np.asarray(target_pt)
    preferred_yaw = np.arctan2(delta[1], delta[0])
    yaw = preferred_yaw + np.linspace(0.0, 2.0 * np.pi, azimuth_steps, endpoint=False)
    axes = np.column_stack((np.cos(yaw), np.sin(yaw), np.ones(azimuth_steps))) / np.sqrt(2.0)
    distances = np.linspace(0.2 * standoff, standoff, 20)
    samples = np.asarray(target_pt) + axes[:, None, :] * distances[None, :, None]
    clearances, _ = occupied_tree.query(samples.reshape(-1, 3), k=1)
    clearances = clearances.reshape(azimuth_steps, -1)
    minimum = clearances.min(axis=1)
    eligible = np.flatnonzero(np.isclose(minimum, minimum.max(), rtol=1e-6, atol=1e-9))
    means = clearances[eligible].mean(axis=1)
    tied = np.flatnonzero(np.isclose(means, means.max(), rtol=1e-6, atol=1e-9))
    # Azimuth samples begin at preferred_yaw; break remaining ties by heading.
    indices = eligible[tied]
    alignment = np.cos(yaw[indices] - preferred_yaw)
    return axes[indices[np.argmax(alignment)]]


def generate_cone_viewpoints(
        target_pt, normal, standoff=1.5, polar_angles_deg=None,
        azimuth_steps=8):
    """Generate the central viewpoint and unchanged cone offsets around its axis.

    The historical ``normal`` argument now receives the chosen central axis;
    actual surface normals remain available for clustering and gain evaluation.
    """
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

def evaluate_viewpoint(vp_pos, cluster_pts, cluster_normals, cluster_complexities,
                       complexity_weight=15.0):
    rays = np.asarray(cluster_pts) - vp_pos
    distances = np.linalg.norm(rays, axis=1)
    valid = distances >= 1e-4
    directions = rays[valid] / distances[valid, None]
    cos_theta = np.clip(np.sum(-directions * np.asarray(cluster_normals)[valid],
                               axis=1), 0.0, 1.0)
    return float(np.sum(cos_theta * (1.0 + complexity_weight * np.asarray(cluster_complexities)[valid])))

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
    parameters=None, timing_callback=None, *, preprocessor, diagnostics, clock):
    """
    Takes raw OctoMap occupied points and returns a sorted list of the best 
    Next-Best-View (NBV) waypoints for the AUV trajectory planner.
    """
    diagnostics.info('Calculating NBV waypoints...')
    def timed(stage, operation):
        started = clock.perf_counter()
        value = operation()
        if timing_callback is not None:
            timing_callback(stage, (clock.perf_counter() - started) * 1000.0)
        return value
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

    pcd = preprocessor.prepare(pipe_points, auv_position, parameters, timed)
    if pcd is None:
        return []

    cleaned_points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    pcd_tree = KDTree(cleaned_points)
    # Use every occupied map sample for axis selection, including points removed
    # by target filtering/downsampling. Gain and cost retain their existing data.
    occupied_tree = KDTree(points_raw)

    # 2. Mathematical Pipeline
    complexities = timed('nbv_pca', lambda: calculate_pca_complexity(
        pcd, k=int(parameters['pca_neighbors'])))
    frontier_indices = timed('nbv_frontier_detection', lambda: detect_frontiers(
        pcd,
        k=int(parameters['frontier_neighbors']),
        angle_threshold_deg=float(parameters['frontier_angle_threshold_deg'])))
    clusters = timed('nbv_clustering', lambda: cluster_frontiers(
        cleaned_points,
        normals,
        frontier_indices,
        fov_radius=float(parameters['cluster_fov_radius']),
        max_angle_deg=float(parameters['cluster_max_angle_deg']),
        min_samples=int(parameters['cluster_min_samples'])))

    if not clusters:
        return []

    # 3. Candidate Generation & Raw Evaluation
    evaluation_started = clock.perf_counter()
    all_candidates = []
    min_raw_gain = float(parameters['min_raw_gain'])
    observation_positions = np.asarray(
        [sample[0] for sample in observation_history], dtype=float).reshape(-1, 3)
    observation_yaws = np.asarray([sample[1] for sample in observation_history])

    for cluster in clusters:
        vectors = cluster['centroid'] - observation_positions
        distances = np.linalg.norm(vectors, axis=1)
        angles = np.arctan2(vectors[:, 1], vectors[:, 0]) - observation_yaws
        yaw_diffs = np.abs(np.arctan2(np.sin(angles), np.cos(angles)))
        already_visited = np.any(
            (distances > parameters['visited_min_distance'])
            & (distances < parameters['visited_max_distance'])
            & (yaw_diffs < np.radians(parameters['visited_yaw_threshold_deg'])))
        if already_visited:
            continue # Drop this cluster, we have confirmed visual coverage

        # =========================================================
        # THE MISSING MATH BLOCK: Generate & evaluate viewpoints
        # =========================================================
        c_complexities = complexities[cluster['global_indices']]
        view_axis = select_upward_view_axis(
            cluster['centroid'], auv_position, occupied_tree,
            standoff=float(parameters['standoff']),
            azimuth_steps=int(parameters['view_axis_azimuth_steps']))
        candidates = generate_cone_viewpoints(
            cluster['centroid'],
            view_axis,
            standoff=float(parameters['standoff']),
            polar_angles_deg=parameters['polar_angles_deg'],
            azimuth_steps=int(parameters['azimuth_steps']))

        for cand in candidates:
            gain = evaluate_viewpoint(
                cand['position'], cluster['points'], cluster['normals'], c_complexities,
                complexity_weight=float(parameters['complexity_weight']))

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
    if timing_callback is not None:
        timing_callback('nbv_candidate_evaluation',
                        (clock.perf_counter() - evaluation_started) * 1000.0)
    if not all_candidates:
        return []

    # 4. Global Utility Normalization
    gains = [c['raw_gain'] for c in all_candidates]
    costs = [c['raw_cost'] for c in all_candidates]

    norm_gains = normalize_array(gains)
    norm_costs = normalize_array(costs)

    cluster_winners = {}
    for i, cand in enumerate(all_candidates):
        cand['utility'] = (float(parameters['utility_gain_weight']) * norm_gains[i]
                           - float(parameters['utility_cost_weight']) * norm_costs[i])
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
