import numpy as np
from scipy.spatial import KDTree




from ..domain.nbv_parameters import DEFAULT_NBV_PARAMETERS

# ==========================================
# NBV MATHEMATICAL HELPER FUNCTIONS
# ==========================================

def calculate_pca_complexity(pcd, k=20, indices=None, tree=None):
    """Evaluate requested samples against the unchanged full-cloud neighborhood."""
    points = np.asarray(pcd.points)
    n_points = len(points)

    if n_points < 3:
        return np.zeros(n_points)

    tree = KDTree(points) if tree is None else tree
    complexities = np.zeros(n_points)
    selected = np.arange(n_points) if indices is None else np.asarray(indices, dtype=int)
    k_actual = min(int(k), n_points)
    if k_actual < 2:
        return complexities
    # Batch queries and eigensolves; cap temporary storage for large maps.
    for start in range(0, len(selected), 1024):
        batch_indices = selected[start:start + 1024]
        _, idxs = tree.query(points[batch_indices], k=k_actual)
        neighbors = points[idxs]
        centered = neighbors - neighbors.mean(axis=1, keepdims=True)
        covariance = np.einsum('nki,nkj->nij', centered, centered) / (k_actual - 1)
        eigenvalues = np.linalg.eigvalsh(covariance)
        totals = eigenvalues.sum(axis=1)
        complexities[batch_indices] = np.divide(
            eigenvalues[:, 0], totals, out=np.zeros_like(totals), where=totals > 0)
    return complexities

def detect_frontiers(pcd, k=40, angle_threshold_deg=130, radius=0.5, min_neighbors=3,
                     tree=None, focus_indices=None):
    """Find local surface boundaries from gaps between tangent-plane neighbors.

    A radius bound prevents distant samples across a hole from filling its gap.
    ``k`` caps work per point; ``radius=None`` permits an unbounded diagnostic
    comparison. Neighbors with no tangent displacement carry no boundary angle.
    ``focus_indices`` selects query centers, retaining full-cloud neighbors and
    returning indices into the original cloud.
    """
    points = np.asarray(pcd.points, dtype=np.float64)
    normals = np.asarray(pcd.normals, dtype=np.float64)
    if points.shape != normals.shape or points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('Frontier points and normals must be matching Nx3 arrays')
    if radius is not None and (not np.isfinite(radius) or radius <= 0):
        raise ValueError('Frontier radius must be finite and positive')
    if not np.isfinite(angle_threshold_deg) or not 0 < angle_threshold_deg < 360:
        raise ValueError('Frontier angle threshold must be between 0 and 360 degrees')
    if min_neighbors < 2 or int(k) <= min_neighbors:
        raise ValueError('Frontier k must exceed min_neighbors, which must be at least two')
    if len(points) <= min_neighbors:
        return []
    selected = (np.arange(len(points)) if focus_indices is None
                else np.asarray(focus_indices, dtype=int))
    if selected.ndim != 1 or np.any(selected < 0) or np.any(selected >= len(points)):
        raise ValueError('Frontier focus indices must be a one-dimensional array of valid indices')
    if not len(selected):
        return []
    tree = KDTree(points) if tree is None else tree
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, lengths, out=np.zeros_like(normals),
                        where=np.isfinite(lengths) & (lengths > 1e-12))
    frontier_indices = []
    angle_threshold = np.radians(angle_threshold_deg)
    k_actual = min(int(k), len(points))
    for start in range(0, len(selected), 1024):
        batch_indices = selected[start:start + 1024]
        batch = points[batch_indices]
        batch_normals = normals[batch_indices]
        distances, neighbor_idxs = tree.query(
            batch, k=k_actual, distance_upper_bound=(np.inf if radius is None
                                                     else np.nextafter(radius, np.inf)))
        within_radius = np.isfinite(distances)
        # Radius-limited queries pad to k with infinities. Avoid projecting and
        # sorting that padding, and skip rows unable to supply enough neighbors.
        support = within_radius.sum(axis=1)
        rows = np.flatnonzero(support > min_neighbors)
        if not len(rows):
            continue
        width = int(support[rows].max())
        batch = batch[rows]
        batch_normals = batch_normals[rows]
        within_radius = within_radius[rows, :width]
        neighbor_idxs = neighbor_idxs[rows, :width]
        neighbor_idxs = np.where(within_radius, neighbor_idxs, 0)
        vectors = points[neighbor_idxs] - batch[:, None, :]
        u = np.cross(batch_normals, [0, 0, 1])
        vertical = np.linalg.norm(u, axis=1) < 1e-4
        u[vertical] = np.cross(batch_normals[vertical], [0, 1, 0])
        u /= np.maximum(np.linalg.norm(u, axis=1, keepdims=True), 1e-12)
        v = np.cross(batch_normals, u)
        projected_u = np.einsum('nki,ni->nk', vectors, u)
        projected_v = np.einsum('nki,ni->nk', vectors, v)
        valid = within_radius & (np.hypot(projected_u, projected_v) >= 1e-5)
        angles = np.arctan2(projected_v, projected_u)
        angles[~valid] = np.inf
        angles.sort(axis=1)
        counts = valid.sum(axis=1)
        has_neighbors = counts >= min_neighbors
        interior = np.arange(width - 1)[None, :] < (counts - 1)[:, None]
        gaps = np.subtract(angles[:, 1:], angles[:, :-1],
                           out=np.zeros((len(batch), width - 1)), where=interior)
        last = angles[np.arange(len(batch)), np.maximum(counts - 1, 0)]
        wrap = np.zeros(len(batch))
        wrap[has_neighbors] = (angles[has_neighbors, 0] + 2 * np.pi
                               - last[has_neighbors])
        largest = np.maximum(gaps.max(axis=1), wrap)
        valid_frontiers_in_batch = rows[has_neighbors & (largest > angle_threshold)]
        frontier_indices.extend(batch_indices[valid_frontiers_in_batch].tolist())
    return frontier_indices

def cluster_frontiers(points, normals, frontier_indices, fov_radius=1.0, max_angle_deg=45, min_samples=5):
    """Pack dense seed neighborhoods without consuming rejected small groups."""
    if len(frontier_indices) == 0:
        return []
    if not np.isfinite(fov_radius) or fov_radius <= 0 or min_samples < 1:
        raise ValueError('Cluster radius and minimum sample count must be positive')
    if not np.isfinite(max_angle_deg) or not 0 <= max_angle_deg <= 180:
        raise ValueError('Cluster normal angle must be between 0 and 180 degrees')
    frontier_indices = np.asarray(frontier_indices, dtype=int)
    frontier_pts = np.asarray(points)[frontier_indices]
    frontier_norms = np.asarray(normals, dtype=float)[frontier_indices]
    lengths = np.linalg.norm(frontier_norms, axis=1)
    valid = (np.isfinite(frontier_pts).all(axis=1)
             & np.isfinite(lengths) & (lengths > 1e-12))
    frontier_indices = frontier_indices[valid]
    frontier_pts = frontier_pts[valid]
    frontier_norms = frontier_norms[valid] / lengths[valid, None]
    if not len(frontier_pts):
        return []
    tree = KDTree(frontier_pts)
    neighborhoods = []
    minimum_dot = np.cos(np.radians(max_angle_deg))
    for index, neighbors in enumerate(tree.query_ball_point(frontier_pts, fov_radius)):
        neighbors = np.asarray(neighbors, dtype=int)
        compatible = frontier_norms[neighbors] @ frontier_norms[index] >= minimum_dot - 1e-12
        neighborhoods.append(neighbors[compatible])
    # Prefer dense neighborhoods, breaking ties spatially rather than by input
    # order. A rejected seed leaves its neighbors available to later clusters.
    counts = np.array([len(neighbors) for neighbors in neighborhoods])
    order = np.lexsort((frontier_pts[:, 2], frontier_pts[:, 1], frontier_pts[:, 0], -counts))
    unassigned = np.ones(len(frontier_pts), dtype=bool)
    clusters = []
    for seed_idx in order:
        if not unassigned[seed_idx]:
            continue
        neighbors = neighborhoods[seed_idx]
        cluster_mask = neighbors[unassigned[neighbors]]
        if len(cluster_mask) < min_samples:
            continue
        unassigned[cluster_mask] = False
        c_pts = frontier_pts[cluster_mask]
        c_norms = frontier_norms[cluster_mask]
        c_indices = frontier_indices[cluster_mask]

        centroid = np.mean(c_pts, axis=0)
        avg_normal = np.mean(c_norms, axis=0)
        length = np.linalg.norm(avg_normal)
        avg_normal = avg_normal / length if length > 1e-12 else frontier_norms[seed_idx]

        clusters.append({
            'id': len(clusters),
            'centroid': centroid,
            'avg_normal': avg_normal,
            'points': c_pts,
            'normals': c_norms,
            'global_indices': c_indices
        })

    # Partitioning a real boundary can leave a short tail below min_samples.
    # Attach such points to a compatible retained cluster when the combined
    # patch still fits inside its centroid's FOV radius. Do not count them twice.
    centers = np.array([cluster['centroid'] for cluster in clusters]).reshape(-1, 3)
    averages = np.array([cluster['avg_normal'] for cluster in clusters]).reshape(-1, 3)
    sizes = np.array([len(cluster['points']) for cluster in clusters])
    for index in order[unassigned[order]]:
        distances = np.linalg.norm(centers - frontier_pts[index], axis=1)
        compatible = np.einsum('ij,j->i', averages, frontier_norms[index]) >= minimum_dot - 1e-12
        eligible = np.flatnonzero(
            compatible & (distances <= fov_radius * (sizes + 1) / sizes + 1e-12))
        # Match the previous nearest-cluster preference and stable tie order,
        # but sort only eligible clusters instead of calling norm M times.
        for cluster_id in eligible[np.argsort(distances[eligible], kind='stable')]:
            cluster = clusters[cluster_id]
            combined = np.vstack((cluster['points'], frontier_pts[index]))
            centroid = combined.mean(axis=0)
            if np.max(np.linalg.norm(combined - centroid, axis=1)) > fov_radius + 1e-12:
                continue
            cluster['points'] = combined
            cluster['centroid'] = centroid
            cluster['normals'] = np.vstack((cluster['normals'], frontier_norms[index]))
            cluster['global_indices'] = np.append(cluster['global_indices'], frontier_indices[index])
            average = cluster['normals'].mean(axis=0)
            if np.linalg.norm(average) > 1e-12:
                cluster['avg_normal'] = average / np.linalg.norm(average)
            centers[cluster_id] = cluster['centroid']
            averages[cluster_id] = cluster['avg_normal']
            sizes[cluster_id] += 1
            break
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


def evaluate_viewpoints(positions, cluster_pts, cluster_normals, cluster_complexities,
                        complexity_weight=15.0):
    """Score the same incidence/PCA gain for a batch of candidate viewpoints."""
    positions = np.asarray(positions, dtype=float).reshape(-1, 3)
    points = np.asarray(cluster_pts)
    normals = np.asarray(cluster_normals)
    weights = 1.0 + complexity_weight * np.asarray(cluster_complexities)
    gains = np.zeros(len(positions))
    # Bound temporary storage even for large surface clusters or wide cones.
    batch_size = max(1, 65536 // max(1, len(points)))
    for start in range(0, len(positions), batch_size):
        rays = points[None, :, :] - positions[start:start + batch_size, None, :]
        distances = np.linalg.norm(rays, axis=2)
        directions = np.divide(rays, distances[:, :, None], out=np.zeros_like(rays),
                               where=distances[:, :, None] >= 1e-4)
        incidence = np.clip(np.sum(-directions * normals[None, :, :], axis=2), 0., 1.)
        gains[start:start + batch_size] = np.sum(incidence * weights, axis=1)
    return gains


def calculate_kinematic_costs(auv_pos, auv_heading, positions, target_pos, pcd_tree,
                              distance_weight=1.0, rotation_weight=0.5, collision_weight=2.0,
                              collision_distance_threshold=0.8, collision_distance_offset=0.1):
    """Batch the existing cost formula and exact nearest-obstacle queries."""
    positions = np.asarray(positions, dtype=float).reshape(-1, 3)
    travel = positions - auv_pos
    distances = np.linalg.norm(travel, axis=1)
    moving = distances > 1e-4
    travel_dirs = np.divide(travel, distances[:, None], out=np.zeros_like(travel),
                            where=moving[:, None])
    turn1 = np.zeros(len(positions))
    turn1[moving] = np.arccos(np.clip(travel_dirs[moving] @ auv_heading, -1., 1.))
    view = target_pos - positions
    view_distances = np.linalg.norm(view, axis=1)
    turning = moving & (view_distances > 1e-4)
    view_dirs = np.divide(view, view_distances[:, None], out=np.zeros_like(view),
                          where=turning[:, None])
    turn2 = np.zeros(len(positions))
    turn2[turning] = np.arccos(np.clip(np.sum(
        travel_dirs[turning] * view_dirs[turning], axis=1), -1., 1.))
    nearest, _ = pcd_tree.query(positions, k=1)
    risk = np.zeros(len(positions))
    close = nearest < collision_distance_threshold
    risk[close] = 1.0 / (nearest[close] + collision_distance_offset)
    return distance_weight * distances + rotation_weight * (turn1 + turn2) + collision_weight * risk


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
    diagnostics.info(
        f'NBV target mask: {len(points_raw)} occupied points -> {len(pipe_points)} '
        f'above z_min_target={z_min_target}')
    if len(pipe_points) == 0:
        return []

    pcd = preprocessor.prepare(pipe_points, auv_position, parameters, timed)
    if pcd is None:
        diagnostics.info(f'NBV preprocessing left no usable surface from {len(pipe_points)} points')
        return []

    cleaned_points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    exploration_radius = float(parameters['exploration_radius'])
    if not np.isfinite(exploration_radius) or exploration_radius <= 0:
        raise ValueError('NBV exploration radius must be finite and positive')
    distances_to_auv = np.linalg.norm(cleaned_points - auv_position, axis=1)
    focus_indices = np.flatnonzero(distances_to_auv <= exploration_radius)
    diagnostics.info(
        f'Focus Window: Processing {len(focus_indices)}/{len(cleaned_points)} '
        f'points within {exploration_radius}m')
    if not len(focus_indices):
        return []
    pcd_tree = KDTree(cleaned_points)
    # Use every occupied map sample for axis selection, including points removed
    # by target filtering/downsampling. Gain and cost retain their existing data.
    occupied_tree = KDTree(points_raw)

    # 2. Mathematical Pipeline
    frontier_indices = timed('nbv_frontier_detection', lambda: detect_frontiers(
        pcd,
        k=int(parameters['frontier_neighbors']),
        angle_threshold_deg=float(parameters['frontier_angle_threshold_deg']),
        radius=float(parameters['frontier_radius']),
        min_neighbors=int(parameters['frontier_min_neighbors']), tree=pcd_tree,
        focus_indices=focus_indices))
    clusters = timed('nbv_clustering', lambda: cluster_frontiers(
        cleaned_points,
        normals,
        frontier_indices,
        fov_radius=float(parameters['cluster_fov_radius']),
        max_angle_deg=float(parameters['cluster_max_angle_deg']),
        min_samples=int(parameters['cluster_min_samples'])))

    clustered_count = sum(len(cluster['points']) for cluster in clusters)
    diagnostics.info(
        f'NBV frontier coverage: {len(pipe_points)} input points -> '
        f'{len(cleaned_points)} surface points -> {len(frontier_indices)} frontiers -> '
        f'{clustered_count} clustered points in {len(clusters)} clusters')

    if not clusters:
        return []

    # Only clustered focus points contribute to gain. Keep their original k-nearest
    # neighbors in the full cloud, while avoiding eigensolves for unused points.
    gain_indices = np.unique(np.concatenate([cluster['global_indices'] for cluster in clusters]))
    complexities = timed('nbv_pca', lambda: calculate_pca_complexity(
        pcd, k=int(parameters['pca_neighbors']), indices=gain_indices, tree=pcd_tree))

    # 3. Candidate Generation & Raw Evaluation
    evaluation_started = clock.perf_counter()
    all_candidates = []
    min_raw_gain = float(parameters['min_raw_gain'])
    observation_positions = np.asarray(
        [sample[0] for sample in observation_history], dtype=float).reshape(-1, 3)
    observation_yaws = np.asarray([sample[1] for sample in observation_history])
    visited_clusters = generated_candidates = gain_rejections = 0

    for cluster in clusters:
        vectors = cluster['centroid'] - observation_positions
        distances = np.linalg.norm(vectors, axis=1)
        angles = np.arctan2(vectors[:, 1], vectors[:, 0]) - observation_yaws
        yaw_diffs = np.abs(np.arctan2(np.sin(angles), np.cos(angles)))
        
        # 1. Find historical poses that were close enough and looking the right way
        potential_obs_indices = np.flatnonzero(
            (distances > parameters['visited_min_distance']) &
            (distances < parameters['visited_max_distance']) &
            (yaw_diffs < np.radians(parameters['visited_yaw_threshold_deg']))
        )
        
        already_visited = False
        
        # 2. Shoot a ray from those historical poses to the target to check for occlusion
        for obs_idx in potential_obs_indices:
            obs_pos = observation_positions[obs_idx]
            ray_vec = cluster['centroid'] - obs_pos
            ray_dist = distances[obs_idx]
            
            # If we were extremely close, assume it was visible
            if ray_dist <= 0.6: 
                already_visited = True
                break
                
            # Sample points along the line-of-sight (every 0.2m)
            # Stop 0.4m short of the target so we don't accidentally collide with the target itself
            num_samples = int((ray_dist - 0.4) / 0.2)
            if num_samples < 1:
                already_visited = True
                break
                
            fractions = np.linspace(0.2 / ray_dist, (ray_dist - 0.4) / ray_dist, num_samples)
            ray_samples = obs_pos + fractions[:, None] * ray_vec
            
            # Ask the KDTree how close our ray got to the nearest mapped obstacle
            dists, _ = occupied_tree.query(ray_samples, k=1)
            
            # If the ray never gets closer than 0.3m to an obstacle, the line of sight was clear!
            if np.min(dists) > 0.3:
                already_visited = True
                break
                
        if already_visited:
            visited_clusters += 1
            continue

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
        generated_candidates += len(candidates)

        if not candidates:
            continue
        positions = np.asarray([candidate['position'] for candidate in candidates])
        gains = evaluate_viewpoints(
            positions, cluster['points'], cluster['normals'], c_complexities,
            complexity_weight=float(parameters['complexity_weight']))
        accepted = gains >= min_raw_gain
        gain_rejections += int(np.count_nonzero(~accepted))
        if not accepted.any():
            continue
        positions, gains = positions[accepted], gains[accepted]
        costs = calculate_kinematic_costs(
            auv_position, auv_heading, positions, cluster['centroid'], pcd_tree,
            distance_weight=float(parameters['distance_weight']),
            rotation_weight=float(parameters['rotation_weight']),
            collision_weight=float(parameters['collision_weight']),
            collision_distance_threshold=float(parameters['collision_distance_threshold']),
            collision_distance_offset=float(parameters['collision_distance_offset']))

        for position, gain, cost in zip(positions, gains, costs):
            all_candidates.append({
                'cluster_id': cluster['id'],
                'target': cluster['centroid'],
                'position': position,
                'raw_gain': gain,
                'raw_cost': cost
            })

    diagnostics.info(
        f'NBV candidate filtering: {visited_clusters}/{len(clusters)} clusters '
        f'suppressed by pose history; {generated_candidates} viewpoints generated, '
        f'{gain_rejections} below minimum raw gain, {len(all_candidates)} retained')
    # No surviving candidate is a failed planning pass, not proof of full coverage.
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
