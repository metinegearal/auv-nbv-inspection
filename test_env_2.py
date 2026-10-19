import argparse
import csv
import os
from pathlib import Path
import sys

# Open3D's native Wayland startup can load a GTK/libdecor plugin that expects
# obsolete GNOME font settings and then fails to initialize GLEW. Use XWayland
# when available, before importing Open3D or initializing any GUI libraries.
if sys.platform == "linux" and os.environ.get("DISPLAY") and (
        os.environ.get("WAYLAND_DISPLAY") or os.environ.get("XDG_SESSION_TYPE") == "wayland"):
    os.environ["GDK_BACKEND"] = "x11"
    os.environ["XDG_SESSION_TYPE"] = "x11"
    os.environ.pop("WAYLAND_DISPLAY", None)

import numpy as np
import open3d as o3d
from scipy.spatial import KDTree
import pyoctomap as octomap

PROJECT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_DIR / "src" / "auv_planner_reference"))
sys.path.insert(0, str(PROJECT_DIR / "src" / "auv_benchmark"))
from auv_benchmark.adapters.permanent_bans import (
    BAN_SPHERES_FILENAME, PermanentBanRecorder, load_ban_spheres, target_is_banned,
)
from auv_planner_reference.domain.nbv_parameters import DEFAULT_NBV_PARAMETERS
from auv_planner_reference.domain.seabed import SeabedEstimate
from auv_planner_reference.use_cases.nbv import (
    calculate_pca_complexity, detect_frontiers, cluster_frontiers,
    generate_cone_viewpoints, evaluate_viewpoint, calculate_kinematic_cost,
    normalize_array, select_upward_view_axis,
)
from auv_planner_reference.route_planning import (
    inflate_occupied, plan_multi_waypoint_route, generate_smooth_trajectory,
    resample_trajectory_by_distance, path_is_collision_free,
)

# ==========================================
# 1. SYNTHETIC ENVIRONMENT (WITH HOLE & AUV)
# ==========================================
def load_real_octomap(filepath, auv_pos, return_map=False):
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
    z_min_target = DEFAULT_NBV_PARAMETERS['z_min_target']
    pipe_points = points[points[:, 2] > z_min_target]
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pipe_points)
    
    # 2. Clean up floating noise (critical to prevent fake frontiers)
    pcd = pcd.voxel_down_sample(DEFAULT_NBV_PARAMETERS['voxel_downsample_size'])
    pcd, _ = pcd.remove_statistical_outlier(
        nb_neighbors=DEFAULT_NBV_PARAMETERS['outlier_neighbors'],
        std_ratio=DEFAULT_NBV_PARAMETERS['outlier_std_ratio'])
    if len(pcd.points) < 40:
        raise ValueError("Fewer than 40 surface points remain after preprocessing.")
    
    # 3. Estimate Normals
    # We use a slightly larger radius (0.4m) to smooth out the blocky OctoMap voxels
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=DEFAULT_NBV_PARAMETERS['normal_radius'],
        max_nn=DEFAULT_NBV_PARAMETERS['normal_max_neighbors']))
    
    # 4. Orient Normals Outward
    # By giving Open3D the AUV's position, it ensures normals face the cameras
    pcd.orient_normals_towards_camera_location(auv_pos)
    
    if return_map:
        return pcd, points, tree.getResolution()
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

def load_latest_trajectory(results_dir, csv_path=None):
    """Read the newest recorded trajectory, accepting both CSV header styles."""
    if csv_path is None:
        candidates = sorted(results_dir.glob("trajectory_benchmark_*.csv"), reverse=True)
        if not candidates:
            candidates = [results_dir / "benchmark_trajectory.csv",
                          results_dir / "planner_trajectory.csv"]
        candidates = [path for path in candidates if path.is_file()]
    else:
        candidates = [Path(csv_path)]

    for path in candidates:
        positions = []
        with path.open(newline="") as stream:
            reader = csv.DictReader(stream)
            fields = {name.lower(): name for name in (reader.fieldnames or [])}
            if not all(name in fields for name in ("x_m", "y_m", "z_m")):
                raise ValueError(f"Missing trajectory coordinates in {path}")
            for row_number, row in enumerate(reader, start=2):
                try:
                    point = [float(row[fields[name]]) for name in ("x_m", "y_m", "z_m")]
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"Invalid coordinates in {path}:{row_number}") from exc
                if not np.all(np.isfinite(point)):
                    raise ValueError(f"Non-finite coordinates in {path}:{row_number}")
                positions.append(point)
        if positions:
            return path, np.asarray(positions)
    raise ValueError(f"No recorded trajectory points found in {results_dir}")


def trajectory_yaws(positions):
    """Estimate viewing yaw from horizontal travel; stationary samples retain yaw."""
    steps = np.diff(positions[:, :2], axis=0)
    moving = np.linalg.norm(steps, axis=1) > 1e-4
    yaws = np.zeros(len(positions))
    if np.any(moving):
        first = np.flatnonzero(moving)[0]
        yaw = np.arctan2(steps[first, 1], steps[first, 0])
        yaws[0] = yaw
        for index, step in enumerate(steps, start=1):
            if moving[index - 1]:
                yaw = np.arctan2(step[1], step[0])
            yaws[index] = yaw
    return yaws


def filter_observed_clusters(clusters, trajectory, yaws, parameters):
    """Apply the planner's distance/yaw coverage rule to recorded trajectory poses."""
    remaining, observed = [], []
    for cluster in clusters:
        vectors = cluster['centroid'] - trajectory
        distances = np.linalg.norm(vectors, axis=1)
        angles = np.arctan2(vectors[:, 1], vectors[:, 0]) - yaws
        yaw_diffs = np.abs(np.arctan2(np.sin(angles), np.cos(angles)))
        already_visited = np.any(
            (distances > parameters['visited_min_distance'])
            & (distances < parameters['visited_max_distance'])
            & (yaw_diffs < np.radians(parameters['visited_yaw_threshold_deg'])))
        (observed if already_visited else remaining).append(cluster)
    return remaining, observed


def add_sphere(geometries, position, radius, color):
    # Tiny candidate markers do not need the default 1,520-triangle sphere.
    sphere = o3d.geometry.TriangleMesh.create_sphere(
        radius=radius, resolution=6 if radius <= 0.08 else 12)
    sphere.compute_vertex_normals()
    sphere.translate(position)
    sphere.paint_uniform_color(color)
    geometries.append(sphere)

def add_ban_spheres(geometries, spheres):
    """Draw wire volumes so the mapped surface remains visible inside each ban."""
    for value in spheres:
        mesh = o3d.geometry.TriangleMesh.create_sphere(radius=value['radius_m'], resolution=12)
        mesh.translate(value['center'])
        wire = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
        wire.paint_uniform_color([.7, .05, .05])
        geometries.append(wire)

def load_viewer_bans(map_path, override=None):
    """Refresh the default benchmark snapshot; explicit snapshots stay frozen."""
    if override is None:
        recorder = PermanentBanRecorder(Path(map_path).parent)
        recorder.poll()
    path = Path(override) if override is not None else Path(map_path).parent / BAN_SPHERES_FILENAME
    return path, load_ban_spheres(path)


def batch_marker_meshes(geometries):
    """Combine sphere markers into one draw object, retaining per-vertex colors."""
    meshes = [g for g in geometries if isinstance(g, o3d.geometry.TriangleMesh)]
    if not meshes:
        return geometries
    offsets = np.cumsum([0] + [len(mesh.vertices) for mesh in meshes[:-1]])
    markers = o3d.geometry.TriangleMesh()
    markers.vertices = o3d.utility.Vector3dVector(
        np.concatenate([np.asarray(mesh.vertices) for mesh in meshes]))
    markers.triangles = o3d.utility.Vector3iVector(np.concatenate([
        np.asarray(mesh.triangles) + offset for mesh, offset in zip(meshes, offsets)]))
    markers.vertex_colors = o3d.utility.Vector3dVector(
        np.concatenate([np.asarray(mesh.vertex_colors) for mesh in meshes]))
    markers.vertex_normals = o3d.utility.Vector3dVector(
        np.concatenate([np.asarray(mesh.vertex_normals) for mesh in meshes]))
    return [g for g in geometries if not isinstance(g, o3d.geometry.TriangleMesh)] + [markers]


def calculate_future_trajectory(auv_position, auv_yaw, winners, occupied_points, resolution):
    """Preview the planner's baseline route/spline pipeline on the full map.

    Uses the baseline settings from domain/configuration.py. The result is an
    untimed XYZ/yaw preview, using NBV camera positions as route waypoints.
    """
    eligible = [winner for winner in winners if winner['utility'] >= 0.005]
    candidates = [(winner['position'], winner['target']) for winner in eligible]
    if not candidates:
        print("Future trajectory: no winners pass the planner's minimum utility.")
        return np.empty((0, 4))
    # Preserve raw map obstacles, including points removed during NBV preprocessing.
    occupied_indices = set(map(tuple, np.floor(occupied_points / resolution).astype(int)))
    occupied = inflate_occupied(occupied_indices, collision_radius=0.6 + 0.4,
                               resolution=resolution)
    route, targets = plan_multi_waypoint_route(
        auv_position, candidates, occupied, resolution, astar_max_steps=50000,
        reversal_weight=0.0,
        candidate_utilities=[winner['utility'] for winner in eligible])
    if not len(route) or not path_is_collision_free(route, occupied, resolution):
        print("Future trajectory: no collision-free A* route found.")
        return np.empty((0, 4))

    options = [(route, targets)]
    target_changes = np.flatnonzero(np.any(targets != targets[0], axis=1))
    if len(target_changes) and target_changes[0] >= 4:
        end = int(target_changes[0])
        options.append((route[:end], targets[:end]))
    for option_route, option_targets in options:
        trajectory = generate_smooth_trajectory(
            option_route, option_targets, num_points=300,
            smoothing=2.0, sampling_spacing=0.5)
        if not len(trajectory):
            continue
        trajectory[0, :3], trajectory[-1, :3] = option_route[0], option_route[-1]
        trajectory[0, 3] = auv_yaw
        trajectory = resample_trajectory_by_distance(trajectory, 0.5, max_points=300)
        if path_is_collision_free(trajectory[:, :3], occupied, resolution):
            length = np.linalg.norm(np.diff(trajectory[:, :3], axis=0), axis=1).sum()
            print(f"Future trajectory: {len(trajectory)} points, {length:.2f} m; collision checks passed.")
            return trajectory
    # For visualization, retain the validated A* polyline when the planner rejects
    # its spline. This is a route preview, not a controller-ready smooth trajectory.
    yaw = np.unwrap(np.arctan2(targets[:, 1] - route[:, 1], targets[:, 0] - route[:, 0]))
    yaw[0] = auv_yaw
    length = np.linalg.norm(np.diff(route, axis=0), axis=1).sum()
    print(f"Future trajectory: spline rejected; showing collision-free A* route preview "
          f"({len(route)} points, {length:.2f} m).")
    return np.column_stack((route, yaw))


def main():
    parser = argparse.ArgumentParser(description="Visualize NBV selection after the latest trajectory.")
    parser.add_argument("--trajectory-csv", type=Path, help="Override the latest trajectory CSV.")
    parser.add_argument("--map", type=Path, default=PROJECT_DIR / "results" / "final_map_ros2.bt")
    parser.add_argument("--ban-spheres", type=Path,
                        help="Saved permanent ban spheres (default: alongside the map).")
    parser.add_argument("--no-gui", action="store_true", help="Evaluate without opening Open3D.")
    parser.add_argument("--future-trajectory", action=argparse.BooleanOptionalAction,
                        default=False, help="Calculate and show the planner's future path (default: off).")
    args = parser.parse_args()
    ban_path, ban_spheres = load_viewer_bans(args.map, args.ban_spheres)
    print(f"Permanent ban spheres: {len(ban_spheres)} from {ban_path}")
    parameters = DEFAULT_NBV_PARAMETERS
    trajectory_path, trajectory = load_latest_trajectory(PROJECT_DIR / "results", args.trajectory_csv)
    yaws = trajectory_yaws(trajectory)
    auv_position = trajectory[-1].copy()
    auv_heading = np.array([np.cos(yaws[-1]), np.sin(yaws[-1]), 0.0])
    print(f"Trajectory: {trajectory_path.name} ({len(trajectory)} points)")
    print(f"Last point / current AUV position: {auv_position}")
    print("CSV has no measured yaw: viewing headings are estimated from horizontal travel.")

    pcd, occupied_points, resolution = load_real_octomap(str(args.map), auv_position, return_map=True)
    seabed = SeabedEstimate(resolution)
    if parameters['seabed_enabled']:
        seabed.update(occupied_points)
        print(f"Seabed floor Z from map: {seabed.mean_z}; map samples: {seabed.sample_count}")
    points, normals = np.asarray(pcd.points), np.asarray(pcd.normals)
    print(f"Loaded {len(points)} valid pipe voxels.")
    complexities = calculate_pca_complexity(pcd, k=parameters['pca_neighbors'])
    frontier_indices = detect_frontiers(
        pcd, k=parameters['frontier_neighbors'],
        angle_threshold_deg=80,
        min_neighbors=2)
    clusters = cluster_frontiers(
        points, normals, frontier_indices, fov_radius=parameters['cluster_fov_radius'],
        max_angle_deg=parameters['cluster_max_angle_deg'],
        min_samples=parameters['cluster_min_samples'])
    clusters, observed = filter_observed_clusters(clusters, trajectory, yaws, parameters)
    print(f"Coverage filter: {len(observed)} observed clusters removed; {len(clusters)} remain.")
    banned_clusters = [c for c in clusters if target_is_banned(c['centroid'], ban_spheres)]
    clusters = [c for c in clusters if not target_is_banned(c['centroid'], ban_spheres)]
    print(f"Permanent ban filter: {len(banned_clusters)} clusters removed; {len(clusters)} remain.")

    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    geometries = [pcd]
    add_ban_spheres(geometries, ban_spheres)
    path = o3d.geometry.LineSet()
    path.points = o3d.utility.Vector3dVector(trajectory)
    segments = np.column_stack((np.arange(len(trajectory) - 1), np.arange(1, len(trajectory))))
    path.lines = o3d.utility.Vector2iVector(segments)
    path.colors = o3d.utility.Vector3dVector(np.tile([0, 0.7, 0], (len(segments), 1)))
    geometries.append(path)
    add_sphere(geometries, trajectory[0], 0.1, [0, 0.7, 0])
    add_sphere(geometries, auv_position, 0.2, [0, 0.5, 1])

    # Show only frontier points belonging to clusters that survived coverage filtering.
    remaining_indices = [index for cluster in clusters for index in cluster['global_indices']]
    if remaining_indices:
        frontiers = o3d.geometry.PointCloud()
        frontiers.points = o3d.utility.Vector3dVector(points[remaining_indices])
        frontiers.paint_uniform_color([1, 0, 0])
        geometries.append(frontiers)

    pcd_tree = KDTree(points)
    occupied_tree = KDTree(occupied_points)
    all_candidates = []
    for cluster in clusters:
        view_axis = select_upward_view_axis(
            cluster['centroid'], auv_position, occupied_tree,
            standoff=parameters['standoff'], azimuth_steps=parameters['view_axis_azimuth_steps'])
        candidates = generate_cone_viewpoints(
            cluster['centroid'], view_axis, standoff=parameters['standoff'],
            polar_angles_deg=parameters['polar_angles_deg'], azimuth_steps=parameters['azimuth_steps'])
        for candidate in candidates:
            gain = evaluate_viewpoint(
                candidate['position'], cluster['points'], cluster['normals'],
                complexities[cluster['global_indices']], complexity_weight=parameters['complexity_weight'],
                seabed_z=seabed.mean_z if parameters['seabed_enabled'] else None,
                seabed_decay_height=parameters['seabed_decay_height'],
                seabed_penalty_strength=parameters['seabed_penalty_strength'])
            if gain < parameters['min_raw_gain']:
                continue
            cost = calculate_kinematic_cost(
                auv_position, auv_heading, candidate['position'], cluster['centroid'], pcd_tree,
                distance_weight=parameters['distance_weight'], rotation_weight=parameters['rotation_weight'],
                collision_weight=parameters['collision_weight'],
                collision_distance_threshold=parameters['collision_distance_threshold'],
                collision_distance_offset=parameters['collision_distance_offset'])
            all_candidates.append({**candidate, 'cluster_id': cluster['id'],
                                   'target': cluster['centroid'], 'raw_gain': gain, 'raw_cost': cost})
            if not args.no_gui:
                add_sphere(geometries, candidate['position'], 0.04, [1, 0, 1])

    winners = {}
    ranked = []
    best = None
    if all_candidates:
        gains = normalize_array([candidate['raw_gain'] for candidate in all_candidates])
        costs = normalize_array([candidate['raw_cost'] for candidate in all_candidates])
        for candidate, gain, cost in zip(all_candidates, gains, costs):
            candidate['utility'] = (parameters['utility_gain_weight'] * gain
                                    - parameters['utility_cost_weight'] * cost)
            cid = candidate['cluster_id']
            if cid not in winners or candidate['utility'] > winners[cid]['utility']:
                winners[cid] = candidate
        ranked = sorted(winners.values(), key=lambda winner: winner['utility'], reverse=True)
        for winner in ranked:
            print(f"Cluster {winner['cluster_id']:>2} | Utility: {winner['utility']:.3f} | "
                  f"Gain: {winner['raw_gain']:.3f} | Cost: {winner['raw_cost']:.3f}")
            add_sphere(geometries, winner['position'], 0.08, [1, 1, 0])
        best = ranked[0]
        print(f"Global best NBV: cluster {best['cluster_id']} at {best['position']}")
        add_sphere(geometries, best['position'], 0.15, [0, 1, 1])
        connection = o3d.geometry.LineSet()
        connection.points = o3d.utility.Vector3dVector([auv_position, best['position']])
        connection.lines = o3d.utility.Vector2iVector([[0, 1]])
        connection.colors = o3d.utility.Vector3dVector([[0, 1, 1]])
        geometries.append(connection)
    else:
        print("No NBV candidates remain after coverage and gain filtering.")

    if args.future_trajectory:
        print("Calculating future trajectory with auv_planner_reference...")
        future = calculate_future_trajectory(
            auv_position, yaws[-1], ranked, occupied_points, resolution)
        if len(future) > 1:
            future_path = o3d.geometry.LineSet()
            future_path.points = o3d.utility.Vector3dVector(future[:, :3])
            segments = np.column_stack((np.arange(len(future) - 1), np.arange(1, len(future))))
            future_path.lines = o3d.utility.Vector2iVector(segments)
            future_path.colors = o3d.utility.Vector3dVector(np.tile([1, 0.4, 0], (len(segments), 1)))
            geometries.append(future_path)
            add_sphere(geometries, future[-1, :3], 0.12, [1, 0.4, 0])

    if not args.no_gui:
        print("Gray: Surface | Green: Trajectory/start | Blue: Last point/AUV | Red: Remaining frontiers")
        print("Magenta: Candidates | Yellow: Cluster winners | Cyan: Global best NBV | Labels: Cluster IDs")
        print("Dark red wire spheres: Permanent ban regions (saved map centres and radii)")
        if args.future_trajectory:
            print("Orange: Collision-checked future trajectory/end")

        def add_cluster_labels(visualizer):
            pass
            # for sphere in ban_spheres:
            #     radius = sphere['radius_m']
            #     visualizer.add_3d_label(np.asarray(sphere['center']) + [0, 0, radius + .1],
            #                            f"Permanent ban ({radius:g} m)")
            # for winner in winners.values():
            #     label_position = winner['position'] + np.array([0.0, 0.0, 0.25])
            #     label = f"Cluster {winner['cluster_id']}"
            #     if winner is best:
            #         label += " (best)"
            #     visualizer.add_3d_label(label_position, label)

        # The GUI visualizer supports screen-facing text anchored to 3D positions.
        o3d.visualization.draw(
            geometries, title="NBV cluster winners", raw_mode=True,
            bg_color=(1.0, 1.0, 1.0, 1.0), show_skybox=False,
            line_width=3, on_init=add_cluster_labels)


if __name__ == "__main__":
    main()
