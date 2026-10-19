from dataclasses import dataclass
import numpy as np
import open3d as o3d
from scipy.spatial import KDTree

@dataclass(frozen=True)
class CloudSnapshot:
    points: object
    normals: object

class Open3DPreprocessor:
    def prepare(self, pipe_points, auv_position, parameters, timed):
        pipe_points = np.asarray(pipe_points, dtype=float)
        pipe_points = pipe_points[np.isfinite(pipe_points).all(axis=1)]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pipe_points)
        pcd = timed('nbv_downsample', lambda: pcd.voxel_down_sample(
            voxel_size=float(parameters['voxel_downsample_size'])))
        minimum_points = max(3, int(parameters['cluster_min_samples']),
                             int(parameters['frontier_min_neighbors']) + 1)
        if len(pcd.points) < minimum_points:
            return None

        def filter_noise():
            _, inliers = pcd.remove_statistical_outlier(
                nb_neighbors=int(parameters['outlier_neighbors']),
                std_ratio=float(parameters['outlier_std_ratio']))
            points = np.asarray(pcd.points)
            # Global statistical density rejects genuine corners and sparse
            # borders. Retain rejected samples supported by three other local
            # points at the normal-estimation scale; isolated noise stays out.
            distances, _ = KDTree(points).query(points, k=4)
            supported = distances[:, -1] <= float(parameters['normal_radius'])
            supported[np.asarray(inliers, dtype=int)] = True
            return pcd.select_by_index(np.flatnonzero(supported).tolist())

        pcd = timed('nbv_outlier_filter', filter_noise)

        if len(pcd.points) < minimum_points:
            return None

        # Normal generation (oriented towards AUV)
        timed('nbv_normal_estimation', lambda: pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=float(parameters['normal_radius']),
            max_nn=int(parameters['normal_max_neighbors']))))
        timed('nbv_normal_orientation', lambda: pcd.orient_normals_towards_camera_location(auv_position))


        points = np.asarray(pcd.points).copy()
        normals = np.asarray(pcd.normals).copy()
        points.setflags(write=False)
        normals.setflags(write=False)
        return CloudSnapshot(points, normals)
