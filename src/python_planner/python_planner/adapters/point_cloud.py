from dataclasses import dataclass
import numpy as np
import open3d as o3d
from ..domain.nbv_parameters import MIN_NBV_CLOUD_POINTS

@dataclass(frozen=True)
class CloudSnapshot:
    points: object
    normals: object

class Open3DPreprocessor:
    def prepare(self, pipe_points, auv_position, parameters, timed):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pipe_points)
        pcd = timed('nbv_downsample', lambda: pcd.voxel_down_sample(
            voxel_size=float(parameters['voxel_downsample_size'])))
        pcd, _ = timed('nbv_outlier_filter', lambda: pcd.remove_statistical_outlier(
            nb_neighbors=int(parameters['outlier_neighbors']),
            std_ratio=float(parameters['outlier_std_ratio'])))

        if len(pcd.points) < MIN_NBV_CLOUD_POINTS:
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
