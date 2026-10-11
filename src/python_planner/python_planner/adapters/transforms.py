"""Time-aware TF adapter for camera projection and inverse mount placement."""
import math

import numpy as np
from rclpy.time import Time
from tf2_ros import TransformException
from scipy.spatial.transform import Rotation


def components(transform):
    value = transform.transform
    t, q = value.translation, value.rotation
    return np.array([t.x, t.y, t.z]), Rotation.from_quat([q.x, q.y, q.z, q.w])


class TfCameraTransforms:
    def __init__(self, buffer, world_frame, camera_frame, body_frame):
        self.buffer = buffer
        self.world_frame = world_frame
        self.camera_frame = camera_frame
        self.body_frame = body_frame

    def project_depth(self, optical_points, stamp):
        # No latest-pose fallback: every projected image uses its capture time.
        if stamp.sec == 0 and stamp.nanosec == 0:
            raise TransformException('Depth image has no capture timestamp')
        transform = self.buffer.lookup_transform(
            self.world_frame, self.camera_frame,
            Time(seconds=stamp.sec, nanoseconds=stamp.nanosec))
        origin, rotation = components(transform)
        return rotation.apply(optical_points) + origin, origin

    def camera_pose(self):
        transform = self.buffer.lookup_transform(
            self.world_frame, self.camera_frame, Time())
        origin, rotation = components(transform)
        return origin, rotation.apply([0., 0., 1.])

    def vehicle_viewpoint(self, camera_position, target):
        # TF supplies the inverse extrinsic, including sensor orientation.
        transform = self.buffer.lookup_transform(
            self.camera_frame, self.body_frame, Time())
        translation, camera_from_body = components(transform)
        body_from_camera = camera_from_body.inv()
        mount = -body_from_camera.apply(translation)
        delta = np.asarray(target) - np.asarray(camera_position)
        radius = float(np.linalg.norm(delta[:2]))
        # The controller points body X at the target. In that yaw frame the
        # target-to-camera lateral displacement must cancel the mount's Y.
        # Solve radius * sin(target_yaw - body_yaw) = -mount_y exactly;
        # fixed-point iteration can oscillate or converge too slowly here.
        if radius < abs(mount[1]) - 1e-10:
            raise ValueError('Camera viewpoint is too close laterally for the vehicle mount')
        forward = math.sqrt(max(0., radius * radius - mount[1] * mount[1])) + mount[0]
        if forward <= 1e-10:
            raise ValueError('Camera viewpoint would put the target behind the vehicle')
        yaw = math.atan2(delta[1], delta[0])
        if radius > 1e-10:
            yaw += math.asin(float(np.clip(mount[1] / radius, -1., 1.)))
        world_from_camera = Rotation.from_euler('z', yaw) * body_from_camera
        return np.asarray(camera_position) + world_from_camera.apply(translation)
