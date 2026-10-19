import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data

import numpy as np
import time
import traceback

from sensor_msgs.msg import Image, CameraInfo, Imu, MagneticField
from std_msgs.msg import Float32, Header, Float32MultiArray
from geometry_msgs.msg import TwistStamped, Vector3, Quaternion, Pose, Point
from nav_msgs.msg import Odometry

# cv_bridge for numpy <-> ROS Image
from cv_bridge import CvBridge

# Helper quaternion conversion
import math
import json


class RosSensorOutputs:
    def __init__(self, publishers, bridge, clock, diagnostics, camera_info):
        for name, publisher in publishers.items():
            setattr(self, name, publisher)
        self.bridge = bridge
        self.clock = clock
        self.diagnostics = diagnostics
        self.default_cam_info = camera_info

    def _stamp(self):
        # ROS2 builtin time
        return self.clock.now().to_msg()


    def _publish_camera(self, np_img):
        """
        np_img: H x W x 3 (uint8) or H x W x 4
        Publishes sensor_msgs/Image and CameraInfo.
        """
        try:
            if np_img is None:
                return
            # Ensure uint8 and 3 channels
            if np_img.dtype != np.uint8:
                # attempt normalization/convert
                np_img = np.clip(np_img, 0, 255).astype(np.uint8)

            # if 4 channels, drop alpha
            if np_img.ndim == 3 and np_img.shape[2] == 4:
                np_img = np_img[:, :, :3]

            # cv_bridge expects BGR by default. HoloOcean likely gives RGB.
            # We'll convert RGB -> BGR for cv2 encoding then specify rgb8 after swapping back.
            # Simpler: tell bridge encoding='rgb8' and pass numpy RGB directly.
            header = Header()
            header.stamp = self._stamp()
            header.frame_id = 'camera_link'

            img_msg = self.bridge.cv2_to_imgmsg(np_img, encoding='rgb8')
            img_msg.header.stamp = header.stamp
            img_msg.header.frame_id = header.frame_id
            self.camera_pub.publish(img_msg)

            # CameraInfo
            cam_info = CameraInfo()
            cam_info.header = img_msg.header
            cam_info.width = np_img.shape[1]
            cam_info.height = np_img.shape[0]
            cam_info.distortion_model = self.default_cam_info.distortion_model
            cam_info.k = self.default_cam_info.k
            cam_info.d = self.default_cam_info.d
            cam_info.r = self.default_cam_info.r
            cam_info.p = self.default_cam_info.p
            cam_info.binning_x = self.default_cam_info.binning_x
            cam_info.binning_y = self.default_cam_info.binning_y
            self.cam_info_pub.publish(cam_info)
        except Exception as e:
            self.diagnostics.error("Failed to publish camera: %s" % str(e))
            self.diagnostics.debug(traceback.format_exc())


    def _publish_camera_down(self, np_img):
        """
        np_img: H x W x 3 (uint8) or H x W x 4
        Publishes sensor_msgs/Image and CameraInfo.
        """
        try:
            if np_img is None:
                return
            # Ensure uint8 and 3 channels
            if np_img.dtype != np.uint8:
                # attempt normalization/convert
                np_img = np.clip(np_img, 0, 255).astype(np.uint8)

            # if 4 channels, drop alpha
            if np_img.ndim == 3 and np_img.shape[2] == 4:
                np_img = np_img[:, :, :3]

            # cv_bridge expects BGR by default. HoloOcean likely gives RGB.
            # We'll convert RGB -> BGR for cv2 encoding then specify rgb8 after swapping back.
            # Simpler: tell bridge encoding='rgb8' and pass numpy RGB directly.
            header = Header()
            header.stamp = self._stamp()
            header.frame_id = 'camera2_link'

            img_msg = self.bridge.cv2_to_imgmsg(np_img, encoding='rgb8')
            img_msg.header.stamp = header.stamp
            img_msg.header.frame_id = header.frame_id
            self.cameraDown_pub.publish(img_msg)
            # self.diagnostics.info("Published Down Camera Image")
            # self.diagnostics.info(f"Image header: {img_msg.header.frame_id}")

            # CameraInfo
            # cam_info = CameraInfo()
            # cam_info.header = img_msg.header
            # cam_info.width = np_img.shape[1]
            # cam_info.height = np_img.shape[0]
            # cam_info.distortion_model = self.default_cam_info.distortion_model
            # cam_info.k = self.default_cam_info.k
            # cam_info.d = self.default_cam_info.d
            # cam_info.r = self.default_cam_info.r
            # cam_info.p = self.default_cam_info.p
            # cam_info.binning_x = self.default_cam_info.binning_x
            # cam_info.binning_y = self.default_cam_info.binning_y

            # self.cam_info_pub.publish(cam_info)
        except Exception as e:
            self.diagnostics.error("Failed to publish camera: %s" % str(e))
            self.diagnostics.debug(traceback.format_exc())


    def _publish_depth(self, depth_data):
        """
        depth_data can be:
         - 2D numpy array -> publish as sensor_msgs/Image (32FC1 or 16UC1)
         - scalar (float) -> publish as std_msgs/Float32
        """
        try:
            if depth_data is None:
                return
            if isinstance(depth_data, (float, int, np.floating, np.integer)):
                msg = Float32()
                msg.data = float(depth_data)
                self.depth_scalar_pub.publish(msg)
                return

            depth_np = np.array(depth_data)
            if depth_np.ndim == 2:
                # publish as 32-bit float image
                # ensure float32
                depth_f = depth_np.astype(np.float32)
                img_msg = self.bridge.cv2_to_imgmsg(depth_f, encoding='32FC1')
                img_msg.header.stamp = self._stamp()
                img_msg.header.frame_id = 'depth_link'
                self.depth_image_pub.publish(img_msg)
            elif depth_np.ndim == 3 and depth_np.shape[2] == 1:
                depth_f = depth_np[:, :, 0].astype(np.float32)
                img_msg = self.bridge.cv2_to_imgmsg(depth_f, encoding='32FC1')
                img_msg.header.stamp = self._stamp()
                img_msg.header.frame_id = 'depth_link'
                self.depth_image_pub.publish(img_msg)
            else:
                # fallback: try scalar conversion
                try:
                    val = float(depth_np[0])
                    msg = Float32()
                    msg.data = val
                    self.depth_scalar_pub.publish(msg)
                except Exception:
                    self.diagnostics.warn("Unsupported depth shape: %s" % (str(depth_np.shape),))
        except Exception as e:
            self.diagnostics.error("Failed to publish depth: %s" % str(e))
            self.diagnostics.debug(traceback.format_exc())


    def _publish_imu(self, imu_data):
        """imu_data expected as dict/array: ax,ay,az; gx,gy,gz; maybe orientation"""
        try:
            if imu_data is None:
                return
            msg = Imu()
            msg.header.stamp = self._stamp()
            msg.header.frame_id = 'imu_link'
            # try various formats
            if isinstance(imu_data, dict):
                lin_acc = imu_data.get('linear_acceleration') or imu_data.get('acc') or imu_data.get('accelerometer')
                ang_vel = imu_data.get('angular_velocity') or imu_data.get('gyro')
                ori = imu_data.get('orientation')
            else:
                # maybe a numpy array [ax,ay,az, gx,gy,gz] or nested
                try:
                    arr = np.array(imu_data).flatten()
                    if arr.size >= 6:
                        lin_acc = arr[0:3].tolist()
                        ang_vel = arr[3:6].tolist()
                    else:
                        lin_acc = None
                        ang_vel = None
                    ori = None
                except Exception:
                    lin_acc = ang_vel = ori = None

            if lin_acc is not None:
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = [float(x) for x in lin_acc]
            if ang_vel is not None:
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = [float(x) for x in ang_vel]
            if ori is not None:
                # accept list/tuple [x,y,z,w] or dict
                if isinstance(ori, dict):
                    msg.orientation.x = float(ori.get('x', 0.0))
                    msg.orientation.y = float(ori.get('y', 0.0))
                    msg.orientation.z = float(ori.get('z', 0.0))
                    msg.orientation.w = float(ori.get('w', 1.0))
                else:
                    q = list(ori)
                    if len(q) >= 4:
                        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = [float(x) for x in q[:4]]
            self.imu_pub.publish(msg)
        except Exception as e:
            self.diagnostics.error("Failed to publish IMU: %s" % str(e))
            self.diagnostics.debug(traceback.format_exc())


    def _publish_mag(self, mag_data):
        try:
            if mag_data is None:
                return
            msg = MagneticField()
            msg.header.stamp = self._stamp()
            msg.header.frame_id = 'mag_link'
            # accept sequences
            try:
                v = np.array(mag_data).flatten()
                msg.magnetic_field.x = float(v[0])
                msg.magnetic_field.y = float(v[1])
                msg.magnetic_field.z = float(v[2])
            except Exception:
                # if dict:
                if isinstance(mag_data, dict):
                    msg.magnetic_field.x = float(mag_data.get('x', 0.0))
                    msg.magnetic_field.y = float(mag_data.get('y', 0.0))
                    msg.magnetic_field.z = float(mag_data.get('z', 0.0))
            self.mag_pub.publish(msg)
        except Exception as e:
            self.diagnostics.error("Failed to publish magnetometer: %s" % str(e))
            self.diagnostics.debug(traceback.format_exc())


    def _publish_dvl(self, dvl_data):
        """
        dvl_data: velocity vector or dict with 'vx','vy','vz'
        Publishes geometry_msgs/TwistStamped (linear velocity in m/s)
        """
        try:
            if dvl_data is None:
                return
            msg = TwistStamped()
            msg.header.stamp = self._stamp()
            msg.header.frame_id = 'dvl_link'
            # try:
            v = np.array(dvl_data)[:3]
            msg.twist.linear.x = float(v[0])
            msg.twist.linear.y = float(v[1]) if v.size > 1 else 0.0
            msg.twist.linear.z = float(v[2]) if v.size > 2 else 0.0
            # except Exception:
            #     if isinstance(dvl_data, dict):
            #         msg.twist.linear.x = float(dvl_data.get('vx', 0.0))
            #         msg.twist.linear.y = float(dvl_data.get('vy', 0.0))
            #         msg.twist.linear.z = float(dvl_data.get('vz', 0.0))
            self.dvl_pub.publish(msg)

            dist = np.average(np.array(dvl_data)[3:])
            self.depth_distance_pub.publish(Float32(data=float(dist)))

        except Exception as e:
            self.diagnostics.error("Failed to publish DVL: %s" % str(e))
            self.diagnostics.debug(traceback.format_exc())


    def _publish_range_finder(self, range_data):
        """
        range_data: scalar distance value (float)
        Publishes std_msgs/Float32
        """
        try:
            if range_data is None:
                return
            msg = Float32()
            msg.data = float(range_data)
            self.range_finder_pub.publish(msg)
        except Exception as e:
            self.diagnostics.error("Failed to publish range finder: %s" % str(e))
            self.diagnostics.debug(traceback.format_exc())


    def _publish_location(self, loc_data):
        """
        loc_data expected like [x,y,z] plus maybe orientation or velocity.
        Publishes nav_msgs/Odometry (pose + optionally twist)
        """
        try:
            if loc_data is None:
                return
            msg = Odometry()
            msg.header.stamp = self._stamp()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'
            # accept dict or array
            if isinstance(loc_data, dict):
                pos = loc_data.get('position') or loc_data.get('pos') or loc_data.get('location')
                ori = loc_data.get('orientation')
                vel = loc_data.get('velocity')
            else:
                arr = np.array(loc_data)
                # heuristics
                if arr.size >= 3:
                    pos = arr[0:3].tolist()
                    vel = arr[3:6].tolist() if arr.size >= 6 else None
                    ori = None
                else:
                    pos = None
                    vel = None
                    ori = None

            if pos is not None:
                msg.pose.pose.position.x = float(pos[0])
                msg.pose.pose.position.y = float(pos[1])
                msg.pose.pose.position.z = float(pos[2])
            if ori is not None:
                if isinstance(ori, dict):
                    msg.pose.pose.orientation.x = float(ori.get('x', 0.0))
                    msg.pose.pose.orientation.y = float(ori.get('y', 0.0))
                    msg.pose.pose.orientation.z = float(ori.get('z', 0.0))
                    msg.pose.pose.orientation.w = float(ori.get('w', 1.0))
                else:
                    q = list(ori)
                    if len(q) >= 4:
                        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = [float(x) for x in q[:4]]
            if vel is not None:
                msg.twist.twist.linear.x = float(vel[0]) if len(vel) > 0 else 0.0
                msg.twist.twist.linear.y = float(vel[1]) if len(vel) > 1 else 0.0
                msg.twist.twist.linear.z = float(vel[2]) if len(vel) > 2 else 0.0

            self.odom_pub.publish(msg)
        except Exception as e:
            self.diagnostics.error("Failed to publish location/odom: %s" % str(e))
            self.diagnostics.debug(traceback.format_exc())

