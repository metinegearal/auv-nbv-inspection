from .compute_depth import compute_stereo_depth
from std_msgs.msg import Float32, Header, Float32MultiArray
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, Imu, MagneticField, FluidPressure
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
import numpy as np
import pandas as pd
import cv2
from cv_bridge import CvBridge
# from myMessages.msg import objectDetected2D

from collections import deque


class obstacleAvoidance(Node):
    def __init__(self):
        super().__init__('object_detector_node')

        self.bridge = CvBridge()

        self.frame = None  # Initialize the secondary camera frame to None

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # self.sub_cam = self.create_subscription(Image, 'camera/image_processed', self.cam_callback, 10)
        self.sub_cam = self.create_subscription(Image, 'holocean/camera/image_raw', self.cam_callback, 10)
        self.sub_cam_2 = self.create_subscription(Image, 'holocean/cameraDown/image_raw', self.cam_secondary_callback, 10)

        self.depth_image_pub = self.create_publisher(Image, 'holocean/depth/image_raw', qos)

        self.get_logger().info("✅ obstacle_detector_node Started")

    def _stamp(self):
        """
        Returns a ROS 2 Time message for the current time.
        """
        return self.get_clock().now().to_msg()

    def cam_callback(self, msg):
        if self.frame is None:
            self.get_logger().warn("Secondary camera frame not yet received. Cannot compute depth.")
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        depth = compute_stereo_depth(self.frame, frame, focal_length_px=256, baseline_meters=0.5)
        self._publish_depth(depth)
        # self.get_logger().info("Depth map computed and published.")

    def cam_secondary_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.frame=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        # self.get_logger().info("Secondary camera frame received and stored.")

    def _publish_depth(self, depth_data):
        """
        depth_data can be:
            - 2D numpy array -> publish as sensor_msgs/Image (32FC1 or 16UC1)
            - scalar (float) -> publish as std_msgs/Float32
        """
        try:
            if depth_data is None:
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
                # try:
                #     val = float(depth_np[0])
                #     msg = Float32()
                #     msg.data = val
                #     self.depth_scalar_pub.publish(msg)
                # except Exception:
                self.get_logger().warn("Unsupported depth shape: %s" % (str(depth_np.shape),))
        except Exception as e:
            self.get_logger().error("Failed to publish depth: %s" % str(e))


def main(args=None):
    rclpy.init(args=args)
    node = obstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
