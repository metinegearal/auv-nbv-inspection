from .compute_depth import compute_stereo_depth
from std_msgs.msg import Float32, Header, Float32MultiArray
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import cv2
from cv_bridge import CvBridge

# NEW: Import message_filters for stereo synchronization
import message_filters

class obstacleAvoidance(Node):
    def __init__(self):
        super().__init__('object_detector_node')

        self.bridge = CvBridge()

        # Optimize QoS to ONLY keep the absolute newest frame (queue size 1)
        # This prevents the node from getting "stuck" processing old backlog frames
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1 
        )

        # 1. Create message filter subscribers (they do not trigger callbacks directly)
        self.sub_cam_left = message_filters.Subscriber(
            self, Image, 'holocean/camera/image_raw', qos_profile=qos)
        self.sub_cam_right = message_filters.Subscriber(
            self, Image, 'holocean/cameraDown/image_raw', qos_profile=qos)

        # 2. Synchronize the two image streams based on their timestamps.
        # queue_size=2 keeps memory low, slop=0.05 allows 50ms of timestamp jitter.
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_cam_left, self.sub_cam_right], 
            queue_size=10, 
            slop=0.02  # 20ms tolerance prevents ghosting but allows sim jitter
        )
        self.ts.registerCallback(self.sync_cam_callback)

        self.depth_image_pub = self.create_publisher(Image, 'holocean/depth/image_raw', qos)

        self.get_logger().info("✅ obstacle_detector_node Started (Synchronized)")

    def sync_cam_callback(self, msg_left, msg_right):
        # This callback ONLY fires when a perfectly matched pair of images arrives
        
        # Convert Left Image
        frame_left = self.bridge.imgmsg_to_cv2(msg_left, desired_encoding='bgr8')
        frame_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2RGB)
        
        # Convert Right (Secondary) Image
        frame_right = self.bridge.imgmsg_to_cv2(msg_right, desired_encoding='bgr8')
        frame_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2RGB)

        # Compute Depth
        depth = compute_stereo_depth(frame_right, frame_left, focal_length_px=256, baseline_meters=0.5)
        
        # Publish using the exact timestamp of the left image to prevent temporal smearing downstream
        self._publish_depth(depth, msg_left.header.stamp)

    def _publish_depth(self, depth_data, original_stamp):
        try:
            if depth_data is None:
                return

            depth_np = np.array(depth_data)
            if depth_np.ndim == 2 or (depth_np.ndim == 3 and depth_np.shape[2] == 1):
                if depth_np.ndim == 3:
                    depth_f = depth_np[:, :, 0].astype(np.float32)
                else:
                    depth_f = depth_np.astype(np.float32)
                    
                img_msg = self.bridge.cv2_to_imgmsg(depth_f, encoding='32FC1')
                # CRITICAL: Pass the timestamp of the original image, not the current time!
                img_msg.header.stamp = original_stamp
                img_msg.header.frame_id = 'depth_link'
                self.depth_image_pub.publish(img_msg)
            else:
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