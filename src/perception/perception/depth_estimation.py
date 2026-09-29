from .compute_depth import compute_stereo_depth
from .compute_depth import get_last_depth_timings
from std_msgs.msg import Float32, Header, Float32MultiArray
import rclpy
from rclpy.node import Node
import time

from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import cv2
from cv_bridge import CvBridge
from benchmark_evaluator.midrun_logger import MidrunLogger

# NEW: Import message_filters for stereo synchronization
import message_filters

class obstacleAvoidance(Node):
    def __init__(self):
        super().__init__('object_detector_node')

        self.bridge = CvBridge()
        self.declare_parameter('sync_queue_size', 1)
        self.declare_parameter('sync_slop_s', 0.02)
        self.declare_parameter('results_dir', 'results')
        sync_queue_size = int(self.get_parameter('sync_queue_size').value)
        sync_slop_s = float(self.get_parameter('sync_slop_s').value)
        self.frame_id = 0
        self.telemetry = MidrunLogger(
            self.get_parameter('results_dir').value,
            prefix='perception',
            pipeline_only=True)

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
        # Keep only the newest pair while the model is processing the previous one.
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_cam_left, self.sub_cam_right], 
            queue_size=sync_queue_size,
            slop=sync_slop_s
        )
        self.ts.registerCallback(self.sync_cam_callback)

        self.depth_image_pub = self.create_publisher(Image, 'holocean/depth/image_raw', qos)

        self.get_logger().info("✅ obstacle_detector_node Started (Synchronized)")

    def sync_cam_callback(self, msg_left, msg_right):
        # This callback ONLY fires when a perfectly matched pair of images arrives
        callback_started = time.perf_counter()
        self.frame_id += 1
        frame_id = self.frame_id
        source_stamp_s = (
            msg_left.header.stamp.sec
            + msg_left.header.stamp.nanosec * 1e-9)
        
        # Convert Left Image
        conversion_started = time.perf_counter()
        frame_left = self.bridge.imgmsg_to_cv2(msg_left, desired_encoding='bgr8')
        frame_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2RGB)
        
        # Convert Right (Secondary) Image
        frame_right = self.bridge.imgmsg_to_cv2(msg_right, desired_encoding='bgr8')
        frame_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2RGB)
        conversion_ms = (time.perf_counter() - conversion_started) * 1000.0

        # Compute Depth
        depth = compute_stereo_depth(frame_right, frame_left, focal_length_px=256, baseline_meters=0.5)
        depth_timings = get_last_depth_timings()
        for stage, duration_ms in depth_timings.items():
            if stage.endswith('_ms'):
                self.telemetry.record_stage(
                    stage, duration_ms, frame_id=frame_id,
                    source_stamp_s=source_stamp_s)
        
        # Publish using the exact timestamp of the left image to prevent temporal smearing downstream
        publish_started = time.perf_counter()
        self._publish_depth(depth, msg_left.header.stamp)
        publish_ms = (time.perf_counter() - publish_started) * 1000.0
        total_ms = (time.perf_counter() - callback_started) * 1000.0
        self.telemetry.record_stage(
            'image_conversion', conversion_ms, frame_id=frame_id,
            source_stamp_s=source_stamp_s)
        self.telemetry.record_stage(
            'depth_publish', publish_ms, frame_id=frame_id,
            source_stamp_s=source_stamp_s)
        self.telemetry.record_stage(
            'depth_callback_total', total_ms, frame_id=frame_id,
            source_stamp_s=source_stamp_s)

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