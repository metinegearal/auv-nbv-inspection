"""
1. Exploration Rate (Information vs. Distance): The number of newly discovered voxels divided by the distance traveled. (High is good; it means the AUV isn't wandering pointlessly).
2. Total Path Length (Meters): The total distance the AUV drove to achieve a certain percentage of map completion.
3. Planning Latency (Milliseconds): How long your algorithm takes to generate viewpoints and run A*. (Under 100ms is ideal for real-time robotic control).
4. Surface Coverage (%): The number of occupied voxels currently mapped divided by the estimated total voxels of the target.
"""

from datetime import datetime
import sys
sys.path.append('/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem')

from Tools.get_point_cloud import RealTimeStereoMapper
#!/usr/bin/env python3
from std_msgs.msg import Float32, Header, Float32MultiArray
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, Imu, MagneticField, FluidPressure
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
import numpy as np
import pandas as pd
import cv2
from cv_bridge import CvBridge
# from myMessages.msg import objectDetected2D

from collections import deque

class objectDetection(Node):
    def __init__(self):
        super().__init__('object_detector_node')

        self.current_position = np.array([0.0, 0.0, 0.0])
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.left_image = None
        self.depth_frame = None
        self.started_mapping = False
        self.finished_mapping = False
        
        self.mapper = RealTimeStereoMapper(voxel_resolution=0.1)
        self.bridge = CvBridge()
        
        self.file = open("Notes/trajectory.txt","w")
        self.file_heading=open("Notes/trajectory_heading.txt","w")
        self.eval_parameters = open("Notes/evaluation_parameters.txt","w")

        # self.sub_cam = self.create_subscription(Image, 'camera/image_processed', self.cam_callback, 10)
        self.sub_cam = self.create_subscription(Image, 'holocean/depth/image_raw', self.depth_cam_callback, 10)
        self.sub_cam2 = self.create_subscription(Image, 'holocean/cameraDown/image_raw', self.cam_callback, 10)

        self.position_sub = self.create_subscription(Odometry, 'holocean/odom', self.position_callback, 10)
        self.sub_mag = self.create_subscription(MagneticField, 'holocean/mag', self.magnetometer_callback, 10)

        self.pointSegment_pub = self.create_publisher(Float32MultiArray, 'object/areaWaypoints', 10)
        self.finished_mapping_pub = self.create_publisher(Bool, '/movement/finished_execution', 10)


        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i=0

        self.get_logger().info("✅ object_detector_node Started")
    
    def position_callback(self, msg):
        pos = msg.pose.pose.position
        self.current_position = np.array([pos.x, pos.y, pos.z])

        # Extract roll and pitch from the odometry orientation quaternion.
        q = msg.pose.pose.orientation
        sin_roll = 2.0 * (q.w * q.x + q.y * q.z)
        cos_roll = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.roll = np.arctan2(sin_roll, cos_roll)

        sin_pitch = 2.0 * (q.w * q.y - q.z * q.x)
        self.pitch = np.arcsin(np.clip(sin_pitch, -1.0, 1.0))
        # print("Current Position:", pos.x, pos.y, pos.z)

    def magnetometer_callback(self, msg):
        self.magnet = [msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z]
        ya=np.arctan2(self.magnet[0], self.magnet[1])-(np.pi/2) 
        if ya < -1*np.pi:
            ya+=np.pi*2  
        self.yaw = ya

    def cam_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.left_image=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)

        self.get_logger().info(f"Camera frame received.")

    def depth_cam_callback(self, msg):
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Normalize to 0-255 so OpenCV can display it
        depth_norm = cv2.normalize(self.depth_frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        
        # Optional: Make it colorful (much easier to see than grayscale)
        depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

        cv2.imshow("Depth Frame", depth_color)
        cv2.waitKey(1)

    def timer_callback(self):
        if self.finished_mapping:
            self.get_logger().info(f"Mapping finished, no more waypoints to explore.")
            return
        if self.left_image is None or self.depth_frame is None:
            self.get_logger().info(f"Cant Process frame for mapping.")
            return

        self.file.write(f"{self.current_position[0]},{self.current_position[1]},{self.current_position[2]}\n")
        self.file_heading.write(f"{self.yaw},{self.pitch},{self.roll}\n")

        now=datetime.now()
        self.mapper.process_frame(location=self.current_position, yaw_radians=self.yaw, pitch_radians=self.pitch, roll_radians=self.roll, frame1=self.left_image, depth_map=self.depth_frame)
        process_time=(datetime.now()-now).total_seconds()*1000

        waypoints=[]
        if self.i%20==0:
            self.mapper.save_map("final_map.bt")

            now=datetime.now()
            candidates, smooth_traj=self.mapper.generate_candidate_viewpoints(self.current_position)
            waypoints_time=(datetime.now()-now).total_seconds()*1000

            waypoints = [c[0] for c in candidates] #c[1] is the point that it should look
            print("waypoints----", waypoints)
            if len(waypoints)>0:
                # # Publish the waypoints
                # waypoint_msg = Float32MultiArray()
                # # FLATTEN THE LIST OF ARRAYS HERE:
                # waypoint_msg.data = np.array(waypoints).flatten().tolist()
                # self.pointSegment_pub.publish(waypoint_msg)

                self.started_mapping = True

                # 3. Publish
                waypoint_msg = Float32MultiArray()
                waypoint_msg.data = smooth_traj.flatten().tolist()
                self.pointSegment_pub.publish(waypoint_msg)
            else:
                if self.started_mapping:
                    self.finished_mapping = True
                    self.get_logger().info("Mapping finished, no more waypoints to explore.")
                    # Publish the finished mapping signal
                    finished_msg = Bool()
                    finished_msg.data = True
                    self.finished_mapping_pub.publish(finished_msg)


        self.i+=1
        if len(waypoints)>0:
            self.eval_parameters.write(f"Process Time (ms): {process_time}, Waypoints Generation Time (ms): {waypoints_time}, Total Waypoints: {len(waypoints)}\n")
        else:
            self.eval_parameters.write(f"Process Time (ms): {process_time} \n")



def main(args=None):
    rclpy.init(args=args)
    node = objectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

