"""
Legacy two-dimensional waypoint planning.

Generate exploration targets using the optional repository exploration memory.
"""

from cv_bridge import CvBridge
from legacy_ports import Distance, ExplorationMemory
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
# from myMessages.msg import objectDetected2D


def boustrophedon_path_planning(area, step=20):
    """
    Generate 2D waypoints using Boustrophedon (lawnmower) pattern.

    Parameters
    ----------
    area : tuple
        Bounds ((xmin, xmax), (ymin, ymax)).
    step : float
        Distance between sweep lines in the input coordinate units.

    Returns
    -------
    list
        Waypoints as (x, y) coordinates.

    """
    (xmin, xmax), (ymin, ymax) = area

    # Create y positions for each sweep line
    y_positions = np.arange(ymin, ymax, step)

    waypoints = []

    for i, y in enumerate(y_positions):
        if i % 2 == 0:
            # Move left → right
            x_line = np.linspace(xmin, xmax, int((xmax - xmin) / step) + 1)
        else:
            # Move right → left
            x_line = np.linspace(xmax, xmin, int((xmax - xmin) / step) + 1)

        for x in x_line:
            waypoints.append((x, y))

    return waypoints
    # return [[15,-110]]


class PathPlanner(Node):
    def __init__(
        self, exploration: ExplorationMemory = None, distance: Distance = None
    ):
        if exploration is None or distance is None:
            from legacy_adapters import RepositoryExplorationMemory, repository_distance

            exploration = (
                exploration
                if exploration is not None
                else RepositoryExplorationMemory()
            )
            distance = distance if distance is not None else repository_distance
        self.exploration = exploration
        self.distance = distance
        super().__init__('path_planner')

        self.bridge = CvBridge()
        # self.areaSearch([(-300, -100), (-200, 190)])
        # self.areaSearch([(-15,-80), (-11,-90)]) #(-15,-80), (-11,-90) /// (-10, 20), (-115, -85)
        # 9,-93,-30
        self.waypoints = [
            (-16, -81, -30, 0)
        ]  # ,(-15,-80),(-11,-80),(-11,-90),(-15,-90)]
        # self.waypoints = [(56,-119,2)]
        # self.waypoints = [(15,-110,10)] # docking station
        # self.waypoints = [(134,-106,-31.5)] # to the orange flare
        # self.waypoints = calculate_route((20, -130),(10,-157),(25, -150),10)
        # Legacy object-detection subscription remains disabled.
        self.stop_sub = self.create_subscription(
            Float32MultiArray, '/motion/report', self.stop_callback, 10
        )
        self.point_sub = self.create_subscription(
            Float32MultiArray, 'object/detected', self.point_callback, 10
        )
        self.pointSegment_sub = self.create_subscription(
            Float32MultiArray, 'object/areaWaypoints', self.pointSegment_callback, 10
        )
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.publisher = self.create_publisher(
            Float32MultiArray, '/trajectory/waypoint', 10
        )
        self.planner_sub = self.create_publisher(Bool, '/planner/report', 10)

        self.get_logger().info('✅ Path planner Started')

    def areaSearch(self, area):
        area = ((area[0][0], area[1][0]), (area[1][1], area[0][1]))
        print('Area to search:', area)
        # try:
        wps = boustrophedon_path_planning(area, step=5)
        # except:
        #     area=((area[0][0],area[1][0]),(area[0][1],area[1][1]))

        print('Generated waypoints:', wps)
        for wp in wps:
            if not self.exploration.is_scanned((wp[0], wp[1])):
                print('Adding waypoint:', wp)
                self.waypoints.append((wp[0], wp[1]))
                self.exploration.add_waypoint((wp[0], wp[1]))
            else:
                print('Already scanned:', wp)

    def point_callback(self, msg):
        point = msg.data
        self.waypoints = [point.tolist()]
        self.get_logger().info(f'Published {point} waypoints.')

    def pointSegment_callback(self, msg):
        data = np.array(msg.data).reshape(-1, 4)
        wps = []
        for wp in data:
            print('Adding segmentation waypoint:', wp)
            if len(wp) == 3:
                wps.append((wp[0], wp[1], wp[2]))
            elif len(wp) == 4:
                wps.append(
                    (
                        wp[0],
                        wp[1],
                        wp[2],
                        wp[3],
                    )
                )
        if wps:
            self.waypoints = wps
            self.get_logger().info(f'Received {len(wps)} segmentation waypoints.')

    # def pointSegment_callback(self, msg):
    #     data=np.array(msg.data).reshape(-1,2)
    #     # self.areaSearch([data[:2],data[2:]])
    #     wps=[]
    #     for wp in data:
    #         if not isItScanned((wp[0],wp[1])):
    #             print("Adding segmentation waypoint:",wp)
    #             wps.append((wp[0],wp[1],4))
    #             addWaypoint((wp[0],wp[1]))
    #         else:
    #             print("Already scanned segmentation waypoint:",wp)
    #     # self.waypoints.extend(wps)
    #     self.waypoints=wps
    #     self.get_logger().info(f"Received {len(wps)} segmentation waypoints.")

    def timer_callback(self):
        if self.waypoints:
            msg = Float32MultiArray()
            flat_waypoints = []
            for wp in self.waypoints:
                if len(wp) == 2:
                    flat_waypoints.extend(
                        [wp[0], wp[1], -30.0]
                    )  # Assuming constant depth of -29.0
                elif len(wp) == 3:
                    flat_waypoints.extend(
                        [wp[0], wp[1], wp[2]]
                    )  # Ensure depth is not belove -30.0
                    # Alternate constant depth (-31.5) remains disabled.
                elif len(wp) == 4:
                    flat_waypoints.extend([wp[0], wp[1], wp[2], wp[3]])
            msg.data = flat_waypoints
            self.publisher.publish(msg)
            self.get_logger().info(f'Published {len(self.waypoints)} waypoints.')
        else:
            self.get_logger().info('No more waypoints to publish.')

    def stop_callback(self, msg):
        try:
            # data=tuple(msg.data[:2].tolist())
            data = msg.data[:3].tolist()
            if self.distance(self.waypoints[0][:2], data[:2]) < 1.0:
                self.waypoints.pop(0)
        except BaseException:
            return 0
        self.get_logger().info('Received stop signal, giving next waypoint.')
        msg = Float32MultiArray()
        # self.planner_sub.publish(Bool(data=True))# debug
        try:
            try:
                self.waypoints.remove(tuple(msg.data[:3].tolist()))
            except BaseException:
                self.waypoints.remove(data)
            self.exploration.add_waypoint(data[:2])
            if not self.waypoints:
                pass
                # self.planner_sub.publish(Bool(data=True))
            else:
                self.waypoints = self.waypoints[-5:]
        except ValueError:
            print('Waypoint not in list')

        self.timer_callback()

    # def cam_callback(self, msg):
    #     # Display if needed
    #     # cv2.imshow("HoloOcean Camera", frame); cv2.waitKey(1)
    #     # Create a dummy object detection message
    #     pos=msg.position

    #     self.target_pub.publish(pos)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
