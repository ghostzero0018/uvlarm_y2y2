#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point32
from visualization_msgs.msg import Marker
from std_msgs.msg import String, Float32, Header
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
import math
import signal


class Move(Node):
    def __init__(self):
        super().__init__('ultimate_move')
        self.initialize_ros_components()

    def initialize_ros_components(self):
        # Publishers
        self.marker_publisher = self.create_publisher(Marker, 'marker_test', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/multi/cmd_nav', 10)
        self.cloud_publisher = self.create_publisher(pc2.PointCloud2, 'laser_link', 10)

        # Subscribers
        self.create_subscription(Float32, 'x', self.x_callback, 10)
        self.create_subscription(Float32, 'z', self.z_callback, 10)
        self.create_subscription(Float32, 'bottle_position', self.bottle_position_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(String, 'detection', self.detection_callback, 10)
        self.create_subscription(Float32, 'distance_bottle', self.distance_bottle_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        # State variables
        self.isOk = True
        self.detection = False
        self.x_label = None
        self.y_label = None
        self.x_add = None
        self.y_add = None

    def x_callback(self, msg):
        self.x_label = msg.data

    def z_callback(self, msg):
        self.y_label = msg.data

    def odom_callback(self, msg):
        self.x_add = msg.pose.pose.position.x
        self.y_add = msg.pose.pose.position.y
        self.orientation_euler = msg.pose.pose.orientation.z

    def bottle_position_callback(self, msg):
        pass

    def scan_callback(self, scanMsg):
        angle = scanMsg.angle_min
        obstacles = []
        cmd_debug_points_left = []
        cmd_debug_points_right = []

        for aDistance in scanMsg.ranges:
            if 0.1 < aDistance < 5.0:
                x = math.cos(angle) * aDistance
                y = math.sin(angle) * aDistance
                obstacles.append([x, y, 0.0])
                if 0.01 < y < 0.2 and 0.1 < x < 0.5:
                    cmd_debug_points_left.append([x, y])
                if -0.2 < y < -0.01 and 0.1 < x < 0.5:
                    cmd_debug_points_right.append([x, y])
            angle += scanMsg.angle_increment

        self.handle_obstacles(cmd_debug_points_left, cmd_debug_points_right, obstacles)

    def handle_obstacles(self, left_points, right_points, obstacles):
        velo = Twist()
        if len(right_points) > 0:
            self.get_logger().info("Avoiding left")
            velo.angular.z = 1.5
            velo.linear.x = 0.05
        elif len(left_points) > 0:
            self.get_logger().info("Avoiding right")
            velo.angular.z = -1.5
            velo.linear.x = 0.05
        else:
            velo.linear.x = 0.15
            velo.angular.z = 0.0

        cloud_points = pc2.create_cloud_xyz32(Header(frame_id='laser_link'), obstacles)
        self.publish_move(velo, cloud_points)

    def detection_callback(self, detectionMsg):
        self.detection = detectionMsg.data == "bottle founded"

    def distance_bottle_callback(self, msg):
        self.distance_bottle = msg.data

    def publish_move(self, velo, cloud_points):
        self.velocity_publisher.publish(velo)
        self.cloud_publisher.publish(cloud_points)

    def signal_interruption(self, signum, frame):
        self.get_logger().info("Ctrl-C pressed, shutting down.")
        self.isOk = False


def main(args=None):
    rclpy.init(args=args)
    ros_node = Move()
    signal.signal(signal.SIGINT, ros_node.signal_interruption)

    while ros_node.isOk:
        rclpy.spin_once(ros_node, timeout_sec=0.1)

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
