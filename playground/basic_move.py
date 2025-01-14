#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import time
import math
import sys

class SmartControl(Node):
    def __init__(self):
        super().__init__('basic_move')
        self._logger = self.get_logger()
        self._pub_velocity = self.create_publisher(Twist, '/multi/cmd_nav', 10)
        self.create_subscription(PointCloud, '/scan_points', self.scan_callback, 10)

        # State variables for obstacles
        self.obstacle_left = False
        self.obstacle_right = False
        self.last_action = None

        # Initialize control callback
        self.create_timer(0.1, self.control_callback)

    def scan_callback(self, point_cloud):
        # Determine if there are obstacles within 1 meter on either side
        left_points = [p for p in point_cloud.points if p.y > 0 and p.x < 1]
        right_points = [p for p in point_cloud.points if p.y <= 0 and p.x < 1]

        self.obstacle_left = len(left_points) > 0
        self.obstacle_right = len(right_points) > 0

    def control_callback(self):
        msg = Twist()
        if self.obstacle_left and self.obstacle_right:
            # Both sides have obstacles, turn left or continue previous action
            msg.angular.z = -0.5  # Negative for turning left
            self.last_action = 'turn_left'
        elif self.obstacle_left:
            # Only left side has an obstacle, turn right
            msg.angular.z = 0.5
            self.last_action = 'turn_right'
        elif self.obstacle_right:
            # Only right side has an obstacle, turn left
            msg.angular.z = -0.5
            self.last_action = 'turn_left'
        else:
            # No obstacles, go straight
            msg.linear.x = 0.5
            self.last_action = 'go_straight'

        self._pub_velocity.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SmartControl()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()