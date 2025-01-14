#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys

def main():
    rclpy.init(args=sys.argv)
    node = Node('basic_move_test')
    control = RobotControl()
    control.initialize_ros_node(node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

class RobotControl:
    def __init__(self):
        self._logger = None
        self._pub_velocity = None
        self._sub_to_scan = None
        self._timer_for_control = None
        self._can_move = True
        self._obs_threshold = 0.5  # meters

    def initialize_ros_node(self, ros_node):
        self._logger = ros_node.get_logger()
        self._sub_to_scan = ros_node.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self._pub_velocity = ros_node.create_publisher(Twist, '/multi/cmd_nav', 10)
        self._timer_for_control = ros_node.create_timer(0.1, self.control_callback)

    def scan_callback(self, msg):
        # Process laser scan data to find obstacles
        ranges = msg.ranges
        min_distance = min(ranges) if ranges else float('inf')
        if min_distance < self._obs_threshold:
            self._can_move = False
        else:
            self._can_move = True

    def control_callback(self):
        if self._can_move:
            self.publish_velocity(0.3, 0.0)  # Move forward
        else:
            self.publish_velocity(0.0, 0.3)  # Turn in place

    def publish_velocity(self, linear, angular):
        velocity_msg = Twist()
        velocity_msg.linear.x = linear
        velocity_msg.angular.z = angular
        self._pub_velocity.publish(velocity_msg)

# Script trigger
if __name__ == '__main__':
    main()