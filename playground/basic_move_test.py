#!/usr/bin/python3

import rclpy, math
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
        self._obs_threshold = 0.5  # meters, dynamic adjustment based on context
        self._robot_speed = 0.3    # Initial speed, can be adjusted
        self.min_distance = float('inf')  # Initialize min_distance

    def initialize_ros_node(self, ros_node):
        self._logger = ros_node.get_logger()
        self._sub_to_scan = ros_node.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self._pub_velocity = ros_node.create_publisher(Twist, '/multi/cmd_nav', 10)
        self._timer_for_control = ros_node.create_timer(0.1, self.control_callback)

    def scan_callback(self, scanMsg):
        # Transform Scan :
        self._obstacles= []
        angle= scanMsg.angle_min
        self.min_distance= 6.0
        for aDistance in scanMsg.ranges :
            if 0.1 < aDistance and aDistance < 5.0 :
                aPoint= [
                    math.cos(angle) * aDistance,
                    math.sin(angle) * aDistance
                ]
                self._obstacles.append(aPoint)
                if aDistance < self.min_distance :
                    self.min_distance= aDistance
                    self.min_point= aPoint
            angle+= scanMsg.angle_increment

        # Process laser scan data to find obstacles
        if self.min_distance < self._obs_threshold:
            self._can_move = False
        else:
            self._can_move = True

    def control_callback(self):
        if self._can_move :
            self.publish_velocity( self._robot_speed, 0.0 )  # Move forward
        else:
            self.publish_velocity( 0.0, self.adjust_turning() )  # Dynamic turning

    def publish_velocity(self, linear, angular):
        velocity_msg = Twist()
        velocity_msg.linear.x = linear
        velocity_msg.angular.z = angular
        self._pub_velocity.publish(velocity_msg)

    def adjust_velocity_based_on_obstacle(self):
        # Adjust speed based on proximity to the obstacle
        if self.min_distance < 0.3:
            self._robot_speed = 0.1  # Slow down significantly
        elif self.min_distance < 0.5:
            self._robot_speed = 0.2  # Moderate speed

    def adjust_turning( self ):
        print( f"> adjust_velocity {self.min_distance} - {self.min_point}" )
        # Adjust turning angle based on obstacle proximity
        if self.min_distance < 0.3:
            return 0.5  # Sharper turn
        else:
            return 0.3  # Less sharp turn

# Script trigger
if __name__ == '__main__':
    main()