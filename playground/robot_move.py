#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud
from kobuki_ros_interfaces.msg import BumperEvent, ButtonEvent, WheelDropEvent
import time
import sys

# ROS Node process:
def main():
    # Initialize ROS and a ROS node
    rclpy.init(args=sys.argv)
    node = Node('basic_move')

    # Initialize the control object:
    control = StraightCtrl()
    control.initialize_ros_node(node)

    # Infinite Loop for processing:
    rclpy.spin(node)

    # Clean termination
    node.destroy_node()
    rclpy.shutdown()

# Control logic class for robot movement:
class StraightCtrl:
    def __init__(self):
        self._logger = None
        self._pub_velocity = None
        self._sub_to_points = None
        self._timer_for_control = None

        self._obs_left = False
        self._obs_right = False
        self._last_move = 0
        self._nbr_points_right = 0
        self._nbr_points_left = 0

        self._bumper_state = None
        self._is_reversing = False
        self._can_move = False

        # Current speeds
        self._current_linear_velocity = 0.0
        self._current_angular_velocity = 0.0

        # Smoothing and limits parameters
        self._velocity_smoothing_factor = 0.08
        self._max_linear_speed = 0.6
        self._max_angular_speed = 1.8
        self._low_linear_speed = 0.2
        self._low_angular_speed = 0.9

        # Blockage detection
        self._direction_changes = 0  # Count of direction changes
        self._last_direction = 0     # Last used direction (-1 = left, 1 = right)
        self._stuck_timer = 0        # Timer to detect being stuck
        self._stuck_threshold = 5    # Threshold to consider robot stuck (number of changes)
        self._stuck_recovery_timer = 0  # Timer for recovery from being stuck

    def initialize_ros_node(self, ros_node):
        # Retrieve logger from the node:
        self._logger = ros_node.get_logger()

        # Initialize subscriptions and publisher:
        self._sub_to_points = ros_node.create_subscription(PointCloud, '/points', self.scan_callback, 10)
        self._bumper_subscriber = ros_node.create_subscription(BumperEvent, '/events/bumper', self.bumper_callback, 10)
        self._button_subscriber = ros_node.create_subscription(ButtonEvent, '/events/button', self.button_callback, 10)
        self._pickup_subscriber = ros_node.create_subscription(WheelDropEvent, '/events/wheel_drop', self.pickup_callback, 10)
        self._pub_velocity = ros_node.create_publisher(Twist, '/multi/cmd_nav', 10)
        
        # Timer for control callbacks:
        self._timer_for_control = ros_node.create_timer(0.05, self.control_callback)

    def scan_callback(self, point_cloud):
        # Reset detection flags and counters:
        self._nbr_points_right = self._nbr_points_left = 0
        self._obs_left = self._obs_right = False
        for point in point_cloud.points:
            if point.y > 0:
                self._obs_left = True 
                self._nbr_points_left += 1
            else:
                self._obs_right = True
                self._nbr_points_right += 1

    def control_callback(self):
        if not self._can_move:
            return

        # Initialization of target speeds
        target_linear_velocity = target_angular_velocity = 0.0

        # Handle bumper event:
        if self._bumper_state is not None and not self._is_reversing:
            self._is_reversing = True
            target_linear_velocity = -0.2  # Back up slowly
            self._logger.info(f"Bumper activated ({self._bumper_state}). Reversing.")

            # Publish the reverse command
            self.publish_velocity(target_linear_velocity, target_angular_velocity)

            # Wait for 2 seconds
            time.sleep(2)

            # Reset bumper state
            self._bumper_state = None
            self._is_reversing = False
            return

        # Blockage detection logic
        if self._obs_right and self._obs_left:
            # Handle obstacles on both sides:
            if self._nbr_points_left > self._nbr_points_right:
                target_angular_velocity = -self._low_angular_speed
                new_direction = -1
            else:
                target_angular_velocity = self._low_angular_speed
                new_direction = 1

            # Check for direction changes:
            if new_direction != self._last_direction:
                self._direction_changes += 1
                self._stuck_timer += 1
                self._last_direction = new_direction
            else:
                self._stuck_timer = 0  # Reset if moving consistently

            # Check if robot is stuck:
            if self._direction_changes > self._stuck_threshold:
                self._logger.info("Stuck detected: Activating recovery.")
                self._stuck_recovery_timer = 50  # Time to perform a recovery action
                self._direction_changes = 0

        elif self._obs_right:
            target_linear_velocity = self._low_linear_speed
            target_angular_velocity = self._low_angular_speed
            self._last_direction = 1
        elif self._obs_left:
            target_linear_velocity = self._low_linear_speed
            target_angular_velocity = -self._low_angular_speed
            self._last_direction = -1
        else:
            target_linear_velocity = self._max_linear_speed
            target_angular_velocity = 0.0
            self._last_direction = 0

        # Handle stuck recovery:
        if self._stuck_recovery_timer > 0:
            target_linear_velocity = -self._low_linear_speed  # Stop linear movement
            target_angular_velocity = self._max_angular_speed * self._last_direction

            # Decrease the recovery timer
            self._stuck_recovery_timer -= 1

        # Apply velocity smoothing:
        self.smooth_and_publish_velocity(target_linear_velocity, target_angular_velocity)

    def bumper_callback(self, msg):
        # Update bumper state based on event:
        self._bumper_state = msg.bumper

    def button_callback(self, msg):
        # Toggle movement ability based on button press:
        self._can_move = (msg.button == ButtonEvent.BUTTON0)

    def pickup_callback(self, msg):
        # Disable movement if wheel is dropped:
        self._can_move = (msg.state != WheelDropEvent.DROPPED)

    def publish_velocity(self, linear, angular):
        # Create Twist message and publish:
        velocity_msg = Twist()
        velocity_msg.linear.x = linear
        velocity_msg.angular.z = angular
        self._pub_velocity.publish(velocity_msg)

    def smooth_and_publish_velocity(self, target_linear, target_angular):
        # Apply smoothing to velocities:
        self._current_linear_velocity += self._velocity_smoothing_factor * (target_linear - self._current_linear_velocity)
        self._current_angular_velocity += self._velocity_smoothing_factor * (target_angular - self._current_angular_velocity)

        # Publish smoothed velocities:
        self.publish_velocity(self._current_linear_velocity, self._current_angular_velocity)

# Script trigger
if __name__ == '__main__':
    main()
