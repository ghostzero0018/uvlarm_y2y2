#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from kobuki_ros_interfaces.msg import BumperEvent
import time

print("Bumping process initiated...")

class Robot:
    def __init__(self, rosNode):
        # Publisher for robot commands
        self._publisher = rosNode.create_publisher(Twist, '/multi/cmd_nav', 10)
        
        # Subscriber for bumper events
        self._bumper_state = None
        self._bumper_subscriber = rosNode.create_subscription(
            BumperEvent, '/events/bumper', self.bumper_callback, 10
        )
        
        # Timer to handle movement commands
        self._timer = rosNode.create_timer(0.1, self.timer_callback)
    
    def bumper_callback(self, msg):
        # Log received bumper event
        print(f"Bumper Event Received: {msg}")
        
        # Update bumper state based on which bumper was hit
        if msg.bumper == BumperEvent.LEFT:
            self._bumper_state = 0  # Left bumper
        elif msg.bumper == BumperEvent.CENTER:
            self._bumper_state = 1  # Center bumper
        elif msg.bumper == BumperEvent.RIGHT:
            self._bumper_state = 2  # Right bumper

    def timer_callback(self):
        print("Timer callback activated")
        if self._bumper_state is None:
            # No bumper hit, move forward
            self.move(0.3, 0.0)
            print("Moving forward")
        else:
            # Bumper was hit, stop and back up
            self.stop()
            print("Stopping")
            self.move(-0.3, 1.0)  # Move backward and turn
            time.sleep(1)  # Delay to complete movement
            self._bumper_state = None
            print("Resuming movement")
        print("")

    def move(self, x, ang):
        # Create velocity command
        velocity = Twist()
        velocity.linear.x = x
        velocity.angular.z = ang
        self._publisher.publish(velocity)
    
    def stop(self):
        # Publish zero velocity to stop the robot
        self._publisher.publish(Twist())

def start():
    # Initialize ROS2 and the node
    rclpy.init()
    aNode = Node("bumping")
    robot = Robot(aNode)

    # Enter the ROS2 event loop
    rclpy.spin(aNode)

    # Cleanup resources after shutdown
    aNode.destroy_node()
    rclpy.shutdown()

# Script trigger
if __name__ == "__main__":
    start()
