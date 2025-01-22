#!/usr/bin/python3

import rclpy # ROS2 Python library
from rclpy.node import Node # ROS2 Python class for creating a node
from sensor_msgs.msg import LaserScan # ROS2 Python class for handling laser scan messages
from geometry_msgs.msg import Twist, Point32 # ROS2 Python class for handling velocity commands
from visualization_msgs.msg import Marker # ROS2 Python class for handling markers
from std_msgs.msg import String, Float32, Header # ROS2 Python class for handling standard messages
from nav_msgs.msg import Odometry # ROS2 Python class for handling odometry messages
import sensor_msgs_py.point_cloud2 as pc2 # ROS2 Python class for handling point cloud messages
import math # Python library for handling math operations
import signal # Python library for handling signals


class Move(Node): # Python class for the Move node
    def __init__(self): # Python method for initializing the Move node
        super().__init__('ultimate_move') # Initialize the Move node
        self.initialize_ros_components() # Initialize the ROS components

    def initialize_ros_components(self): # Python method for initializing the ROS components
        # Publishers
        self.marker_publisher = self.create_publisher(Marker, 'marker_test', 10) # Create a publisher for the marker messages
        self.velocity_publisher = self.create_publisher(Twist, '/multi/cmd_nav', 10) # Create a publisher for the velocity commands
        self.cloud_publisher = self.create_publisher(pc2.PointCloud2, 'laser_link', 10) # Create a publisher for the point cloud messages

        # Subscribers
        self.create_subscription(Float32, 'x', self.x_callback, 10) # Create a subscriber for the x coordinate
        self.create_subscription(Float32, 'z', self.z_callback, 10) # Create a subscriber for the z coordinate
        self.create_subscription(Float32, 'bottle_position', self.bottle_position_callback, 10) # Create a subscriber for the bottle position
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10) # Create a subscriber for the odometry messages
        self.create_subscription(String, 'detection', self.detection_callback, 10) # Create a subscriber for the detection messages
        self.create_subscription(Float32, 'distance_bottle', self.distance_bottle_callback, 10) # Create a subscriber for the distance to the bottle
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10) # Create a subscriber for the laser scan messages

        # State variables
        self.isOk = True # Variable for checking if the node is running
        self.detection = False # Variable for checking if the bottle is detected
        self.x_label = None # Variable for storing the x label
        self.y_label = None # Variable for storing the y label
        self.x_add = None # Variable for storing the x addition
        self.y_add = None # Variable for storing the y addition

    def x_callback(self, msg): # Python method for handling the x coordinate
        self.x_label = msg.data # Store the x label

    def z_callback(self, msg): # Python method for handling the z coordinate
        self.y_label = msg.data # Store the z label

    def odom_callback(self, msg): # Python method for handling the odometry messages
        self.x_add = msg.pose.pose.position.x # Store the x addition
        self.y_add = msg.pose.pose.position.y # Store the y addition
        self.orientation_euler = msg.pose.pose.orientation.z # Store the orientation

    def bottle_position_callback(self, msg): # Python method for handling the bottle position
        pass # Do nothing
   
    def scan_callback(self, scanMsg): # Python method for handling the laser scan messages
        angle = scanMsg.angle_min # Initialize the angle
        obstacles = [] # Initialize the obstacles
        cmd_debug_points_left = [] # Initialize the debug points for the left side
        cmd_debug_points_right = [] # Initialize the debug points for the right side

        for aDistance in scanMsg.ranges: # Iterate through the distances
            if 0.1 < aDistance < 5.0: # Check if the distance is within the range
                x = math.cos(angle) * aDistance # Calculate the x coordinate
                y = math.sin(angle) * aDistance # Calculate the y coordinate
                obstacles.append([x, y, 0.0]) # Append the obstacle
                if 0.01 < y < 0.2 and 0.1 < x < 0.5: # Check if the obstacle is on the left side
                    cmd_debug_points_left.append([x, y]) # Append the debug point for the left side
                if -0.2 < y < -0.01 and 0.1 < x < 0.5: # Check if the obstacle is on the right side
                    cmd_debug_points_right.append([x, y]) # Append the debug point for the right side
            angle += scanMsg.angle_increment # Update the angle
        self.handle_obstacles(cmd_debug_points_left, cmd_debug_points_right, obstacles) # Handle the obstacles

    def handle_obstacles(self, left_points, right_points, obstacles): # Python method for handling the obstacles
        velo = Twist() # Initialize the velocity command
        if len(right_points) > 0: # Check if there are obstacles on the right side
            self.get_logger().info("Avoiding left") # Log the message
            velo.angular.z = 1.5 # Set the angular velocity
            velo.linear.x = 0.05 # Set the linear velocity
        elif len(left_points) > 0: # Check if there are obstacles on the left side
            self.get_logger().info("Avoiding right") # Log the message
            velo.angular.z = -1.5 # Set the angular velocity
            velo.linear.x = 0.05 # Set the linear velocity
        else: # Otherwise
            velo.linear.x = 0.15 # Set the linear velocity
            velo.angular.z = 0.0 # Set the angular velocity

        cloud_points = pc2.create_cloud_xyz32(Header(frame_id='laser_link'), obstacles) # Create the point cloud
        self.publish_move(velo, cloud_points) # Publish the velocity command and the point cloud

    def detection_callback(self, detectionMsg): # Python method for handling the detection messages
        self.detection = detectionMsg.data == "bottle founded" # Check if the bottle is detected

    def distance_bottle_callback(self, msg): # Python method for handling the distance to the bottle
        self.distance_bottle = msg.data # Store the distance to the bottle

    def publish_move(self, velo, cloud_points): # Python method for publishing the velocity command and the point cloud
        self.velocity_publisher.publish(velo) # Publish the velocity command
        self.cloud_publisher.publish(cloud_points) # Publish the point cloud

    def signal_interruption(self, signum, frame): # Python method for handling the signal interruption
        self.get_logger().info("Ctrl-C pressed, shutting down.") # Log the message
        self.isOk = False # Set the node to not running


def main(args=None): # Python method for the main function
    rclpy.init(args=args) # Initialize the ROS components
    ros_node = Move() # Create an instance of the Move node
    signal.signal(signal.SIGINT, ros_node.signal_interruption) # Handle the signal interruption
 
    while ros_node.isOk: # Check if the node is running
        rclpy.spin_once(ros_node, timeout_sec=0.1) # Spin the node

    ros_node.destroy_node() # Destroy the node
    rclpy.shutdown() # Shutdown the ROS components


if __name__ == '__main__': # Check if the script is being run directly
    main() # Run the main function
