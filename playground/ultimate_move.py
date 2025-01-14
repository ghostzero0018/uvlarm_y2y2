#!/usr/bin/python3
from visualization_msgs.msg import Marker # Import the Marker message type
import rclpy # Import the ROS Client Library for Python
from rclpy.node import Node # Import the Node class from rclpy
from sensor_msgs.msg import LaserScan # Import the LaserScan message type
from geometry_msgs.msg import Point32 # Import the Point32 message type
from std_msgs.msg import String # Import the String message type
from std_msgs.msg import Float32 # Import the Float32 message type
from nav_msgs.msg import Odometry # Import the Odometry message type
import sensor_msgs_py.point_cloud2 as pc2 # Import the PointCloud2 message type
from std_msgs.msg import Header # Import the Header message type
import time # Import the time module
import math # Import the math module
import random # Import the random module
import signal # Import the signal module
# Message to publish:
from geometry_msgs.msg import Twist # Import the Twist message type
import transformations as tf # Import the transformations module
import rospy # Import the rospy module

class Move(Node): # Define the Move class
    def __init__(self, fps=60): # Define the __init__ method
        super().__init__('move') # Call the __init__ method of the parent class
        self.marker_publisher = self.create_publisher(Marker, 'marker_test', 10) # Create a publisher for the Marker message type
        self.create_subscription(Float32, 'x', self.x_callback, 10) # Create a subscription to the 'x' topic
        self.create_subscription(Float32, 'z', self.z_callback, 10) # Create a subscription to the 'z' topic
        self.create_subscription(Float32, 'bottle_position', self.bottle_position_callback, 10) # Create a subscription to the 'bottle_position' topic 
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10) # Create a subscription to the 'odom' topic
        self.create_subscription(String, 'detection', self.detection_callback, 10) # Create a subscription to the 'detection' topic
        self.create_subscription(Float32, 'distance_bottle', self.distance_bottle_callback, 10) # Create a subscription to the 'distance_bottle' topic
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10) # Create a subscription to the 'scan' topic
        
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10) # Create a publisher for the Twist message type
        self.cloud_publisher = self.create_publisher(pc2.PointCloud2, 'laser_link', 10) # Create a publisher for the PointCloud2 message type
        self.isOk = True # Set the isOk attribute to True
        self.detection = False # Set the detection attribute to False
        
        # Initialize ROS node
        rospy.init_node('odom_to_map_converter')
        
        # Create a TF listener
        listener = tf.TransformListener()
        
        # Wait for TF transformation tree to be fully available
        rospy.sleep(1.0)

        self.target_frame = 'map' # Define the target and source frames
        self.source_frame = 'odom' # Define the target and source frames

    def transform(self): # Define the transform method
        target_frame = self.target_frame # Define the target and source frames
        source_frame = self.source_frame # Define the target and source frames
        try:
            now = rospy.Time.now()
            listener.waitForTransform(self.target_frame, self.source_frame, now, rospy.Duration(1.0)) # Wait for the transform to be available
            (trans, rot) = listener.lookupTransform(self.target_frame, self.source_frame, now) # Look up the transform
            print(f'Translation: {trans}') # Print the translation and rotation
            print(f'Rotation: {rot}') # Print the translation and rotation
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): # Handle exceptions
            print('Failed to obtain transform.') # Print an error message
            rospy.signal_shutdown('Coordinate transformation completed.') # Shut down the node

    def x_callback(self, msg): # Define the x_callback method
        self.x_label = msg.data # Set the x_label attribute to the data in the message

    def z_callback(self, msg): # Define the z_callback method
        self.y_label = msg.data # Set the y_label attribute to the data in the message

    def odom_callback(self, msg): # Define the odom_callback method
        self.x_add = msg.pose.pose.position.x # Set the x_add attribute to the x position in the message
        self.y_add = msg.pose.pose.position.y # Set the y_add attribute to the y position in the message
        self.orientation_euler = msg.pose.pose.position.z # Set the orientation_euler attribute to the z position in the message

    def bottle_position_callback(self, msg): # Define the bottle_position_callback method
        pass # Do nothing

    def marker_callback(self,marker): # Define the marker_callback method
        print('obstacle_founded') # Print a message
        pass # Do nothing
        
    def scan_callback(self, scanMsg): # Define the scan_callback method
        angle = scanMsg.angle_min # Set the angle to the minimum angle in the scan message
        obstacles = [] # Create an empty list to store obstacles
        cmd_debug_points_left = [] # Create an empty list to store debug points
        cmd_debug_points_right = [] # Create an empty list to store debug points
        
        self.transform() # Call the transform method

        if self.detection and hasattr(self, 'x_add'): # Check if detection is True and x_add attribute exists
            print('Hello_World') # Print a message
            marker = Marker() # Create a new Marker message
            marker.header.frame_id = 'camera_link' # Set the frame_id of the marker
            marker.type = Marker.SPHERE # Set the type of the marker
            x_real_label = self.x_label # Set the x_real_label to the x_label attribute
            y_real_label = self.y_label # Set the y_real_label to the y_label attribute
            if x_real_label and y_real_label: # Check if x_real_label and y_real_label are not None
                marker.pose.position.x = x_real_label # Set the x position of the marker
                marker.pose.position.y = y_real_label # Set the y position of the marker
                marker.pose.position.z = 0.0 # Set the z position of the marker
                marker.scale.x = 0.2 # Set the x scale of the marker
                marker.scale.y = 0.2 # Set the y scale of the marker
                marker.scale.z = 0.2 # Set the z scale of the marker
                marker.color.r = 1.0 # Set the red color of the marker
                marker.color.g = 0.0 # Set the green color of the marker
                marker.color.b = 0.0 # Set the blue color of the marker
                marker.color.a = 1.0 # Set the alpha value of the marker
                self.marker_publisher.publish(marker) # Publish the marker message

        for aDistance in scanMsg.ranges: # Iterate over the distances in the scan message
            if 0.1 < aDistance < 5.0: # Check if the distance is within a valid range
                aPoint = [ # Create a new point
                    math.cos(angle) * aDistance, # Calculate the x position
                    math.sin(angle) * aDistance, # Calculate the y position
                    0.0 # Set the z position to 0.0
                ] # End of the list
                obstacles.append(aPoint) # Append the point to the obstacles list
                if 0.01 < aPoint[1] < 0.2 and 0.1 < aPoint[0] < 0.5: 
                    cmd_debug_points_left.append(aPoint) # Append the point to the debug points list
                if -0.2 < aPoint[1] < -0.01 and 0.1 < aPoint[0] < 0.5: 
                    cmd_debug_points_right.append(aPoint) # Append the point to the debug points list
            angle += scanMsg.angle_increment # Increment the angle
        velo = Twist() # Create a new Twist message
            
        if len(cmd_debug_points_right) > 0: # Check if the length of the debug points list is greater than 0
            print("avoid Left") # Print a message
            velo.angular.z = 1.5 # Set the angular velocity
            velo.linear.x = 0.05 # Set the linear velocity
        elif len(cmd_debug_points_left) > 0: # Check if the length of the debug points list is greater than 0
            print("avoid right") # Print a message
            velo.angular.z = -1.5 # Set the angular velocity
            velo.linear.x = 0.05 # Set the linear velocity
        else:
            velo.linear.x = 0.15 # Set the linear velocity
            velo.angular.z = 0.0 # Set the angular velocity 

        cloudPoints = pc2.create_cloud_xyz32(Header(frame_id='laser_link'), obstacles) # Create a
        self.publish_move(velo, cloudPoints) # Call the publish_move method
        
    def detection_callback(self, detectionMsg): # Define the detection_callback method
        if detectionMsg.data == "bottle unfounded": # Check if the data in the message is "bottle unfounded"
            self.detection = False # Set the detection attribute to False
        elif detectionMsg.data == "bottle founded": # Check if the data in the message is "bottle founded"
            self.detection = True # Set the detection attribute to True
    
    def distance_bottle_callback(self, distance_bottleMsg): # Define the distance_bottle_callback method
        self.distance_bottle = distance_bottleMsg.data # Set the distance_bottle attribute to the data in the message

    def publish_move(self, velo, cloudPoints): # Define the publish_move method
        self.velocity_publisher.publish(velo) # Publish the velocity message
        self.cloud_publisher.publish(cloudPoints) # Publish the cloudPoints message

    def signalInteruption(signum, frame): # Define the signalInteruption method
        print("\nCtrl-c pressed") # Print a message
        self.isOk = False # Set the isOk attribute to False

# Main function
if __name__ == '__main__': # Check if the script is being run directly
    print("moving_wheels") # Print a message
    rclpy.init() # Initialize the ROS client library
    rosNode = Move() # Create a new Move object
    signal.signal(signal.SIGINT, rosNode.signalInteruption) # Register the signal handler
    while rosNode.isOk: # Check if the isOk attribute is True
        rclpy.spin_once(rosNode, timeout_sec=0.1) # Spin the node
    rosNode.destroy_node() # Destroy the ROS node
    print("\nEnding_move") # Print a message
    rclpy.shutdown() # Shut down the ROS client library
