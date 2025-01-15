#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

# Class for detecting a green object in camera images
class GreenObjectDetector(Node): # Create a class that inherits from the Node class
    def __init__(self): # Constructor for the class
        super().__init__('green_object_detector') # Call the constructor of the parent class
        self.bridge = CvBridge() # Create a CvBridge object for converting ROS images to OpenCV images
        self.publisher = self.create_publisher(String, 'green_object_detected', 10) # Create a publisher for the green_object_detected topic
        self.subscription = self.create_subscription(Image, 'sensor_msgs/image', self.image_callback, 10) # Create a subscription to the camera image topic
        self.detection_threshold = 4000  # Pixel count threshold for detection
        self.ghost_detected = False  # State tracking for ghost detection

    def image_callback(self, msg): # Callback function for the camera image subscription
        # Convert ROS image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8") # Convert the ROS image message to an OpenCV image

        # Convert BGR to HSV for better color segmentation
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # Convert the BGR image to an HSV image
        lower_green = np.array([35, 100, 50]) # Define the lower HSV values for green color
        upper_green = np.array([85, 255, 255]) # Define the upper HSV values for green color

        # Create a mask for green color
        mask = cv2.inRange(hsv_image, lower_green, upper_green) # Create a mask for the green color in the HSV image
        green_pixel_count = cv2.countNonZero(mask) # Count the number of green pixels in the mask

        # Check for significant green color presence
        if green_pixel_count > self.detection_threshold and not self.ghost_detected: # If the number of green pixels is greater than the threshold and a ghost has not been detected
            self.publisher.publish(String(data="Green ghost detected!")) # Publish a message to the green_object_detected topic
            self.ghost_detected = True # Set the ghost_detected flag to True
        elif green_pixel_count < self.detection_threshold: # If the number of green pixels is less than the threshold
            self.ghost_detected = False # Set the ghost_detected flag to False

        # Display images for debugging (can be commented out in production)
        cv2.imshow('Original Image', frame) # Display the original image    
        cv2.imshow('Green Mask', mask) # Display the green mask
        cv2.waitKey(1) # Wait for a short time to allow the images to be displayed

def main(args=None): # Main function for the node
    rclpy.init(args=args) # Initialize the ROS client library
    node = GreenObjectDetector() # Create an instance of the GreenObjectDetector class
    rclpy.spin(node) # Run the node
    node.destroy_node() # Destroy the node
    rclpy.shutdown() # Shutdown the ROS client library

# Script trigger
if __name__ == '__main__':
    main()
