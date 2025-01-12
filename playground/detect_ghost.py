#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from kobuki_ros_interfaces.msg import Sound

# Main class for the ROS node
class GreenObjectDetector:
    def __init__(self):
        self._logger = None
        self._publisher = None
        self._subscriber = None
        self._bridge = CvBridge()
        self._ghost_detected = False
        self._detection_threshold = 4000

    def initializeROSnode(self, ros_node):
        # Initialize the logger
        self._logger = ros_node.get_logger()

        # Initialize the publisher for detected messages
        self._publisher = ros_node.create_publisher(
            String, 'green_object_detected', 10
        )
        
        # Initialize the publisher for sound commands
        self._sound_publisher = ros_node.create_publisher(
            Sound, 'commands/sound', 10
        )

        # Initialize the subscriber to receive images
        self._subscriber = ros_node.create_subscription(
            Image, 'sensor_msgs/image', self.image_callback, 10
        )

    def image_callback(self, msg):
        # Convert ROS message to OpenCV image
        frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert image from RGB to HSV color space
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the color range for green detection in HSV
        lower_green = np.array([35, 100, 50])  # Adjust as needed
        upper_green = np.array([85, 255, 255]) # Adjust as needed

        # Create a mask for the green color range
        mask = cv2.inRange(hsv_image, lower_green, upper_green)

        # Count the number of green pixels in the mask
        green_pixel_count = cv2.countNonZero(mask)

        # Uncomment the code below to log the number of detected green pixels
        # self._logger.info(f"{green_pixel_count} green pixels detected.")

        # Check if the green pixel count exceeds a certain threshold
        if green_pixel_count > self._detection_threshold and not self._ghost_detected:
            self._publisher.publish(String(data="Ghost detected!"))
            sound = Sound()
            sound.value = 3
            self._sound_publisher.publish(sound)
            
            self._ghost_detected = True

        if green_pixel_count < self._detection_threshold:
            self._ghost_detected = False

        # Display the original and masked images (for debugging)
        cv2.imshow('Original Image', frame)
        cv2.imshow('Green Mask', mask)
        cv2.waitKey(1)  # Wait for 1 ms to allow for image display

# Main function for the ROS2 node
def main():
    # Initialize ROS2 and a ROS2 node
    rclpy.init()
    node = Node('green_object_detector')

    # Create an instance of the control class and initialize the node
    green_object_detector = GreenObjectDetector()
    green_object_detector.initializeROSnode(node)

    # Run the node continuously
    rclpy.spin(node)

    # Clean up properly
    node.destroy_node()
    rclpy.shutdown()

# Script trigger
if __name__ == '__main__':
    main()
