#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

# Class for detecting a green object in camera images
class GreenObjectDetector(Node):
    def __init__(self):
        super().__init__('green_object_detector')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(String, 'green_object_detected', 10)
        self.subscription = self.create_subscription(Image, 'sensor_msgs/image', self.image_callback, 10)
        self.detection_threshold = 4000
        self.ghost_detected = False

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 100, 50])
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv_image, lower_green, upper_green)
        green_pixel_count = cv2.countNonZero(mask)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours and max(cv2.contourArea(contour) for contour in contours) > self.detection_threshold:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)  # Draw red rectangle around the detected object

            self.get_logger().info(f"Green ghost detected with {green_pixel_count} pixels")
            self.publisher.publish(String(data="Green_ghost_detected!"))
            self.ghost_detected = True
        else:
            self.ghost_detected = False

        cv2.imshow('Original Image', frame)
        cv2.imshow('Green Mask', mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = GreenObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()