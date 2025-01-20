#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class GreenObjectDetector(Node):
    def __init__(self):
        super().__init__('green_object_detector')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(String, 'object_detected', 10)
        self.subscription = self.create_subscription(Image, 'sensor_msgs/image', self.image_callback, 10)
        self.depth_subscription = self.create_subscription(Image, 'sensor_msgs/depth', self.depth_callback, 10)
        self.depth_frame = None
        self.min_contour_area = 100
        self.min_ghost_area = 2500

    def depth_callback(self, msg):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error('Could not convert depth image: %s' % e)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error('Could not convert image: %s' % e)
            return

        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 100, 50])
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv_image, lower_green, upper_green)
        grey_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found_objects = False
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= self.min_contour_area:
                ((x, y), radius) = cv2.minEnclosingCircle(contour)
                x, y = int(x), int(y)
                if area > self.min_ghost_area:
                    depth = self.get_object_distance(x, y)
                    distance_msg = f"{depth:.2f} meters" if depth is not None else "unknown"
                    self.get_logger().info(f"Green Ghost detected at ({x}, {y}) with {area} pixels and depth: {distance_msg}.")
                    cv2.drawContours(frame, [contour], -1, (0, 0, 255), 2)
                    cv2.putText(frame, f"Depth: {distance_msg}", (x - 10, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(grey_image, "Marker", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    found_objects = True

        if not found_objects:
            self.get_logger().info("No significant green objects detected.")

        cv2.imshow('Detected Objects', frame)
        cv2.imshow('Grey Image', grey_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Allows window to close on 'q' key press
            cv2.destroyAllWindows()

    def get_object_distance(self, x, y):
        # Assuming x and y are coordinates in the depth_frame array
        # Ensure that we have a valid depth frame and the coordinates do not go out of bounds
        if self.depth_frame is not None and 0 <= y < self.depth_frame.shape[0] and 0 <= x < self.depth_frame.shape[1]:
            # Get a small region around the point to handle noisy single-pixel depth anomalies
            depth_region = self.depth_frame[max(0, y-5):min(y+5, self.depth_frame.shape[0]), max(0, x-5):min(x+5, self.depth_frame.shape[1])]
            # Check if all depths in a small box around (x, y) are greater than 0
            if np.all(depth_region > 0):  # Using np.all to ensure all values meet the condition
                return np.mean(depth_region)
        else:
            print("Invalid or noisy depth data near the point.")
        return None


def main(args=None):
    rclpy.init(args=args)
    node = GreenObjectDetector()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()