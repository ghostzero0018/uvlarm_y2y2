#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import pyrealsense2 as rs

class GreenObjectDetector(Node):
    def __init__(self):
        super().__init__('green_object_detector')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(String, 'green_object_detected', 10)
        self.subscription = self.create_subscription(Image, 'sensor_msgs/image', self.image_callback, 10)
        self.depth_subscription = self.create_subscription(Image, 'sensor_msgs/depth', self.depth_callback, 10)
        self.depth_frame = None
        self.min_contour_area = 500
        self.color_intrin = None
        self.ghost_count = 0
        self.detected_ghosts = set()
        self.logger = self.get_logger()
        self.logger.info("Detection starts.")  # Initial message

    def depth_callback(self, msg):
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def get_object_distance(self, x, y):
        if self.depth_frame is None or self.color_intrin is None:
            return None
        depth = self.depth_frame[y, x]
        if depth == 0:
            return None
        point_3d = rs.rs2_deproject_pixel_to_point(self.color_intrin, [x, y], depth)
        return math.sqrt(point_3d[0]**2 + point_3d[1]**2 + point_3d[2]**2)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 100, 50])
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv_image, lower_green, upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        found_new_ghost = False

        for contour in contours:
            if cv2.contourArea(contour) > self.min_contour_area:
                x, y, w, h = cv2.boundingRect(contour)
                cx, cy = x + w // 2, y + h // 2
                ghost_id = (cx, cy)
                
                if ghost_id not in self.detected_ghosts:
                    self.detected_ghosts.add(ghost_id)
                    self.ghost_count += 1
                    distance = self.get_object_distance(cx, cy)
                    if distance:
                        message = f"Green ghost detected with {cv2.contourArea(contour)} pixels and {distance:.2f} meters."
                    else:
                        message = f"Green ghost detected with {cv2.contourArea(contour)} pixels but distance unknown."
                    self.publisher.publish(String(data=message))
                    self.logger.info(message)
                    found_new_ghost = True
                    cv2.putText(frame, f"No. {self.ghost_count}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

        if not found_new_ghost and not self.detected_ghosts:
            self.publisher.publish(String(data="No green ghost detected."))

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



