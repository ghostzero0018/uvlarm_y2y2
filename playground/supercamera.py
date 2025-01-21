#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import pyrealsense2 as rs
import cv2
import numpy as np
import math
import signal


# Realsense Node
class Realsense(Node):
    def __init__(self, fps=60):
        super().__init__('realsense')
        # Publishers
        self.image_publisher = self.create_publisher(Image, 'image', 10)
        self.detection_publisher = self.create_publisher(String, 'detection', 10)
        self.distancebottle_publisher = self.create_publisher(Float32, 'distancebottle', 10)
        self.x_publisher = self.create_publisher(Float32, 'x', 10)
        self.y_publisher = self.create_publisher(Float32, 'z', 10)

        # Initialize RealSense pipeline
        self.bridge = CvBridge()
        self.pipeline = rs.pipeline()
        self.colorizer = rs.colorizer()
        config = rs.config()
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()

        # Check if RGB camera is available
        found_rgb = any(s.get_info(rs.camera_info.name) == 'RGB Camera' for s in device.sensors)
        if not found_rgb:
            print("Depth camera required!")
            exit(0)

        config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, fps)

        # Align depth to color
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        # Start streaming
        self.pipeline.start(config)
        self.isOk = True
        self.color_info = (0, 0, 255)

    def read_imgs(self):
        frames = self.pipeline.wait_for_frames()

        # Align color frame to depth frame
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            return

        # Get images
        depth_image = np.asanyarray(depth_frame.get_data())
        self.color_image = np.asanyarray(color_frame.get_data())

        # Convert to HSV and apply mask
        hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        mask = ((50 < h) & (h < 85) & (s > 100) & (v > 50)).astype(np.uint8) * 255

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)
        mask = cv2.blur(mask, (7, 7))

        # Detect objects
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            for contour in contours:
                ((x, y), radius) = cv2.minEnclosingCircle(contour)
                if radius > 10:  # Adjust threshold for small object elimination
                    self.draw_object(x, y, radius, depth_frame, color_frame)

        # Display images
        cv2.imshow('RealSense', self.color_image)
        cv2.waitKey(1)

    def draw_object(self, x, y, radius, depth_frame, color_frame):
        x, y = int(x), int(y)
        depth = depth_frame.get_distance(x, y)

        if depth > 0:  # Only calculate distance if valid depth is available
            intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
            dx, dy, dz = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)
            distance = math.sqrt(dx**2 + dy**2 + dz**2)

            # Draw object markers
            cv2.circle(self.color_image, (x, y), int(radius), self.color_info, 2)
            cv2.putText(self.color_image, f"Dist: {distance:.2f}m", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color_info, 2)

    def publish_imgs(self):
        # Publish processed image
        msg_image = self.bridge.cv2_to_imgmsg(self.color_image, "bgr8")
        self.image_publisher.publish(msg_image)

    def signalInterruption(self, signum, frame):
        print("\nCtrl-C pressed")
        self.isOk = False


def process_img(args=None):
    rclpy.init(args=args)
    rsNode = Realsense()
    signal.signal(signal.SIGINT, rsNode.signalInterruption)

    while rsNode.isOk:
        rsNode.read_imgs()
        rsNode.publish_imgs()
        rclpy.spin_once(rsNode, timeout_sec=0.001)

    # Clean shutdown
    rsNode.pipeline.stop()
    rsNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    process_img()