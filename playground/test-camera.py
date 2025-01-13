#!/usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import cv2
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import signal

# Global flag for interrupt handling
isOk = True

def signalInterruption(signum, frame):
    global isOk
    print("\nCtrl-C pressed, stopping the program.")
    isOk = False

signal.signal(signal.SIGINT, signalInterruption)

class RealSenseCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))
        
        print(f"Connected to: {self.device_product_line}")
        self.found_rgb = any(s.get_info(rs.camera_info.name) == 'RGB Camera' for s in self.device.sensors)
        
        if not self.found_rgb:
            print("An RGB camera is required but not found. Exiting.")
            sys.exit(0)

        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
        self.pipeline.start(self.config)
        
    def capture_images(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return None, None

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        
        return color_image, depth_colormap

    def stop(self):
        self.pipeline.stop()

def main(args=None):
    rclpy.init(args=args)
    ros_node = Node('RealSenseCameraNode')
    camera = RealSenseCamera()
    bridge = CvBridge()

    image_publisher = ros_node.create_publisher(Image, 'sensor_msgs/image', 10)
    depth_publisher = ros_node.create_publisher(Image, 'sensor_msgs/depth', 10)

    try:
        while isOk and rclpy.ok():
            color_image, depth_colormap = camera.capture_images()
            if color_image is not None and depth_colormap is not None:
                # Publish color image
                msg_image = bridge.cv2_to_imgmsg(color_image, "bgr8")
                msg_image.header.stamp = ros_node.get_clock().now().to_msg()
                msg_image.header.frame_id = "image"
                image_publisher.publish(msg_image)

                # Publish depth image
                msg_depth = bridge.cv2_to_imgmsg(depth_colormap, "bgr8")
                msg_depth.header.stamp = msg_image.header.stamp
                msg_depth.header.frame_id = "depth"
                depth_publisher.publish(msg_depth)

                cv2.imshow('RealSense', np.hstack((color_image, depth_colormap)))
                cv2.waitKey(1)

    finally:
        cv2.destroyAllWindows()
        camera.stop()
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
