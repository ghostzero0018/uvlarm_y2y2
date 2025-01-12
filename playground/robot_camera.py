#!/usr/bin/env python3

import pyrealsense2 as rs
import signal
import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.signals import SignalHandlerOptions

# Global flag for interrupt handling
isOk = True

def signalInterruption(signum, frame):
    global isOk
    print("\nCtrl-C pressed, stopping the program.")
    isOk = False

# Register the signal handler
signal.signal(signal.SIGINT, signalInterruption)

# Main process flow
def main():
    global isOk
    
    # Initialize ROS node
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    rosNode = Node("RealSense_driver")
    camera = Realsense()
    camera.initializeROSnode(rosNode)

    # Processing loop
    while isOk and rclpy.ok():
        camera.read_imgs()
        camera.publish_imgs()
        rclpy.spin_once(rosNode, timeout_sec=0.001)

    # Cleanup resources
    try:
        camera.disconnect()
    except Exception as e:
        print(f"Failed to disconnect camera: {e}")
    try:
        rosNode.destroy_node()
    except Exception as e:
        print(f"Failed to destroy ROS node: {e}")
    try:
        rclpy.shutdown()
    except Exception as e:
        print(f"Failed to shutdown ROS: {e}")

# Class to handle RealSense camera operations
class Realsense:
    def __init__(self, fps=60):
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

        # Configure streams
        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, fps)
        self.pipeline.start(self.config)
        
    def initializeROSnode(self, ros_node):
        self.image_publisher = ros_node.create_publisher(Image, 'sensor_msgs/image', 10)
        self.depth_publisher = ros_node.create_publisher(Image, 'sensor_msgs/depth', 10)
        self.ros_node = ros_node

    def read_imgs(self):
        sys.stdout.write("-")
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            return

        # Process frames
        self.color_image = np.asanyarray(color_frame.get_data())
        self.depth_image = np.asanyarray(depth_frame.get_data())
        self.depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
                
    def publish_imgs(self):
        if not self.ros_node or not rclpy.ok():
            self.ros_node.get_logger().error("Cannot publish messages; invalid node or context.")
            return

        bridge = CvBridge()
        try:
            # Publish color image
            msg_image = bridge.cv2_to_imgmsg(self.color_image, "bgr8")
            msg_image.header.stamp = self.ros_node.get_clock().now().to_msg()
            msg_image.header.frame_id = "image"
            self.image_publisher.publish(msg_image)

            # Publish depth image
            msg_depth = bridge.cv2_to_imgmsg(self.depth_colormap, "bgr8")
            msg_depth.header.stamp = msg_image.header.stamp
            msg_depth.header.frame_id = "depth"
            self.depth_publisher.publish(msg_depth)
            
        except Exception as e:
            self.ros_node.get_logger().error(f"Failed to publish images: {e}")

    def disconnect(self):
        print("\nDisconnecting...")
        self.pipeline.stop()

# Script trigger
if __name__ == '__main__':
    main()


