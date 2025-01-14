#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
import math

class ScanProcessor(Node):
    def __init__(self):
        super().__init__('scan_echo')
        # Publisher for PointCloud messages
        self.publisher = self.create_publisher(PointCloud, '/points', 10)
        # Subscription to LaserScan messages
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )
        # Logging initialization message
        self.get_logger().info("Node initialized and ready to process scans.")

    def scan_callback(self, scanMsg):
        distance_threshold = 0.6  # Distance limit for obstacle detection (meters)
        angle_detection = 45  # Angle limit for obstacle detection (degrees)

        # Calculate angles in radians for -45° to 45°
        angle_min = -angle_detection * (math.pi / 180)
        angle_max = angle_detection * (math.pi / 180)

        # Calculate corresponding indices
        index_min = int((angle_min - scanMsg.angle_min) / scanMsg.angle_increment)
        index_max = int((angle_max - scanMsg.angle_min) / scanMsg.angle_increment)

        # Create a point cloud to store detected points
        pointCloud = PointCloud()
        pointCloud.header = scanMsg.header

        for i in range(index_min, index_max + 1):
            if 0.05 < scanMsg.ranges[i] < distance_threshold:
                angle = scanMsg.angle_min + i * scanMsg.angle_increment
                aDistance = scanMsg.ranges[i]
                aPoint = Point32(x=math.cos(angle) * aDistance,
                                 y=math.sin(angle) * aDistance,
                                 z=0.0)
                pointCloud.points.append(aPoint)

        # Publish the processed point cloud
        self.publisher.publish(pointCloud)

def main(args=None):
    rclpy.init(args=args)
    node = ScanProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# Script trigger
if __name__ == '__main__':
    main()



