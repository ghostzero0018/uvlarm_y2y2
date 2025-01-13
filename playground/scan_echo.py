#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import math

class ScanProcessor(Node):
    def __init__(self):
        super().__init__('scan_echo')
        self.publisher = self.create_publisher(PointCloud, '/points', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        self.get_logger().info("Node initialized and ready to process scans.")

    def scan_callback(self, scanMsg):
        distance_threshold = 0.6  # Distance limit for obstacle detection (in meters)
        angle_detection = 65  # Angle limit for obstacle detection (in degrees)

        # Angles in radians for -45° and 45°
        angle_min = -angle_detection * (math.pi / 180)  # -π/4
        angle_max = angle_detection * (math.pi / 180)  # π/4

        # Calculate corresponding indices
        index_min = int((angle_min - scanMsg.angle_min) / scanMsg.angle_increment)
        index_max = int((angle_max - scanMsg.angle_min) / scanMsg.angle_increment)

        # Create a point cloud
        pointCloud = PointCloud()
        pointCloud.header = scanMsg.header

        for i in range(index_min, index_max + 1):
            if 0.05 < scanMsg.ranges[i] < distance_threshold:
                angle = scanMsg.angle_min + i * scanMsg.angle_increment
                aDistance = float(scanMsg.ranges[i])  # Explicit float conversion
                aPoint = Point32()
                aPoint.x = float(math.cos(angle) * aDistance)  # Explicit float conversion
                aPoint.y = float(math.sin(angle) * aDistance)  # Explicit float conversion
                aPoint.z = 0.0  # Ensure it's a float
                pointCloud.points.append(aPoint)
                # Uncomment the code below to log each point's coordinates
                # self.get_logger().info(f"Point: x = {aPoint.x}, y = {aPoint.y}")

        # Publish the point cloud
        self.publisher.publish(pointCloud)
        # Uncomment the code below to log the number of points published
        # self.get_logger().info(f"Published {len(pointCloud.points)} points.")

def main(args=None):
    rclpy.init(args=args)
    node = ScanProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# Script trigger
if __name__ == '__main__':
    main()



