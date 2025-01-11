#!/usr/bin/python3
import rclpy
from  rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sys
import math
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32