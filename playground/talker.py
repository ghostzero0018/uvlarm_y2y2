#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def infiniteTalk():
    # Initialize ROS node with ROS client
    rclpy.init()
    aNode= Node( "infTalker" )
    talker= ROSTalker(aNode)
    # Start infinite loop
    rclpy.spin(aNode)
    # Clean everything and switch the light off
    aNode.destroy_node()
    rclpy.shutdown()

class ROSTalker:
    def __init__(self, rosNode):
        self._publisher= rosNode.create_publisher( String, 'testTopic', 10 )
        self._timer = rosNode.create_timer(0.5, self.timer_callback)
        self._i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self._i
        self._publisher.publish(msg)
        self._i += 1

# Execute the function.
if __name__ == "__main__":
    infiniteTalk()