#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class SlidersNode(Node):
    def __init__(self):
        super().__init__('sliders_node')
        self.pub = self.create_publisher(
            Float64MultiArray,
            'joint_commands',
            10
        )
        self.timer = self.create_timer(0.05, self.publish_cmd)
        self.t = 0.0

    def publish_cmd(self):
        msg = Float64MultiArray()
        # simple sinusoidal joint velocities
        msg.data = [
            0.3*np.sin(self.t),
            0.2*np.sin(self.t),
            0.2*np.sin(self.t),
            0.1*np.sin(self.t),
            0.1*np.sin(self.t),
            0.1*np.sin(self.t)
        ]
        self.pub.publish(msg)
        self.t += 0.05

def main():
    rclpy.init()
    node = SlidersNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
