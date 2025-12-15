#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray

class FuzzyController(Node):
    def __init__(self):
        super().__init__('fuzzy_controller')
        self.sub = self.create_subscription(
            Point, 'ee_error', self.cb, 10)
        self.pub = self.create_publisher(
            Float64MultiArray, 'joint_commands', 10)

    def rule(self, e):
        if abs(e) < 0.01:
            return 0.0
        elif e > 0:
            return 0.3
        else:
            return -0.3

    def cb(self, msg):
        u1 = self.rule(msg.x)
        u2 = self.rule(msg.y)

        cmd = Float64MultiArray()
        cmd.data = [u1, u2, 0, 0, 0, 0]
        self.pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(FuzzyController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
