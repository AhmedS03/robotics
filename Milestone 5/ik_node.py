#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')

        self.q = np.zeros(6)

        self.sub_js = self.create_subscription(
            JointState, 'joint_state', self.js_cb, 10)

        self.sub_target = self.create_subscription(
            Point, 'ee_target', self.target_cb, 10)

        self.pub = self.create_publisher(
            Float64MultiArray, 'joint_commands', 10)

    def js_cb(self, msg):
        self.q = np.array(msg.position)

    def target_cb(self, msg):
        # Desired task-space velocity
        v = np.array([msg.x, msg.y, msg.z])

        # Simplified Jacobian (LECTURE-ACCEPTED)
        J = np.eye(6) * 0.1
        dq = np.linalg.pinv(J) @ np.hstack((v, [0,0,0]))

        cmd = Float64MultiArray()
        cmd.data = dq.tolist()
        self.pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(IKNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
