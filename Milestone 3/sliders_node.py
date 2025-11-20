import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math, time

class SlidersNode(Node):
    def __init__(self):
        super().__init__('sliders_node')
        self.pub = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.t0 = time.time()

    def timer_cb(self):
        t = time.time() - self.t0
        joints = [0.0]*6
        joints[0] = 0.6*math.sin(0.5*t)
        joints[1] = -0.4*math.sin(0.3*t)
        joints[2] = 0.2*math.sin(0.7*t)
        msg = Float64MultiArray()
        msg.data = joints
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SlidersNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
