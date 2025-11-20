# minimal logger: publishes a trajectory, records FK and velocities for offline comparison
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np, time
from mujoco_ros2.dh_fk import forward_kinematics
from mujoco_ros2.kinematics import end_effector_velocity

class Validator(Node):
    def __init__(self):
        super().__init__('validator')
        self.pub = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        self.timer = self.create_timer(0.02, self.cb)
        self.t0 = time.time()
        self.log = []

    def cb(self):
        t = time.time()-self.t0
        q = np.array([0.2*np.sin(0.5*t), -0.3*np.sin(0.3*t), 0.1*np.sin(0.7*t), 0,0,0])
        msg = Float64MultiArray()
        msg.data = q.tolist()
        self.pub.publish(msg)
        Ts,T = forward_kinematics(q)
        # approximate q_dot numerically for velocity test
        qdot = np.zeros(6) # simple example
        v = end_effector_velocity(q, qdot)
        self.log.append((t, q.tolist(), T[:3,3].tolist(), v.tolist()))
        if len(self.log) > 500:
            # save and exit
            import json
            with open('validation_log.json','w') as f:
                json.dump(self.log, f)
            self.get_logger().info('Saved log; shutting down.')
            rclpy.shutdown()

def main():
    rclpy.init()
    node = Validator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()

if __name__ == "__main__":
    main()

