#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import mujoco
import numpy as np
import threading
import time
import os


class MujocoInterface(Node):

    def __init__(self):
        super().__init__("mujoco_interface")

        # Parameters from launch file
        self.declare_parameter("scene_path", "")
        self.scene_path = self.get_parameter("scene_path").value

        # Load model
        if not os.path.exists(self.scene_path):
            raise FileNotFoundError(f"Scene file not found: {self.scene_path}")

        self.model = mujoco.MjModel.from_xml_path(self.scene_path)
        self.data = mujoco.MjData(self.model)

        self.get_logger().info("MuJoCo model loaded (Python side).")

        # Subscribers + Publishers
        self.joint_pub = self.create_publisher(JointState, "joint_state", 20)
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            "joint_commands",
            self.command_callback,
            10
        )

        # Internal state
        self.cmd_vel = np.zeros(self.model.nu)

        # Start simulation thread
        self.running = True
        self.sim_thread = threading.Thread(target=self.simulation_loop)
        self.sim_thread.start()

    def command_callback(self, msg):
        """Stores incoming velocity commands."""
        arr = np.array(msg.data, dtype=float)
        if len(arr) == self.model.nu:
            self.cmd_vel = arr
        else:
            self.get_logger().warn("Received incorrect joint command dimension.")

    def simulation_loop(self):
        """Runs mj_step in a loop (1 kHz)."""
        rate = 0.001  # 1 kHz
        while self.running:
            # Apply velocities
            self.data.qvel[:] = self.cmd_vel

            # Step simulation
            mujoco.mj_step(self.model, self.data)

            # Publish joint states
            self.publish_joint_states()

            time.sleep(rate)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Joint names from model
        msg.name = [self.model.joint(j).name for j in range(self.model.njnt)]

        msg.position = np.copy(self.data.qpos[:self.model.njnt]).tolist()
        msg.velocity = np.copy(self.data.qvel[:self.model.njnt]).tolist()

        self.joint_pub.publish(msg)

    def stop(self):
        self.running = False
        self.sim_thread.join()


def main():
    rclpy.init()
    node = MujocoInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

