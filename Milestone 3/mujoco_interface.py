import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import os, numpy as np

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None

try:
    import mujoco
    from mujoco import mjcf, MjSim
    MUJOCO_AVAILABLE = True
except Exception:
    MUJOCO_AVAILABLE = False

class MuJoCoInterface(Node):
    def __init__(self):
        super().__init__('mujoco_interface')
        self.declare_parameter('model_path', 'models/scene.xml')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not os.path.isabs(model_path) and get_package_share_directory is not None:
            pkg = get_package_share_directory('mujoco_ros2')
            model_path = os.path.join(pkg, model_path)
        self.get_logger().info(f'Using model: {model_path}')
        self.joint_sub = self.create_subscription(Float64MultiArray, 'joint_commands', self.joint_callback, 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.02, self.timer_cb)
        self.joint_positions = np.zeros(6)
        self.sim = None
        if MUJOCO_AVAILABLE:
            try:
                self.model = mjcf.from_xml_path(model_path)
                self.sim = MjSim(self.model)
                self.get_logger().info('Loaded MuJoCo model successfully.')
            except Exception as e:
                self.get_logger().error(f'Failed to load MuJoCo model: {e}')
                self.sim = None
        else:
            self.get_logger().warning('MuJoCo python bindings not available — dry-mode')

    def joint_callback(self, msg):
        data = msg.data
        if len(data) >= 6:
            self.joint_positions = np.array(data[:6], dtype=float)

    def timer_cb(self):
        if self.sim is not None:
            try:
                for i in range(min(self.sim.model.nq, 6)):
                    self.sim.data.qpos[i] = float(self.joint_positions[i])
                self.sim.step()
                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                js.name = [f'joint_{i+1}' for i in range(min(self.sim.model.nq,6))]
                js.position = [float(self.sim.data.qpos[i]) for i in range(min(self.sim.model.nq,6))]
                self.joint_pub.publish(js)
            except Exception as e:
                self.get_logger().error(f'Sim step error: {e}')
        else:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [f'joint_{i+1}' for i in range(6)]
            js.position = list(self.joint_positions)
            self.joint_pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoInterface()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
