import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class CommanderNode(Node):
    def __init__(self):
        super().__init__('ur5e_commander_node')
        
        # Create a publisher to the topic the robot controller listens to
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)
        
        # Create a timer to call the send_command function after 2 seconds
        self.timer = self.create_timer(2.0, self.send_command)
        self.get_logger().info('Commander node started. Sending command in 2 seconds...')

    def send_command(self):
        # --- This is where you put the joint angles you want the robot to move to ---
        # Use your inverse_kinematics.py to calculate these angles for a target position
        goal_positions = [0.5, -1.0, 1.2, -0.8, 1.57, 0.5]

        # Create the message
        traj_msg = JointTrajectory()
        
        # IMPORTANT: The joint names must be in the same order as the goal_positions
        traj_msg.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]

        # Create a trajectory point
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in goal_positions]
        point.time_from_start = Duration(sec=4, nanosec=0) # Time to reach the goal

        # Add the point to the trajectory
        traj_msg.points.append(point)

        # Publish the message
        self.publisher_.publish(traj_msg)
        self.get_logger().info(f'Publishing command: {goal_positions}')
        
        # Destroy the timer after sending the command to only run once
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    commander_node = CommanderNode()
    rclpy.spin(commander_node)
    commander_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
