import rclpy
from rclpy.node import Node
from ctrl_interfaces.msg import UnityCommand
import subprocess

class GripperServer(Node):
    def __init__(self):
        super().__init__('gripper_server')
        self.subscription = self.create_subscription(
            UnityCommand, 
            '/unity_command', 
            self.listener_callback, 
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Command received: "%s"' % msg.command)
        
        try:
            if msg.command == "gripper_home":
                # Execute the ROS 2 action to home the gripper with a timeout
                completed_process = subprocess.run(
                    ["ros2", "action", "send_goal", "/panda_gripper/homing", "franka_msgs/action/Homing", "{}"],
                    text=True,
                    capture_output=True,
                    timeout=10  # Timeout in seconds
                )
                # Check if the subprocess completed successfully
                if completed_process.returncode == 0:
                    print("Gripper homing completed successfully.\n")
                else:
                    print(f"Error during gripper homing: {completed_process.stderr}\n")

            elif msg.command == "gripper_grasp":
                # Execute the ROS 2 action to grasp with specified parameters with a timeout
                completed_process = subprocess.run(
                    ["ros2", "action", "send_goal", "-f", "/panda_gripper/grasp", "franka_msgs/action/Grasp", 
                     "{width: 0.00, speed: 0.05, force: 50}"],
                    text=True,
                    capture_output=True,
                    timeout=10  # Timeout in seconds
                )
                # Check if the subprocess completed successfully
                if completed_process.returncode == 0:
                    print("Gripper grasp completed successfully.\n")
                else:
                    print(f"Error during gripper grasp: {completed_process.stderr}\n")

        except subprocess.TimeoutExpired:
            print(f"Command '{msg.command}' timed out.\n")


def main(args=None):
    rclpy.init(args=args)
    service_node = GripperServer()
    rclpy.spin(service_node)
    service_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
