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
        try:
            if msg.command == "gripper_home":
                self.get_logger().info('Command received: "%s"' % msg.command)
                # Start the subprocess with Popen
                process = subprocess.Popen(
                    ["ros2", "action", "send_goal", "/panda_gripper/homing", "franka_msgs/action/Homing", "{}"],
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
            elif msg.command == "gripper_grasp":
                self.get_logger().info('Command received: "%s"' % msg.command)
                # Start the subprocess with Popen
                process = subprocess.Popen(
                    ["ros2", "action", "send_goal", "-f", "/panda_gripper/grasp", "franka_msgs/action/Grasp",
                    "{width: 0.00, speed: 0.02, force: 50}"],
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )

            # Wait for the subprocess to complete within the timeout
            try:
                stdout, stderr = process.communicate(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()  # Terminate the process if it times out
                stdout, stderr = process.communicate()  # Collect any output after killing
                print(f"Command '{msg.command}' timed out.\n")
            else:
                if process.returncode == 0:
                    print(f"{msg.command} completed successfully.\n")
                else:
                    print(f"Error during {msg.command}: {stderr}\n")

        except Exception as e:
            print(f"An error occurred handling command '{msg.command}': {str(e)}\n")


def main(args=None):
    rclpy.init(args=args)
    service_node = GripperServer()
    rclpy.spin(service_node)
    service_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
