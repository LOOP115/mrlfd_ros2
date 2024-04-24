import rclpy
from rclpy.node import Node
from ctrl_interfaces.msg import UnityCommand
import subprocess


# cmd = [
#     "ros2", "run", "pymoveit2", "ex_joint_goal.py",
#     "--ros-args",
#     "-p", "joint_positions:=[0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483]"
# ]

class MoveToStartServer(Node):
    def __init__(self):
        super().__init__('move_to_start_server')
        self.subscription = self.create_subscription(
            UnityCommand, 
            '/unity_command', 
            self.listener_callback, 
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        try:
            if msg.command == "move_to_start":
                self.get_logger().info('Command received: "%s"' % msg.command)
                # Start the subprocess with Popen
                process = subprocess.Popen(
                    ["ros2", "run", "real_ctrl", "trajectory_to_start"],
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )

                try:
                    # Wait for the subprocess to finish within the timeout
                    stdout, stderr = process.communicate(timeout=15)
                except subprocess.TimeoutExpired:
                    # Handle the timeout case by terminating the process
                    process.kill()
                    stdout, stderr = process.communicate()  # Ensure you get the outputs after killing
                    self.get_logger().info(f"Command '{msg.command}' timed out.")
                else:
                    # Process completed within the timeout
                    if process.returncode == 0:
                        self.get_logger().info("Completed successfully.\n")
                    else:
                        self.get_logger().info(f"Command '{msg.command}' failed with return code {process.returncode}.\n")

        except Exception as e:
            self.get_logger().info(f"An error occurred: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    service_node = MoveToStartServer()
    rclpy.spin(service_node)
    service_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
