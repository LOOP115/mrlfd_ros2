import rclpy
from rclpy.node import Node
from ctrl_interfaces.msg import UnityCommand
import subprocess

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
        self.get_logger().info('Command received: "%s"' % msg.command)

        try:
            if msg.command == "move_to_start":
                completed_process = subprocess.run(
                    ["ros2", "run", "real_ctrl", "move_to_start"],
                    text=True,
                    capture_output=True,
                    timeout=10
                )
                # completed_process = subprocess.run(
                #     ["ros2", "run", "sim_ctrl", "move_to_start"],
                #     text=True,
                #     capture_output=True,
                #     timeout=10
                # )
                if completed_process.returncode == 0:
                    print("Completed successfully.\n")
                else:
                    print("Terminated with an error.\n")
        except subprocess.TimeoutExpired:
            print(f"Command '{msg.command}' timed out.\n")


def main(args=None):
    rclpy.init(args=args)
    service_node = MoveToStartServer()
    rclpy.spin(service_node)
    service_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
