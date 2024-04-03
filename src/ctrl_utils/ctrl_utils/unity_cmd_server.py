import rclpy
from rclpy.node import Node
from ctrl_interfaces.msg import UnityCommand
import subprocess

class UnityCommandServerNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.subscription = self.create_subscription(
            UnityCommand, 
            '/unity_command', 
            self.listener_callback, 
            1)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Command received: "%s"' % msg.command)
        if msg.command == "move_to_start":
            completed_process = subprocess.run(["ros2", "run", "sim_ctrl", "move_to_start"])
            if completed_process.returncode == 0:
                print("Completed successfully.\n")
            else:
                print("Terminated with an error.\n")
        else:
            self.get_logger().info('Unknown command: "%s"' % msg.command)


def main(args=None):
    rclpy.init(args=args)
    service_node = UnityCommandServerNode()
    rclpy.spin(service_node)
    service_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
