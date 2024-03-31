import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from ctrl_interfaces.msg import FrankaJoints


class JointPositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_position_publisher')
        self.declare_parameter('hz', 200.0)
        self.publish_hz = self.get_parameter('hz').value

        # Subscribe to /joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        # Publisher for joint positions
        self.publisher = self.create_publisher(
            FrankaJoints,
            'franka_joints',
            10)

        # Create a timer to publish
        self.timer = self.create_timer(1.0 / self.publish_hz, self.timer_callback)
        self.last_positions = FrankaJoints()

    def listener_callback(self, msg):
        # Create an array to hold positions in order
        ordered_positions = [0.0] * 7
        for name, position in zip(msg.name, msg.position):
            if 'finger' in name:  # Ignore finger joints
                continue
            # Extract joint number and adjust for zero-based indexing
            joint_number = int(name[-1]) - 1
            ordered_positions[joint_number] = position

        # Update the last_positions with the ordered joint positions
        self.last_positions.joints = ordered_positions

    def timer_callback(self):
        # Publish the last_positions at 20Hz
        self.publisher.publish(self.last_positions)


def main(args=None):
    rclpy.init(args=args)
    joint_position_publisher = JointPositionPublisher()

    try:
        rclpy.spin(joint_position_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        joint_position_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
