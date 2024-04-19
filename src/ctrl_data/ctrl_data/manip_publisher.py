from ctrl_data.franka_jacobian import get_jacobian

import numpy as np
import rclpy
from rclpy.node import Node
from ctrl_interfaces.msg import Manipulability, FrankaJoints
import time


class ManipulabilityPublisher(Node):

    def __init__(self):
        super().__init__('manipulability_publisher')
        self.declare_parameter('hz', 5.0)
        self.publish_hz = self.get_parameter('hz').value

        self.publisher = self.create_publisher(
            Manipulability,
            '/manipulability',
            10
        )
        # self.timer = self.create_timer(1 / self.publish_hz, self.publish_manipulability)
        self.manipulability = None

        self.subscription = self.create_subscription(
            FrankaJoints,
            '/franka_joints',
            self.joint_state_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.last_time = time.time()
        self.throttle_period = 1 / self.publish_hz


    def joint_state_callback(self, msg):
        current_time = time.time()
        if (current_time - self.last_time) > self.throttle_period:
            joint_positions = msg.joints

            # Call your Jacobian function with joint positions
            jacobian_matrix = get_jacobian(joint_positions)

            # Compute SVD of the Jacobian matrix
            _, singular_values, _ = np.linalg.svd(jacobian_matrix)

            # Compute the overall product of singular values
            self.manipulability = np.prod(singular_values)

            self.publish_manipulability()

            self.last_time = current_time


    def publish_manipulability(self):
        if self.manipulability is not None:
            msg = Manipulability()
            msg.value = self.manipulability
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    manipulability_publisher = ManipulabilityPublisher()
    rclpy.spin(manipulability_publisher)
    manipulability_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
