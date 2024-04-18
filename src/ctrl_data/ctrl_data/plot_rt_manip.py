#!/usr/bin/env python3

from ctrl_data.franka_jacobian import get_jacobian

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt

class JacobianCalculator(Node):

    def __init__(self):
        super().__init__('jacobian_calculator')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Initialize lists to store time steps and overall products of singular values
        self.time_steps = []
        self.overall_products = []

        # Create a timer to update the plot periodically
        self.timer = self.create_timer(0.1, self.update_plot)

    def joint_state_callback(self, msg):
        joint_positions = msg.position

        # Call your Jacobian function with joint positions
        jacobian_matrix = get_jacobian(joint_positions)

        # Compute SVD of the Jacobian matrix
        _, singular_values, _ = np.linalg.svd(jacobian_matrix)

        # Compute the overall product of singular values
        overall_product = np.prod(singular_values)

        # Append the time step (length of overall_products) and overall product to the lists
        self.time_steps.append(len(self.overall_products))
        self.overall_products.append(overall_product)

    def update_plot(self):
        # Check if there is any data to plot
        if not self.time_steps or not self.overall_products:
            return

        # Update the plot with new data
        plt.plot(self.time_steps, self.overall_products, 'r-', linewidth = 2)
        plt.xlabel('Time Step', fontsize=18)
        plt.ylabel('Manipulability Measure', fontsize=20)
        plt.tick_params(axis='both', which='major', labelsize=20) 
        plt.grid(True)
        plt.ylim(0,0.1)
        plt.draw()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    jacobian_calculator = JacobianCalculator()
    rclpy.spin(jacobian_calculator)
    jacobian_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
