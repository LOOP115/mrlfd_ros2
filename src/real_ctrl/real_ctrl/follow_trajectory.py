import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ctrl_interfaces.msg import FrankaJoints


panda_joint_names = ['panda_joint1',
                     'panda_joint2',
                     'panda_joint3',
                     'panda_joint4',
                     'panda_joint5',
                     'panda_joint6',
                     'panda_joint7']

sec = 0
nano_sec = 100 * 1000000  # n * 1ms


class JointTrajectoryClient(Node):
    def __init__(self):
        super().__init__('joint_trajectory_client')
        self.client = ActionClient(self, FollowJointTrajectory, '/panda_arm_controller/follow_joint_trajectory')
        self.subscription = self.create_subscription(
            FrankaJoints,  # Use the custom message type
            '/unity_franka_joints',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.last_positions = None

    def send_goal(self, current_positions):
        if self.last_positions is None:
            self.last_positions = current_positions  # Initialize last_positions on first receive
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = panda_joint_names
        num_interpolations = 20

        for i in range(num_interpolations + 1):
            point = JointTrajectoryPoint()
            interpolated_positions = [
                last + (current - last) * i / num_interpolations
                for last, current in zip(self.last_positions, current_positions)
            ]
            point.positions = interpolated_positions
            point.time_from_start.sec = sec
            point.time_from_start.nanosec = i * nano_sec
            goal_msg.trajectory.points.append(point)

        self.last_positions = current_positions  # Update the last_positions with the current

        self.client.wait_for_server()
        self.send_goal_future = self.client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def listener_callback(self, msg):
        self.send_goal(msg.joints)  # Access the joints field of the custom message

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        # self.get_logger().info('Goal accepted')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code != result.SUCCESSFUL:
            # self.get_logger().info('Goal succeeded!')
            self.get_logger().info('Goal failed with error code: {0}'.format(result.error_code))

def main(args=None):
    rclpy.init(args=args)
    client = JointTrajectoryClient()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
