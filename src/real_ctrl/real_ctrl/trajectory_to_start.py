import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from threading import Thread


class TrajectoryToStart(Node):
    def __init__(self):
        super().__init__('trajectory_to_start')
        self.client = ActionClient(self, FollowJointTrajectory, '/panda_arm_controller/follow_joint_trajectory')

        self.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 
                            'panda_joint5', 'panda_joint6', 'panda_joint7']

        self.start_positions = [
            0.0,
            -0.7853981633974483,
            0.0,
            -2.356194490192345,
            0.0,
            1.5707963267948966,
            0.7853981633974483,
        ]

        self.exec_sec = 5

    def send_start_positions(self):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.start_positions
        point.time_from_start = Duration(sec=self.exec_sec)  # Allow 5 seconds to reach the start position
        goal_msg.trajectory.points.append(point)

        self.client.wait_for_server()
        self.send_goal_future = self.client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.shutdown_node()  # Shutdown if goal is rejected
            return

        self.get_logger().info('Goal accepted, moving to start positions')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == result.SUCCESSFUL:
            self.get_logger().info('Successfully moved to start positions')
        else:
            self.get_logger().info('Failed to move to start positions: error code {}'.format(result.error_code))
        self.shutdown_node()

    def shutdown_node(self):
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryToStart()
    node.send_start_positions()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # rclpy.shutdown()
    executor_thread.join()
    exit(0)

if __name__ == '__main__':
    main()
