import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from ctrl_interfaces.msg import FrankaJoints
from math import cos, pi
import time


panda_joint_names = ['panda_joint1',
                     'panda_joint2',
                     'panda_joint3',
                     'panda_joint4',
                     'panda_joint5',
                     'panda_joint6',
                     'panda_joint7']

sec = 0
long_sec = 1

nano_sec = 200 * 1000000  # n * 1ms
long_nano_sec = 300 * 1000000  # n * 1ms

nano_to_sec = 1000000000
smoothing_factor = 0.01
joints_change_threshold = 0.25
unity_pos_timeout = 1  # Threshold for message timeout in seconds


class JointTrajectoryControllerV3(Node):
    def __init__(self):
        super().__init__('joint_trajectory_client')
        self.client = ActionClient(self, FollowJointTrajectory, '/panda_arm_controller/follow_joint_trajectory')
        self.unity_pos_subscription = self.create_subscription(
            FrankaJoints,
            '/unity_franka_joints',
            self.unity_pos_listener_callback,
            10)
        self.unity_pos_subscription
        

        self.self_pos_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.self_pos_listener_callback,
            10)
        self.self_pos_subscription
    
        self.last_pos = None
        self.curr_pos = None
        self.last_unity_pos_time = None


    def send_goal(self, unity_pos):
        if self.last_pos is None:
            self.last_pos = self.curr_pos
        
        # Check if last_pos is outdated
        if self.last_unity_pos_time is not None:
            elapsed_time = time.time() - self.last_unity_pos_time
            if elapsed_time > unity_pos_timeout:
                self.get_logger().info('Unity position message timed out')
                self.last_pos = self.curr_pos
        
        # Computer diff
        diff = [(last - target) for last, target in zip(self.last_pos, unity_pos)]
        diff_joints_magnitude = 0.0
        for d in diff:
            diff_joints_magnitude += d * d
        diff_joints_magnitude = diff_joints_magnitude ** 0.5
        max_count = int(diff_joints_magnitude / smoothing_factor)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = panda_joint_names

        if (max_count > 1 and diff_joints_magnitude >= joints_change_threshold):
            for i in range(max_count + 1):
                point = JointTrajectoryPoint()
                w = 0.5 * (1 - cos(pi * i / (max_count - 1)))
                interpolated_positions = [
                    last + w * (last - current)
                    for last, current in zip(self.last_pos, unity_pos)
                ]

                point.positions = interpolated_positions

                total_nanoseconds = i * nano_sec
                additional_seconds = total_nanoseconds // nano_to_sec # Convert nanoseconds to full seconds
                remaining_nanoseconds = total_nanoseconds % nano_to_sec  # Remainder is less than one second in nanoseconds

                # Set the time for this trajectory point
                point.time_from_start.sec = sec + additional_seconds
                point.time_from_start.nanosec = remaining_nanoseconds
                goal_msg.trajectory.points.append(point)

            self.last_pos = unity_pos
            self.client.wait_for_server()
            self.send_goal_future = self.client.send_goal_async(goal_msg)
            self.send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            goal_msg.trajectory.points.append(JointTrajectoryPoint(positions=unity_pos))
            goal_msg.trajectory.points[0].time_from_start.sec = sec
            goal_msg.trajectory.points[0].time_from_start.nanosec = long_nano_sec
            
            self.last_pos = unity_pos
            self.client.wait_for_server()
            self.send_goal_future = self.client.send_goal_async(goal_msg)
            self.send_goal_future.add_done_callback(self.goal_response_callback)


    def unity_pos_listener_callback(self, msg):
        self.last_unity_pos_time = time.time()
        self.send_goal(msg.joints)

    def self_pos_listener_callback(self, msg):
        self.curr_pos = msg.position[:7]
    
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
    client = JointTrajectoryControllerV3()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
