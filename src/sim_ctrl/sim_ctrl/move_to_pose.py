#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
from ctrl_interfaces.srv import MoveToPose  # Adjust this import based on your actual package and service name

from pymoveit2 import MoveIt2
from pymoveit2.robots import panda


class MoveToPoseServerNode(Node):
    def __init__(self):
        super().__init__('move_to_pose_server')
        self.service = self.create_service(MoveToPose, 'move_to_pose', self.move_to_pose_callback)
        self.get_logger().info("MoveToPose service is ready.")

        self.callback_group = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        # You might need to adjust these parameters or make them configurable via service request
        self.moveit2.planner_id = "RRTConnectkConfigDefault"
        self.moveit2.max_velocity = 1.0
        self.moveit2.max_acceleration = 1.0

    def move_to_pose_callback(self, request, response):
        position = request.position
        quat_xyzw = request.quat_xyzw
        cartesian = request.cartesian

        self.get_logger().info(f"Moving to {{position: {position}, quat_xyzw: {quat_xyzw}, cartesian: {cartesian}}}")

        # Perform the move operation
        try:
            self.moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
            self.moveit2.wait_until_executed()  # Assuming you want to wait for execution to complete
            response.success = True
            response.message = "Movement executed successfully."
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPoseServerNode()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        executor_thread.join()
    except KeyboardInterrupt:
        pass

    # Shutdown and cleanup
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
