import rclpy
from rclpy.node import Node
from ctrl_interfaces.msg import PosTarget
from ctrl_interfaces.srv import MoveToPose


class FollowUnityTargetNode(Node):
    def __init__(self):
        super().__init__('move_to_pose_client')
        self.client = self.create_client(MoveToPose, 'move_to_pose')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('move_to_pose service not available, waiting again...')
        
        self.subscription = self.create_subscription(
            PosTarget,
            '/unity_target_pose',
            self.pose_callback,
            10)
        
        self.last_pose = None

    
    def similar_position(self, curr_pose: PosTarget, threshold=0.01):
        """Check if two poses are similar within a certain threshold."""
        position_close = all(
            abs(getattr(self.last_pose, attr) - getattr(curr_pose, attr)) < threshold
            for attr in ['pos_x', 'pos_y', 'pos_z']
        )
        return position_close


    def pose_callback(self, msg: PosTarget):
        # Check if the pose is different from the last pose
        if self.last_pose is None or not self.similar_position(msg):
            
            # Extract position and orientation from the incoming message
            position = [msg.pos_x, msg.pos_y, msg.pos_z]
            quat_xyzw = [msg.rot_x, msg.rot_y, msg.rot_z, msg.rot_w]
            self.get_logger().info(f"New target pose received: {position}, {quat_xyzw}")
            self.send_request(position, quat_xyzw, False)  # Assuming cartesian is always False for simplicity
            self.last_pose = msg


    def send_request(self, position, quat_xyzw, cartesian):
        request = MoveToPose.Request()
        request.position = position
        request.quat_xyzw = quat_xyzw
        request.cartesian = cartesian
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.future_done_callback)


    def future_done_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service call succeeded: {response.success}. Message: '{response.message}'")
        except Exception as e:
            self.get_logger().info(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FollowUnityTargetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
