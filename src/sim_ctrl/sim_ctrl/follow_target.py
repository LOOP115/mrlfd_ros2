import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from ctrl_interfaces.srv import MoveToPose


class FollowTargetNode(Node):
    def __init__(self):
        super().__init__('move_to_pose_client')
        self.client = self.create_client(MoveToPose, 'move_to_pose')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('move_to_pose service not available, waiting again...')
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.pose_callback,
            10)
        
        self.last_pose = None

    
    def similar_position(self, curr_pose: Pose, threshold=0.01):
        """Check if two poses are similar within a certain threshold."""
        position_close = all(
            abs(getattr(self.last_pose.position, attr) - getattr(curr_pose.position, attr)) < threshold
            for attr in ['x', 'y', 'z']
        )
        return position_close


    def pose_callback(self, msg: PoseStamped):
        # Check if the pose is significantly different from the last pose
        if self.last_pose is None or not self.similar_position(msg.pose):
            
            # Extract position and orientation from the incoming message
            position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            quat_xyzw = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            
            self.get_logger().info(f"New target pose received: {position}, {quat_xyzw}")
            self.send_request(position, quat_xyzw, True)  # Assuming cartesian is always False for simplicity
            self.last_pose = msg.pose


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
    node = FollowTargetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
