from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    franka_joints_publisher = Node(
        package="ctrl_utils",
        executable="joints_publisher",
        parameters=[{"hz": 25.0}],
    )

    move_to_start_server = Node(
        package="ctrl_utils",
        executable="move_to_start_server",
    )

    gripper_server = Node(
        package="ctrl_utils",
        executable="gripper_server",
    )
    
    manip_publisher = Node(
        package="ctrl_data",
        executable="manip_publisher",
        parameters=[{"hz": 5.0}],
    )

    follow_trajectory = Node(
        package="real_ctrl",
        executable="follow_trajectory",
    )

    move_to_pose = Node(
        package="real_ctrl",
        executable="move_to_pose",
    )

    follow_unity_target = Node(
        package="real_ctrl",
        executable="follow_unity_target",
    )
    
    
    return LaunchDescription(
        [
            franka_joints_publisher,
            move_to_start_server,
            gripper_server,
            manip_publisher,
            follow_trajectory,
            move_to_pose,
            follow_unity_target
        ]
    )
