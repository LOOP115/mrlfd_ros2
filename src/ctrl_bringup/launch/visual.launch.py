from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    franka_joints_publisher = Node(
        package="ctrl_utils",
        executable="joints_publisher",
        parameters=[{"hz": 25.0}],
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
    
    
    return LaunchDescription(
        [
            franka_joints_publisher,
            gripper_server,
            manip_publisher
        ]
    )
