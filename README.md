# XRFranka ROS2

This is the ROS2 endpoint for [XRFranka](https://github.com/LOOP115/Franka_XR_Hub), which enables launching of ROS2 programs to control Franka in Gazebo or the real world.

<br>

## Instructions

### Dependencies

Before using this repository. please make sure you have setup the following dependencies:

- [franka_ros2](https://github.com/LOOP115/Franka_XR_Hub/blob/main/docs/franka/franka_ros2.md)
- [Gazebo](https://github.com/LOOP115/Franka_XR_Hub/blob/main/docs/franka/gazebo.md)

### Building

```bash
# Clone the repo to anywhere you like, the repo itself is a ROS2 workspace
git clone https://github.com/LOOP115/xrfranka_ros2

# Build
cd xrfranka_ros2
colcon build
```

### Sourcing

Before using this package, remember to source it.

```bash
source install/setup.bash
```

You can also add a line to `~/.bashrc` like this:

```bash
gedit ~/.bashrc
```

```bash
source ~/project/xrfranka_ros2/install/setup.bash
```

<br>

## Usage

- Make sure the ROS machine and the machine running the Unity application are on the same Wi-Fi network.
- Get ROS machine's IP

```bash
hostname -I
```

- Start ROS TCP Endpoint

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=<ros_ip>
```

- Start data recorder, remember to modify the recording details

```bash
ros2 run ctrl_utils data_recorder
```

### Kinesthetic Teaching

- First, launch nodes for visualizations

```bash
ros2 launch ctrl_bringup visual.launch.py
```

- Then, launch the gravity compensation controller

```bash
ros2 launch franka_bringup gravity_compensation_example_controller.launch.py robot_ip:=<robot_ip>
```

### Teleoperation

- First, launch necessary nodes

```bash
ros2 launch ctrl_bringup all_nodes.launch.py
```

- Then, launch the Franka

```bash
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=<robot_ip>
```

<br>

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.
