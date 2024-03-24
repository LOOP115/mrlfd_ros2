# franka_ctrl

<br>

## Instructions

### Dependencies

Please follow this [link](https://github.com/LOOP115/XR_Franka_Hub/blob/main/docs/franka/setup_franka.md) to setup dependencies before using this repository.

### Building

```bash
# Clone the repo and build
git clone https://github.com/LOOP115/franka_ctrl

# Build
cd franka_ctrl
colcon build
```

### Sourcing

Before using this package, remember to source the ROS 2 workspace.

```bash
source install/setup.bash
```

<br>

## Usage

```bash
# Follow a target inside Ignition Gazebo
ros2 launch ctrl_bringup sim_follow.launch.py
```

