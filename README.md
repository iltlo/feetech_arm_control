# Feetech Arm Control

This repository provides a ROS2 hardware interface, URDF model, and MoveIt configuration for controlling a 6-DoF robotic arm using Feetech motors. The 3D model is currently in beta and pending open-source release by [Simple Automation](www.SimpleAutomation.ai), please follow their progress on [X](https://x.com/viktor_vrp)!

![Control Demo](docs/images/control_demo.gif)

## Installation

1. Clone the repository into your ROS2 workspace:
    ```bash
    cd ~/your_ros2_ws/src
    git clone https://github.com/iltlo/feetech_arm_control.git
    ```

2. Install dependencies:
    ```bash
    cd ~/your_ros2_ws/src/feetech_arm_control
    git submodule update --init --recursive
    pip install feetech-servo-sdk
    ```

3. Build and source the workspace:
    ```bash
    colcon build && source ~/your_ros2_ws/install/setup.bash
    ```

## Usage

### MoveIt! Demo

To run the MoveIt 2 demo for End Effector drag-and-drop control:
```bash
ros2 launch moveit_6dof_arm feetech_moveit.launch.py
```

To run with the actual hardware (Optional):
```bash
ros2 run arm_control hardware_interface
```

![moveit-screenshot](docs/images/moveit_demo.png)

### Display in RViz

To display the robot model in RViz:
```bash
ros2 launch urdf_6dof_arm display.launch.py
```

![rviz-screenshot](docs/images/rviz_joint_state_pub.png)

## Development Environment
- Ubuntu 22.04
- ROS2 Humble
