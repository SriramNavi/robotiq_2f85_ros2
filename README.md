# Robotiq 2F-85 Gripper --- ROS 2 Jazzy + Gazebo Harmonic

A fully-working ROS 2 Jazzy + Gazebo Harmonic port of the official ROS1
Robotiq 2F-85 package.\
This version includes:

-   Updated URDF/Xacro\
-   Working `gz_ros2_control` plugin\
-   JointTrajectory controller\
-   Gazebo Harmonic support\
-   Clean launch pipeline\
-   Compatible command API for opening/closing the gripper

------------------------------------------------------------------------

## 📦 Packages

    robotiq_description
    robotiq_gripper_gazebo

-   `robotiq_description`: Xacro + meshes\
-   `robotiq_gripper_gazebo`: Gazebo simulation + controllers

------------------------------------------------------------------------

## 🚀 Launch the Simulation

``` bash
ros2 launch robotiq_gripper_gazebo launch_gripper_gazebo.py
```

### Arguments

  Argument                     Default            Description
  ---------------------------- ------------------ ------------------------
  `gripper_description_file`   URDF/Xacro         Robot description
  `controllers_file`           YAML               Controller config
  `gazebo_gui`                 true               Launch Gazebo with GUI
  `world_file`                 bullet_world.sdf   Custom world

------------------------------------------------------------------------

## ✋ Control the Gripper

### **Open the Gripper**
```bash
ros2 topic pub --once /robotiq_gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "joint_names:
- 'robotiq_85_left_knuckle_joint'
points:
- positions: [0.0]
  time_from_start: {sec: 2, nanosec: 0}
```

### **Close the Gripper**

``` bash
ros2 topic pub --once /robotiq_gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "joint_names:
- 'robotiq_85_left_knuckle_joint'
points:
- positions: [0.8]
  time_from_start: {sec: 2, nanosec: 0}
```

------------------------------------------------------------------------

## 📂 Build

``` bash
colcon build
source install/setup.bash
```

------------------------------------------------------------------------

## 📜 License

MIT License --- see `LICENSE`.

------------------------------------------------------------------------

## 🤝 Contributions

PRs welcome.

Tested on:\
- Ubuntu 24.04\
- ROS 2 Jazzy\
- Gazebo Harmonic

------------------------------------------------------------------------

## ⭐ Acknowledgment

Original ROS1 package © Robotiq.\
This project is an independent ROS 2 port.
