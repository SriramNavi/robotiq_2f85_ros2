# Changelog
All notable changes to this project will be documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)
and this project adheres to [Semantic Versioning](https://semver.org/).

---

## [0.1.0] - 2025-02-XX
### Added
- Initial ROS 2 Jazzy + Gazebo Harmonic port of the Robotiq 2F-85 gripper.
- Converted original ROS1 xacro and description files to ROS2 structure.
- Implemented full `gz_ros2_control` integration.
- Added working `JointTrajectoryController`.
- Added `joint_state_broadcaster`.
- Added standalone simulation launch file (`launch_gripper_gazebo.py`).
- Added Gazebo Harmonic-compatible world and plugin configuration.
- Added complete YAML controller configs.
- Added initial README and MIT License.
- Added CI workflow template (GitHub Actions).
- Added ROS 2 command examples for opening/closing the gripper.

### Known Issues
- Gazebo GUI warnings related to Qt binding loops (cosmetic, upstream).
- Minor physics tolerances when closing gripper at extreme positions.

---

## [Unreleased]
### Planned
- Add Python API for scripted gripper control.
- Add URDF parameters for friction/contact tuning.
- Add example RViz + MoveIt2 configuration.
- Add video/GIF demonstration for README.
- Publish release tags for binary distributions.
