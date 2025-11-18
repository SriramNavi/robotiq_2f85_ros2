# Changelog
All notable changes to this project will be documented in this file.

This project follows semantic versioning: MAJOR.MINOR.PATCH

---

## [0.1.0] - 2025-11-18
### Added
- Initial ROS 2 port of the Robotiq 2F-85 gripper for **ROS 2 Jazzy** and **Gazebo Harmonic**.
- Complete `robotiq_description` package with original URDF/Xacro files.
- `robotiq_gripper_gazebo` package including:
  - Gazebo Harmonic-compatible SDF and world files.
  - ROS 2 control integration via `gz_ros2_control`.
  - Working `joint_state_broadcaster`.
  - Working `robotiq_gripper_controller` using `JointTrajectoryController`.
  - TF tree publication through robot_state_publisher.
  - Automatic spawning in Gazebo + delayed controller load.
  - Clock bridging (`/clock`) from Gazebo → ROS 2.
- Fully functional launch system supporting:
  - GUI / Headless Gazebo
  - Custom world files
  - URDF/Xacro selection
  - Parameter-based robot description injection
- Clean controller YAMLs with realistic constraints.
- Working open/close trajectory commands.

### Changed
- Updated package structure to follow ROS 2 best practices.
- Cleaned CMakeLists & package.xml:
  - Removed unused dependencies.
  - Added missing runtime dependencies.
  - Reorganized install rules.
- Normalized file layout: `launch/`, `config/`, `urdf/`, `worlds/`.
- Improved naming consistency of nodes, controllers, parameters, and paths.

### Fixed
- Controller timing issues caused by no clock source.
- Startup race conditions between robot spawn and controller manager.
- Incorrect controller parameter loading order.
- Missing `/clock` bridge that prevented deterministic simulation time.
- Joint trajectory tolerance warnings via proper constraint definitions.
- Handled fixed-joint skip warnings cleanly.
- Eliminated robot description duplication warnings.

### Known Limitations
- Only simulates the left knuckle joint (mechanically coupled finger behavior not implemented yet).
- No hardware interface for real Robotiq devices — **simulation only**.
- Uses Bullet Featherstone physics; performance may vary.
- No Gazebo control plugin for advanced grasping interactions yet.

### Roadmap
#### Planned for v0.2.x
- Add mechanical mimic joints to fully animate all fingers.
- Provide `ros2_control`-native mimic joint interface.
- Improve grasping behavior via Gazebo contact parameters.
- Provide MoveIt configuration package.
- Add example pick-and-place demo.

#### Planned for v1.x.x
- Add real hardware interface (`serial` / `modbus`).
- Provide multi-gripper support.
- Convert URDF to full SDF with friction tuning.
- Package release on **ROS 2 build farm** (bloom).

---

## Format
Each release follows:

