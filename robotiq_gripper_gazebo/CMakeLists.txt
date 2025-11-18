cmake_minimum_required(VERSION 3.10)
project(robotiq_gripper_gazebo)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Install resource directories
install(
  DIRECTORY launch urdf config worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
