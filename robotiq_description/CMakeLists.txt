cmake_minimum_required(VERSION 3.8)
project(robotiq_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Install the URDF and meshes
install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install the package.xml
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

