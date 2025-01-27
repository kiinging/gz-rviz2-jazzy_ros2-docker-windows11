cmake_minimum_required(VERSION 3.8)
project(generic_robot_package)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)

# Add executable
add_executable(generic_robot_node src/generic_robot_node.cpp)

# Specify include directories for the executable
target_include_directories(generic_robot_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies with the executable
ament_target_dependencies(generic_robot_node
  controller_manager
  ros2_control
  ros2_controllers
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/config
)

# Install executable
install(TARGETS generic_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

# Declare this project as an ament package
ament_package()
