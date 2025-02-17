cmake_minimum_required(VERSION 3.16)
project(rrbot LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Dependencies required for compilation
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Ensure the required version of ros2_control is available
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  rrbot
  SHARED
  hardware/rrbot.cpp
)
target_compile_features(rrbot PUBLIC cxx_std_17)
target_include_directories(rrbot PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/rrbot>
)
ament_target_dependencies(
  rrbot PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface rrbot.xml)

# Install include headers
install(
  DIRECTORY hardware/include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch, URDF, worlds, meshes, and configuration files
install(
  DIRECTORY launch urdf worlds meshes config
  DESTINATION share/${PROJECT_NAME}
)

# Enable testing and linting if required
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export package
ament_package()
