cmake_minimum_required(VERSION 3.8)
project(inspire_rh56_hand_ros2_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Hardware interface library
add_library(${PROJECT_NAME}_hardware_interface SHARED
  src/inspire_rh56_hand_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME}_hardware_interface
  PRIVATE
    include
)

ament_target_dependencies(${PROJECT_NAME}_hardware_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
)

# Install library
install(
  TARGETS ${PROJECT_NAME}_hardware_interface
  DESTINATION lib
)

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include
)

# Install config, launch, urdf, and example files
install(
  DIRECTORY
    config
    launch
    urdf
    examples
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin description
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_hardware_interface)
ament_export_dependencies(
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()