cmake_minimum_required(VERSION 3.8)
project(robot_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Build C++ library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/servo_hardware.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
)

# Install C++ targets
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts with executable permissions
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/servo_driver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME servo_driver
)

# Install directories
install(
  DIRECTORY
    config
    launch
    urdf
    meshes
    hardware_plugin
  DESTINATION share/${PROJECT_NAME}
)

# Export hardware plugin
pluginlib_export_plugin_description_file(
  hardware_interface
  hardware_plugin/servo_hardware.xml
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()
