cmake_minimum_required(VERSION 3.8)
project(robot_arm_control)

include(python_setup.cmake OPTIONAL)

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

# Install Python executables
install(PROGRAMS
  robot_arm_control/servo_driver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME servo_driver
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/setup.py)
    execute_process(
        COMMAND python3 setup.py build
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
endif()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install other resources
install(DIRECTORY
  urdf
  meshes
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install hardware plugin
pluginlib_export_plugin_description_file(hardware_interface hardware_plugin/servo_hardware.xml)

ament_package() 