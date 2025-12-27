cmake_minimum_required(VERSION 3.5)
project(my_bot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)

# Build hardware interface library
add_library(
  ${PROJECT_NAME}
  SHARED
  hardware/diffbot_system.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  hardware/include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export plugin
pluginlib_export_plugin_description_file(hardware_interface ${PROJECT_NAME}.xml)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# Install headers
install(
  DIRECTORY hardware/include/
  DESTINATION include
)

# Install Python scripts
install(PROGRAMS
  scripts/robot_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install other directories
install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Remove this line - we don't need it for just scripts!
# ament_python_install_package(${PROJECT_NAME})

ament_package()