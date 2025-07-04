cmake_minimum_required(VERSION 3.8)
project(odrive_joint_state_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(can_odrive_interface REQUIRED)  # Dùng như 1 ROS package
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)

# Add executable node
add_executable(odrive_joint_state_publisher_node
  src/odrive_joint_state_publisher_node.cpp
)

target_include_directories(odrive_joint_state_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(odrive_joint_state_publisher_node PUBLIC cxx_std_17)

# Link ROS dependencies (can_odrive_interface included)
ament_target_dependencies(odrive_joint_state_publisher_node
  rclcpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  can_odrive_interface
  control_msgs
  rclcpp_action
)

# Install the executable
install(TARGETS odrive_joint_state_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

# Testing section
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
