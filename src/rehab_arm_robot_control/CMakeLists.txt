cmake_minimum_required(VERSION 3.8)
project(rehab_arm_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)


# Add executables and library
add_executable(state_reader src/state_reader.cpp)
add_executable(planning src/planning.cpp)
add_executable(rrt_motion_planning src/rrt_motion_planning.cpp)
add_executable(moveit_odrive_bridge src/moveit_odrive_bridge.cpp)
add_library(can_odrive_interface ../can_odrive_interface/src/odrive_interface.cpp)

# Include directories
target_include_directories(state_reader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(rrt_motion_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(can_odrive_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../can_odrive_interface/include>
  $<INSTALL_INTERFACE:include>)

# Compile features
target_compile_features(state_reader PUBLIC c_std_99 cxx_std_17)
target_compile_features(planning PUBLIC c_std_99 cxx_std_17)
target_compile_features(rrt_motion_planning PUBLIC c_std_99 cxx_std_17)
target_compile_features(can_odrive_interface PUBLIC c_std_99 cxx_std_17)

# Target dependencies
ament_target_dependencies(state_reader
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
)
ament_target_dependencies(planning
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  graph_msgs
  trajectory_msgs
  moveit_visual_tools
)
ament_target_dependencies(rrt_motion_planning
  rclcpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  visualization_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
)
ament_target_dependencies(can_odrive_interface
  rclcpp
  std_msgs
)
ament_target_dependencies(moveit_odrive_bridge
  rclcpp
  trajectory_msgs
  std_msgs
  moveit_msgs
)
target_link_libraries(moveit_odrive_bridge can_odrive_interface)

# Install executables
install(TARGETS
  state_reader
  planning
  rrt_motion_planning
  moveit_odrive_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py"
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()