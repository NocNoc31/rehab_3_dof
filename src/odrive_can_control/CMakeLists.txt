cmake_minimum_required(VERSION 3.5)
project(odrive_can_control)

# Tìm các gói ROS 2 cần thiết
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Thêm các executable
add_executable(odrive_can_node src/odrive_can_node.cpp)
ament_target_dependencies(odrive_can_node rclcpp)

add_executable(odrive_initializer src/odrive_initializer.cpp)
ament_target_dependencies(odrive_initializer rclcpp std_msgs)

add_executable(velocity_controller src/velocity_controller.cpp)
ament_target_dependencies(velocity_controller rclcpp std_msgs)

add_executable(impedance_multi_control src/impedance_multi_control.cpp)
ament_target_dependencies(impedance_multi_control rclcpp std_msgs)

add_executable(read_param src/read_param.cpp)
ament_target_dependencies(read_param rclcpp std_msgs)

add_executable(position_controller src/position_controller.cpp)
ament_target_dependencies(position_controller rclcpp std_msgs trajectory_msgs sensor_msgs)

# Cài đặt các executable
install(TARGETS
  odrive_can_node
  odrive_initializer
  velocity_controller
  impedance_multi_control
  read_param
  position_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Cài đặt thư mục launch
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()