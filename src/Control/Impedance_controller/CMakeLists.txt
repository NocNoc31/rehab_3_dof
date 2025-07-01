cmake_minimum_required(VERSION 3.8)
project(impedance_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


add_executable(impedance_control_node src/impedance_control_node.cpp)
ament_target_dependencies(impedance_control_node rclcpp std_msgs)
target_include_directories(impedance_control_node PUBLIC ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(impedance_control_node rclcpp std_msgs Eigen3)


install(TARGETS
  impedance_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY yaml
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
