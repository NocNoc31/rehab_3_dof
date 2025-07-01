cmake_minimum_required(VERSION 3.8)
project(data_pub)

# Sử dụng ament_cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Thêm executable
add_executable(data_pub_node src/data_pub_node.cpp)

# Link thư viện
ament_target_dependencies(data_pub_node rclcpp std_msgs)

# Cài đặt executable
install(TARGETS
  data_pub_node
  DESTINATION lib/${PROJECT_NAME}
)

# Gọi ament để build
ament_package()
