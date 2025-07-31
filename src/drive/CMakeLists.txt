cmake_minimum_required(VERSION 3.8)
project(drive)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(uart_node src/uart_node.cpp)
ament_target_dependencies(uart_node rclcpp std_msgs)

add_executable(speed_steer_publisher src/speed_steer_publisher.cpp)
ament_target_dependencies(speed_steer_publisher rclcpp std_msgs)

install(TARGETS
  uart_node
  speed_steer_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
