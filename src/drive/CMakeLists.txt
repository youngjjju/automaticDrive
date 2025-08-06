cmake_minimum_required(VERSION 3.8)
project(drive)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)  # ✅ OpenCV 추가

include_directories(include ${OpenCV_INCLUDE_DIRS})  # ✅ OpenCV 헤더 포함

add_executable(uart_node src/uart_node.cpp)
ament_target_dependencies(uart_node rclcpp std_msgs)

add_executable(speed_steer_publisher src/speed_steer_publisher.cpp)
ament_target_dependencies(speed_steer_publisher rclcpp std_msgs)

add_executable(drive_node src/drive.cpp)  # ✅ drive_node 추가
ament_target_dependencies(drive_node rclcpp std_msgs)
target_link_libraries(drive_node ${OpenCV_LIBS})  # ✅ OpenCV 링크

install(TARGETS
  uart_node
  speed_steer_publisher
  drive_node  # ✅ 설치 대상에 추가
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
