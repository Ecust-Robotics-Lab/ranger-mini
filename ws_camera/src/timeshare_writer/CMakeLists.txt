cmake_minimum_required(VERSION 3.8)
project(timeshare_writer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(timeshare_writer_node src/timeshare_writer_node.cpp)
target_compile_features(timeshare_writer_node PUBLIC cxx_std_17)
ament_target_dependencies(timeshare_writer_node rclcpp sensor_msgs)

install(TARGETS timeshare_writer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
