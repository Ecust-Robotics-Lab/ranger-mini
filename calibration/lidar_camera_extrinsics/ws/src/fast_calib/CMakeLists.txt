cmake_minimum_required(VERSION 3.8)
project(fast_calib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(OpenCV REQUIRED)

set(DEPENDENCIES
  rclcpp
  sensor_msgs
  geometry_msgs
  rosbag2_cpp
  rosbag2_storage
  cv_bridge
  pcl_conversions
)
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O1)
endif()

add_executable(fast_calib src/main.cpp)
ament_target_dependencies(fast_calib ${DEPENDENCIES})
find_library(OPENCV_ARUCO_LIB opencv_aruco PATHS /usr/local/lib NO_DEFAULT_PATH)
if(NOT OPENCV_ARUCO_LIB)
  find_library(OPENCV_ARUCO_LIB opencv_aruco)
endif()
target_link_libraries(fast_calib ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OPENCV_ARUCO_LIB})

add_executable(multi_fast_calib src/multi_scene.cpp)
ament_target_dependencies(multi_fast_calib ${DEPENDENCIES})
target_link_libraries(multi_fast_calib ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(input_probe src/input_probe.cpp)
ament_target_dependencies(input_probe ${DEPENDENCIES})
target_link_libraries(input_probe ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(lidar_diagnostic src/lidar_diagnostic.cpp)
ament_target_dependencies(lidar_diagnostic ${DEPENDENCIES})
target_link_libraries(lidar_diagnostic ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(offline_checks src/offline_checks.cpp)
ament_target_dependencies(offline_checks ${DEPENDENCIES})
target_link_libraries(offline_checks ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OPENCV_ARUCO_LIB})
install(TARGETS offline_checks fast_calib multi_fast_calib input_probe lidar_diagnostic
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/distance_filter_tool.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
