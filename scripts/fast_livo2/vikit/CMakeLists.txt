cmake_minimum_required(VERSION 3.16)
project(ranger_vikit_common LANGUAGES CXX)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)
if(NOT VIKIT_SOURCE_DIR)
  message(FATAL_ERROR "VIKIT_SOURCE_DIR is required")
endif()
# Only the camera, image and math library used by FAST-LIVO2. No ROS parameter
# server, executables, install scripts, or external framework is built.
add_library(vikit_common SHARED
  ${VIKIT_SOURCE_DIR}/src/pinhole_camera.cpp
  ${VIKIT_SOURCE_DIR}/src/math_utils.cpp
  ${VIKIT_SOURCE_DIR}/src/vision.cpp
  ${VIKIT_SOURCE_DIR}/src/robust_cost.cpp)
target_compile_features(vikit_common PUBLIC cxx_std_17)
target_include_directories(vikit_common PUBLIC ${VIKIT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vikit_common PUBLIC Sophus::Sophus Eigen3::Eigen fmt::fmt ${OpenCV_LIBRARIES})
install(TARGETS vikit_common LIBRARY DESTINATION lib)
install(DIRECTORY ${VIKIT_SOURCE_DIR}/include/vikit DESTINATION include)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/vikit_commonConfig.cmake
  "set(vikit_common_INCLUDE_DIRS \"${CMAKE_INSTALL_PREFIX}/include\")\nset(vikit_common_LIBRARIES \"${CMAKE_INSTALL_PREFIX}/lib/libvikit_common.so\")\n")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/vikit_commonConfig.cmake DESTINATION share/vikit_common/cmake)
