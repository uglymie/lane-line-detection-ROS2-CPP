cmake_minimum_required(VERSION 3.5)
project(cpp_advanced_lane_lines)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include ${OpenCV_INCLUDE_DIRS})
file(GLOB_RECURSE IMAGE_DIRS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS resource/images/*.png)
file(GLOB_RECURSE SOURCES_DIRS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

# src/main.cpp src/cpp_lane_line_detection.cpp

add_executable(cpp_lane_line_detection ${SOURCES_DIRS})
ament_target_dependencies(cpp_lane_line_detection rclcpp sensor_msgs cv_bridge )
target_include_directories(cpp_lane_line_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(cpp_lane_line_detection ${OpenCV_LIBRARIES})

install(TARGETS cpp_lane_line_detection
  DESTINATION lib/${PROJECT_NAME})
install(FILES launch/lane_line_detection_launch.py
  DESTINATION share/${PROJECT_NAME}/launch)
install(FILES configs/config.yaml
  DESTINATION share/${PROJECT_NAME}/configs)
install(FILES ${IMAGE_DIRS}
  DESTINATION share/${PROJECT_NAME}/images)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(cv_bridge REQUIRED)
  
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
