cmake_minimum_required(VERSION 3.16)
project(ros2_image_publisher_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(NOT DEFINED ENV{WEBOTS_HOME})
  set(WEBOTS_HOME /usr/local/webots)
else()
  set(WEBOTS_HOME $ENV{WEBOTS_HOME})
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(ros2_image_publisher ros2_image_publisher.cpp)

target_include_directories(ros2_image_publisher PRIVATE
  "${WEBOTS_HOME}/include/controller/cpp"
)

target_link_directories(ros2_image_publisher PRIVATE
  "${WEBOTS_HOME}/lib/controller"
)

target_link_libraries(ros2_image_publisher
  CppController
  Controller
)

set_target_properties(ros2_image_publisher PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  BUILD_RPATH "${WEBOTS_HOME}/lib/controller"
)

ament_target_dependencies(ros2_image_publisher
  geometry_msgs
  rclcpp
  rm_interfaces
  sensor_msgs
  tf2
  tf2_ros
)
