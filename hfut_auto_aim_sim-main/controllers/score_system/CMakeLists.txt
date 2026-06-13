cmake_minimum_required(VERSION 3.16)
project(score_system_controller)

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
find_package(rclcpp REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(score_system score_system.cpp)

target_include_directories(score_system PRIVATE
  "${WEBOTS_HOME}/include/controller/cpp"
)

target_link_directories(score_system PRIVATE
  "${WEBOTS_HOME}/lib/controller"
)

target_link_libraries(score_system
  CppController
  Controller
)

set_target_properties(score_system PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  BUILD_RPATH "${WEBOTS_HOME}/lib/controller"
)

ament_target_dependencies(score_system
  rclcpp
  rm_interfaces
  std_msgs
)
