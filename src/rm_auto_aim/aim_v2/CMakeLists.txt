cmake_minimum_required(VERSION 3.16)
project(aim_v2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(rm_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED COMPONENTS Runtime)
find_package(Threads REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

set(VISION_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/vision_core)

add_library(aim_v2_core STATIC
  ${VISION_CORE_DIR}/tools/extended_kalman_filter.cpp
  ${VISION_CORE_DIR}/tools/img_tools.cpp
  ${VISION_CORE_DIR}/tools/logger.cpp
  ${VISION_CORE_DIR}/tools/math_tools.cpp
  ${VISION_CORE_DIR}/tools/ransac_sine_fitter.cpp
  ${VISION_CORE_DIR}/tools/trajectory.cpp
  ${VISION_CORE_DIR}/tasks/auto_aim/aimer.cpp
  ${VISION_CORE_DIR}/tasks/auto_aim/armor.cpp
  ${VISION_CORE_DIR}/tasks/auto_aim/classifier.cpp
  ${VISION_CORE_DIR}/tasks/auto_aim/detector.cpp
  ${VISION_CORE_DIR}/tasks/auto_aim/shooter.cpp
  ${VISION_CORE_DIR}/tasks/auto_aim/solver.cpp
  ${VISION_CORE_DIR}/tasks/auto_aim/target.cpp
  ${VISION_CORE_DIR}/tasks/auto_aim/tracker.cpp
  ${VISION_CORE_DIR}/tasks/auto_aim/voter.cpp
  ${VISION_CORE_DIR}/tasks/auto_aim/yolo.cpp
  ${VISION_CORE_DIR}/tasks/auto_aim/yolos/yolo11.cpp
  ${VISION_CORE_DIR}/tasks/auto_aim/yolos/yolov5.cpp
  ${VISION_CORE_DIR}/tasks/auto_aim/yolos/yolov8.cpp
  ${VISION_CORE_DIR}/tasks/auto_aim/multithread/mt_detector.cpp
  ${VISION_CORE_DIR}/tasks/auto_aim/planner/planner.cpp
  ${VISION_CORE_DIR}/tasks/auto_aim/planner/tinympc/admm.cpp
  ${VISION_CORE_DIR}/tasks/auto_aim/planner/tinympc/rho_benchmark.cpp
  ${VISION_CORE_DIR}/tasks/auto_aim/planner/tinympc/tiny_api.cpp
)

target_include_directories(aim_v2_core PUBLIC
  ${VISION_CORE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(aim_v2_core PUBLIC
  Eigen3::Eigen
  ${OpenCV_LIBS}
  openvino::runtime
  fmt::fmt
  spdlog::spdlog
  yaml-cpp
)

add_executable(aim_v2_node src/aim_v2_node.cpp)

target_include_directories(aim_v2_node PRIVATE
  ${VISION_CORE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(aim_v2_node
  ament_index_cpp
  geometry_msgs
  rclcpp
  rm_interfaces
  rm_utils
  sensor_msgs
  std_msgs
  visualization_msgs
  cv_bridge
)

target_link_libraries(aim_v2_node
  aim_v2_core
  Threads::Threads
)

install(TARGETS
  aim_v2_core
  aim_v2_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  assets
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
