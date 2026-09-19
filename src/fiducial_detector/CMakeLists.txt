cmake_minimum_required(VERSION 3.16)
project(fiducial_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2 -march=native)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui objdetect calib3d)
find_package(Eigen3 REQUIRED)

set(COMMON_SRCS
  src/nodes/visualization.cpp
  src/core/pose_estimator.cpp
  src/core/fps_monitor.cpp
  src/core/detector_parameters.cpp
  src/core/dictionary_manager.cpp
  src/core/confidence_system.cpp
  src/core/marker_decoder.cpp
  src/core/benchmark_runner.cpp
  src/core/gate_alignment.cpp
  src/core/marker_generator.cpp
)

add_executable(aruco_node
  src/main/main.cpp
  src/core/aruco.cpp
  ${COMMON_SRCS}
)

add_executable(calibration_node
  src/main/calibration_main.cpp
  src/nodes/calibration_node.cpp
  ${COMMON_SRCS}
)

foreach(tgt aruco_node calibration_node)
  target_include_directories(${tgt} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )
  ament_target_dependencies(${tgt}
    rclcpp rclcpp_components
    std_msgs sensor_msgs geometry_msgs visualization_msgs
    tf2 tf2_geometry_msgs
    cv_bridge image_transport
  )
  target_link_libraries(${tgt} ${OpenCV_LIBS} Eigen3::Eigen)
endforeach()

# ── capture_node ──────────────────────────────────────────────────────────
add_executable(capture_node
  src/main/capture_main.cpp
  src/nodes/capture_node.cpp
)
target_include_directories(capture_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(capture_node rclcpp sensor_msgs cv_bridge)
target_link_libraries(capture_node ${OpenCV_LIBS})

option(USE_REALSENSE "Enable Intel RealSense support in capture_node" OFF)
if(USE_REALSENSE)
  find_package(realsense2 QUIET)
  if(realsense2_FOUND)
    target_compile_definitions(capture_node PRIVATE FIDUCIAL_USE_REALSENSE)
    if(TARGET realsense2::realsense2)
      target_link_libraries(capture_node realsense2::realsense2)
    else()
      target_link_libraries(capture_node ${realsense2_LIBRARY})
      target_include_directories(capture_node PRIVATE ${realsense2_INCLUDE_DIR})
    endif()
    message(STATUS "capture_node: Intel RealSense support ENABLED")
  else()
    message(WARNING
      "USE_REALSENSE=ON but librealsense2 development files were not found. "
      "capture_node source='realsense' will be disabled. "
      "This does not affect realsense.launch.xml, which uses the ROS "
      "realsense2_camera driver.")
  endif()
else()
  message(STATUS
    "capture_node: direct librealsense support OFF. "
    "Use realsense.launch.xml with ros-jazzy-realsense2-camera for Jetson.")
endif()

# ── Raspberry Pi AI HAT / Hailo NPU status ────────────────────────────────
#
# HailoRT is used as an external device runtime. This package currently runs a
# classical OpenCV ArUco pipeline, so preprocessing remains on CPU; the node can
# still probe the Hailo device at runtime when launched with npu:=true.
message(STATUS "Hailo NPU mode: runtime probe via npu:=true; OpenCV preprocessing uses CPU")

# ── dict_test_node — DICT_7X7_50 50/50 validation ─────────────────────────
add_executable(dict_test_node src/main/dict_test_main.cpp)
target_include_directories(dict_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(dict_test_node rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs cv_bridge image_transport)
target_link_libraries(dict_test_node ${OpenCV_LIBS})

install(TARGETS aruco_node calibration_node capture_node dict_test_node
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY utils/ DESTINATION include/utils)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
