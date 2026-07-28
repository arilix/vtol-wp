cmake_minimum_required(VERSION 3.16)
project(general_box_detector_ros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)
find_package(HailoRT REQUIRED)

add_executable(yolo_camera_node src/yolo_camera_node.cpp)
target_compile_options(yolo_camera_node PRIVATE -O3 -Wall -Wextra)
target_link_libraries(yolo_camera_node ${OpenCV_LIBS} HailoRT::libhailort)
ament_target_dependencies(yolo_camera_node rclcpp sensor_msgs std_msgs cv_bridge)

install(TARGETS yolo_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
