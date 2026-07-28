cmake_minimum_required(VERSION 3.8)
project(px4)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(px4_msgs    REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(src)

add_executable(px4
  src/main/main.cpp
  src/core/mission_manager.cpp
  src/core/control_module.cpp
  src/core/waypoint_handler.cpp
)

ament_target_dependencies(px4
  rclcpp
  px4_msgs
  sensor_msgs
)

install(TARGETS
  px4
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
