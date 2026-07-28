cmake_minimum_required(VERSION 3.8)
project(tfmini_i2c_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(tfmini_i2c_node
  src/main/main.cpp
  src/source/mavlink_distance_sender.cpp
  src/source/tfmini_i2c.cpp
  src/source/tfmini_i2c_node.cpp
)

target_compile_features(tfmini_i2c_node PUBLIC cxx_std_17)
target_include_directories(tfmini_i2c_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tfmini_i2c_node
  rclcpp
  sensor_msgs
)

install(TARGETS tfmini_i2c_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
