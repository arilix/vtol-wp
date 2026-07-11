#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tfmini_i2c_node.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tfmini_i2c_ros::TfminiI2cNode>());
  rclcpp::shutdown();
  return 0;
}
