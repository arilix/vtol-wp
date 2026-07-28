#ifndef TFMINI_I2C_ROS__TFMINI_I2C_NODE_H_
#define TFMINI_I2C_ROS__TFMINI_I2C_NODE_H_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "mavlink_distance_sender.h"
#include "tfmini_i2c.h"

namespace tfmini_i2c_ros
{

class TfminiI2cNode : public rclcpp::Node
{
public:
  TfminiI2cNode();

private:
  void read_and_publish();

  std::unique_ptr<TfminiI2c> driver_;
  std::unique_ptr<MavlinkDistanceSender> mavlink_sender_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_;
  double min_range_{0.03};
  double max_range_{12.0};
  double field_of_view_{0.04};
  double range_offset_{0.0};
  double log_rate_{1.0};
  uint8_t mavlink_covariance_{0};
};

}  // namespace tfmini_i2c_ros

#endif  // TFMINI_I2C_ROS__TFMINI_I2C_NODE_H_
