#include "tfmini_i2c_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace tfmini_i2c_ros
{

TfminiI2cNode::TfminiI2cNode()
: Node("tfmini_i2c_node")
{
  const auto bus_device = declare_parameter<std::string>("bus_device", "/dev/i2c-1");
  const auto frame_id = declare_parameter<std::string>("frame_id", "tfmini_link");
  const auto i2c_read_mode = declare_parameter<std::string>("i2c_read_mode", "command");
  const int i2c_address = declare_parameter<int>("i2c_address", 0x10);
  const double publish_rate = declare_parameter<double>("publish_rate", 20.0);
  const bool mavlink_enabled = declare_parameter<bool>("mavlink_enabled", false);
  const auto mavlink_device = declare_parameter<std::string>("mavlink_device", "/dev/ttyAMA0");
  const int mavlink_system_id = declare_parameter<int>("mavlink_system_id", 1);
  const int mavlink_component_id = declare_parameter<int>("mavlink_component_id", 191);
  const int mavlink_sensor_id = declare_parameter<int>("mavlink_sensor_id", 0);
  const int mavlink_orientation = declare_parameter<int>("mavlink_orientation", 25);
  const int mavlink_covariance = declare_parameter<int>("mavlink_covariance", 0);
  min_range_ = declare_parameter<double>("min_range", 0.03);
  max_range_ = declare_parameter<double>("max_range", 12.0);
  field_of_view_ = declare_parameter<double>("field_of_view", 0.04);
  range_offset_ = declare_parameter<double>("range_offset", 0.0);
  log_rate_ = declare_parameter<double>("log_rate", 1.0);

  if (publish_rate <= 0.0) {
    throw std::runtime_error("publish_rate must be greater than 0");
  }

  frame_id_ = frame_id;
  mavlink_covariance_ = static_cast<uint8_t>(std::clamp(mavlink_covariance, 0, 255));
  driver_ = std::make_unique<TfminiI2c>(bus_device, i2c_address, i2c_read_mode);
  if (mavlink_enabled) {
    mavlink_sender_ = std::make_unique<MavlinkDistanceSender>(
      mavlink_device,
      static_cast<uint8_t>(std::clamp(mavlink_system_id, 1, 255)),
      static_cast<uint8_t>(std::clamp(mavlink_component_id, 1, 255)),
      static_cast<uint8_t>(std::clamp(mavlink_sensor_id, 0, 255)),
      static_cast<uint8_t>(std::clamp(mavlink_orientation, 0, 255)),
      static_cast<uint16_t>(std::clamp(static_cast<int>(std::lround(min_range_ * 100.0)), 1, 65535)),
      static_cast<uint16_t>(std::clamp(static_cast<int>(std::lround(max_range_ * 100.0)), 1, 65535)));
  }
  publisher_ = create_publisher<sensor_msgs::msg::Range>("range", rclcpp::SensorDataQoS());

  const auto period = std::chrono::duration<double>(1.0 / publish_rate);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&TfminiI2cNode::read_and_publish, this));

  RCLCPP_INFO(
    get_logger(), "TF Mini I2C node started on %s at 0x%02x using %s mode",
    bus_device.c_str(), i2c_address, i2c_read_mode.c_str());
  if (mavlink_sender_) {
    RCLCPP_INFO(
      get_logger(), "MAVLink DISTANCE_SENSOR output enabled on %s at 921600 baud",
      mavlink_device.c_str());
  }
}

void TfminiI2cNode::read_and_publish()
{
  try {
    const auto measurement = driver_->read_measurement();
    sensor_msgs::msg::Range msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    msg.field_of_view = static_cast<float>(field_of_view_);
    msg.min_range = static_cast<float>(min_range_);
    msg.max_range = static_cast<float>(max_range_);
    msg.range = (static_cast<float>(measurement.distance_cm) / 100.0F) +
      static_cast<float>(range_offset_);

    publisher_->publish(msg);

    if (mavlink_sender_) {
      const auto distance_cm = static_cast<uint16_t>(
        std::clamp(static_cast<int>(std::lround(static_cast<double>(msg.range) * 100.0)), 1, 65535));
      mavlink_sender_->send_distance_sensor(distance_cm, mavlink_covariance_);
    }

    if (log_rate_ > 0.0) {
      const auto throttle_period_ms = static_cast<int>(1000.0 / log_rate_);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), throttle_period_ms,
        "TF Mini range: raw=%.2f m offset=%.2f m final=%.2f m strength=%u",
        static_cast<double>(measurement.distance_cm) / 100.0,
        range_offset_,
        static_cast<double>(msg.range),
        measurement.strength);
    }
  } catch (const std::exception & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "TF Mini read failed: %s", error.what());
  }
}

}  // namespace tfmini_i2c_ros
