#ifndef TFMINI_I2C_ROS__MAVLINK_DISTANCE_SENDER_H_
#define TFMINI_I2C_ROS__MAVLINK_DISTANCE_SENDER_H_

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

#include <termios.h>

namespace tfmini_i2c_ros
{

class MavlinkDistanceSender
{
public:
  MavlinkDistanceSender(
    std::string serial_device,
    uint8_t system_id,
    uint8_t component_id,
    uint8_t sensor_id,
    uint8_t orientation,
    uint16_t min_distance_cm,
    uint16_t max_distance_cm);

  ~MavlinkDistanceSender();

  MavlinkDistanceSender(const MavlinkDistanceSender &) = delete;
  MavlinkDistanceSender & operator=(const MavlinkDistanceSender &) = delete;

  void send_distance_sensor(uint16_t distance_cm, uint8_t covariance);

private:
  static constexpr uint8_t kMavlinkStx = 0xfe;
  static constexpr uint8_t kDistanceSensorMsgId = 132;
  static constexpr uint8_t kDistanceSensorPayloadLen = 14;
  static constexpr uint8_t kDistanceSensorCrcExtra = 85;
  static constexpr uint8_t kDistanceSensorLaserType = 0;

  static void append_uint16(std::vector<uint8_t> & data, uint16_t value);
  static void append_uint32(std::vector<uint8_t> & data, uint32_t value);
  static void crc_accumulate(uint8_t data, uint16_t & crc);

  void open_serial();
  void write_all(const uint8_t * data, size_t length);
  uint32_t monotonic_ms() const;

  std::string serial_device_;
  uint8_t system_id_;
  uint8_t component_id_;
  uint8_t sensor_id_;
  uint8_t orientation_;
  uint16_t min_distance_cm_;
  uint16_t max_distance_cm_;
  uint8_t sequence_{0};
  int fd_{-1};
};

}  // namespace tfmini_i2c_ros

#endif  // TFMINI_I2C_ROS__MAVLINK_DISTANCE_SENDER_H_
