#ifndef TFMINI_I2C_ROS__TFMINI_I2C_H_
#define TFMINI_I2C_ROS__TFMINI_I2C_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace tfmini_i2c_ros
{

struct Measurement
{
  uint16_t distance_cm{};
  uint16_t strength{};
  int16_t temperature_raw{};
};

class TfminiI2c
{
public:
  TfminiI2c(std::string bus_device, int address, std::string read_mode);
  ~TfminiI2c();

  TfminiI2c(const TfminiI2c &) = delete;
  TfminiI2c & operator=(const TfminiI2c &) = delete;

  Measurement read_measurement();

private:
  static uint16_t combine_le(uint8_t low, uint8_t high);

  void open_bus();
  void enable_output();
  Measurement parse_measurement(const uint8_t * data, size_t length) const;
  Measurement read_raw_frame();
  Measurement read_register_frame();
  Measurement read_command_frame();
  Measurement try_auto_detect();
  void write_exact(const uint8_t * data, size_t length);
  void read_exact(uint8_t * data, size_t length);
  void read_register_block(uint8_t register_address, uint8_t * data, size_t length);
  void smbus_write_i2c_block_data(uint8_t command, const uint8_t * data, size_t length);
  void smbus_read_i2c_block_data(uint8_t command, uint8_t * data, size_t length);

  std::string bus_device_;
  std::string read_mode_;
  std::optional<std::string> detected_read_mode_;
  int address_;
  int fd_{-1};
};

}  // namespace tfmini_i2c_ros

#endif  // TFMINI_I2C_ROS__TFMINI_I2C_H_
