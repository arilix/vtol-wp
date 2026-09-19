#include "tfmini_i2c.h"

#include <array>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <utility>

#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace tfmini_i2c_ros
{

TfminiI2c::TfminiI2c(std::string bus_device, int address, std::string read_mode)
: bus_device_(std::move(bus_device)), read_mode_(std::move(read_mode)), address_(address)
{
  open_bus();
  enable_output();
}

TfminiI2c::~TfminiI2c()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

Measurement TfminiI2c::read_measurement()
{
  if (read_mode_ == "raw") {
    return read_raw_frame();
  }

  if (read_mode_ == "register") {
    return read_register_frame();
  }

  if (read_mode_ == "command") {
    return read_command_frame();
  }

  if (read_mode_ == "auto") {
    return try_auto_detect();
  }

  throw std::runtime_error("unknown i2c_read_mode: " + read_mode_);
}

uint16_t TfminiI2c::combine_le(uint8_t low, uint8_t high)
{
  return static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
}

void TfminiI2c::open_bus()
{
  fd_ = ::open(bus_device_.c_str(), O_RDWR);
  if (fd_ < 0) {
    throw std::system_error(errno, std::generic_category(), "failed to open " + bus_device_);
  }

  if (::ioctl(fd_, I2C_SLAVE, address_) < 0) {
    throw std::system_error(errno, std::generic_category(), "failed to select I2C address");
  }
}

void TfminiI2c::enable_output()
{
  constexpr std::array<uint8_t, 5> kEnableOutputCommand{0x5a, 0x05, 0x07, 0x01, 0x67};
  smbus_write_i2c_block_data(
    kEnableOutputCommand[0], kEnableOutputCommand.data() + 1, kEnableOutputCommand.size() - 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

Measurement TfminiI2c::parse_measurement(const uint8_t * data, size_t length) const
{
  if (length < 6) {
    throw std::runtime_error("TF Mini frame too short");
  }

  if (data[0] == 0x59 && data[1] == 0x59) {
    if (length < 9) {
      throw std::runtime_error("TF Mini serial frame too short");
    }

    uint8_t checksum = 0;
    for (size_t i = 0; i < 8; ++i) {
      checksum = static_cast<uint8_t>(checksum + data[i]);
    }
    if (checksum != data[8]) {
      throw std::runtime_error("invalid TF Mini frame checksum");
    }

    Measurement measurement;
    measurement.distance_cm = combine_le(data[2], data[3]);
    measurement.strength = combine_le(data[4], data[5]);
    measurement.temperature_raw = static_cast<int16_t>(combine_le(data[6], data[7]));
    return measurement;
  }

  Measurement measurement;
  measurement.distance_cm = combine_le(data[0], data[1]);
  measurement.strength = combine_le(data[2], data[3]);
  measurement.temperature_raw = static_cast<int16_t>(combine_le(data[4], data[5]));
  if (measurement.distance_cm == 0) {
    throw std::runtime_error("invalid zero TF Mini distance");
  }
  return measurement;
}

Measurement TfminiI2c::read_raw_frame()
{
  std::array<uint8_t, 9> data{};
  read_exact(data.data(), data.size());
  return parse_measurement(data.data(), data.size());
}

Measurement TfminiI2c::read_register_frame()
{
  constexpr uint8_t kDataRegister = 0x00;
  std::array<uint8_t, 9> data{};
  read_register_block(kDataRegister, data.data(), data.size());
  return parse_measurement(data.data(), data.size());
}

Measurement TfminiI2c::read_command_frame()
{
  constexpr std::array<uint8_t, 5> kReadCommand{0x5a, 0x05, 0x00, 0x01, 0x60};
  std::array<uint8_t, 9> data{};

  smbus_write_i2c_block_data(kReadCommand[0], kReadCommand.data() + 1, kReadCommand.size() - 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  smbus_read_i2c_block_data(0x00, data.data(), data.size());
  return parse_measurement(data.data(), data.size());
}

Measurement TfminiI2c::try_auto_detect()
{
  if (detected_read_mode_) {
    const std::string active_mode = *detected_read_mode_;
    if (active_mode == "raw") {
      return read_raw_frame();
    }
    if (active_mode == "register") {
      return read_register_frame();
    }
    if (active_mode == "command") {
      return read_command_frame();
    }
  }

  try {
    Measurement measurement = read_raw_frame();
    detected_read_mode_ = "raw";
    return measurement;
  } catch (const std::exception &) {
  }

  try {
    Measurement measurement = read_register_frame();
    detected_read_mode_ = "register";
    return measurement;
  } catch (const std::exception &) {
  }

  Measurement measurement = read_command_frame();
  detected_read_mode_ = "command";
  return measurement;
}

void TfminiI2c::write_exact(const uint8_t * data, size_t length)
{
  const ssize_t written = ::write(fd_, data, length);
  if (written != static_cast<ssize_t>(length)) {
    throw std::system_error(errno, std::generic_category(), "failed to write TF Mini command");
  }
}

void TfminiI2c::read_exact(uint8_t * data, size_t length)
{
  const ssize_t bytes_read = ::read(fd_, data, length);
  if (bytes_read != static_cast<ssize_t>(length)) {
    throw std::system_error(errno, std::generic_category(), "failed to read TF Mini frame");
  }
}

void TfminiI2c::read_register_block(uint8_t register_address, uint8_t * data, size_t length)
{
  i2c_msg messages[2]{};
  messages[0].addr = static_cast<uint16_t>(address_);
  messages[0].flags = 0;
  messages[0].len = 1;
  messages[0].buf = &register_address;

  messages[1].addr = static_cast<uint16_t>(address_);
  messages[1].flags = I2C_M_RD;
  messages[1].len = static_cast<uint16_t>(length);
  messages[1].buf = data;

  i2c_rdwr_ioctl_data transaction{};
  transaction.msgs = messages;
  transaction.nmsgs = 2;

  if (::ioctl(fd_, I2C_RDWR, &transaction) < 0) {
    throw std::system_error(errno, std::generic_category(), "failed to read TF Mini I2C register");
  }
}

void TfminiI2c::smbus_write_i2c_block_data(uint8_t command, const uint8_t * data, size_t length)
{
  if (length > I2C_SMBUS_BLOCK_MAX) {
    throw std::runtime_error("SMBus write block too long");
  }

  i2c_smbus_data smbus_data{};
  smbus_data.block[0] = static_cast<uint8_t>(length);
  std::memcpy(&smbus_data.block[1], data, length);

  i2c_smbus_ioctl_data ioctl_data{};
  ioctl_data.read_write = I2C_SMBUS_WRITE;
  ioctl_data.command = command;
  ioctl_data.size = I2C_SMBUS_I2C_BLOCK_DATA;
  ioctl_data.data = &smbus_data;

  if (::ioctl(fd_, I2C_SMBUS, &ioctl_data) < 0) {
    throw std::system_error(errno, std::generic_category(), "failed to write TF Mini SMBus command");
  }
}

void TfminiI2c::smbus_read_i2c_block_data(uint8_t command, uint8_t * data, size_t length)
{
  if (length > I2C_SMBUS_BLOCK_MAX) {
    throw std::runtime_error("SMBus read block too long");
  }

  i2c_smbus_data smbus_data{};
  smbus_data.block[0] = static_cast<uint8_t>(length);

  i2c_smbus_ioctl_data ioctl_data{};
  ioctl_data.read_write = I2C_SMBUS_READ;
  ioctl_data.command = command;
  ioctl_data.size = I2C_SMBUS_I2C_BLOCK_DATA;
  ioctl_data.data = &smbus_data;

  if (::ioctl(fd_, I2C_SMBUS, &ioctl_data) < 0) {
    throw std::system_error(errno, std::generic_category(), "failed to read TF Mini SMBus block");
  }

  std::memcpy(data, &smbus_data.block[1], length);
}

}  // namespace tfmini_i2c_ros
