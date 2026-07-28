#include "mavlink_distance_sender.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <stdexcept>
#include <system_error>
#include <utility>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace tfmini_i2c_ros
{

MavlinkDistanceSender::MavlinkDistanceSender(
  std::string serial_device,
  uint8_t system_id,
  uint8_t component_id,
  uint8_t sensor_id,
  uint8_t orientation,
  uint16_t min_distance_cm,
  uint16_t max_distance_cm)
: serial_device_(std::move(serial_device)),
  system_id_(system_id),
  component_id_(component_id),
  sensor_id_(sensor_id),
  orientation_(orientation),
  min_distance_cm_(min_distance_cm),
  max_distance_cm_(max_distance_cm)
{
  open_serial();
}

MavlinkDistanceSender::~MavlinkDistanceSender()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

void MavlinkDistanceSender::send_distance_sensor(uint16_t distance_cm, uint8_t covariance)
{
  const uint16_t clamped_distance = std::clamp(distance_cm, min_distance_cm_, max_distance_cm_);

  std::vector<uint8_t> payload;
  payload.reserve(kDistanceSensorPayloadLen);
  append_uint32(payload, monotonic_ms());
  append_uint16(payload, min_distance_cm_);
  append_uint16(payload, max_distance_cm_);
  append_uint16(payload, clamped_distance);
  payload.push_back(kDistanceSensorLaserType);
  payload.push_back(sensor_id_);
  payload.push_back(orientation_);
  payload.push_back(covariance);

  std::vector<uint8_t> packet;
  packet.reserve(6 + payload.size() + 2);
  packet.push_back(kMavlinkStx);
  packet.push_back(kDistanceSensorPayloadLen);
  packet.push_back(sequence_++);
  packet.push_back(system_id_);
  packet.push_back(component_id_);
  packet.push_back(kDistanceSensorMsgId);
  packet.insert(packet.end(), payload.begin(), payload.end());

  uint16_t crc = 0xffff;
  for (size_t i = 1; i < packet.size(); ++i) {
    crc_accumulate(packet[i], crc);
  }
  crc_accumulate(kDistanceSensorCrcExtra, crc);

  append_uint16(packet, crc);
  write_all(packet.data(), packet.size());
}

void MavlinkDistanceSender::append_uint16(std::vector<uint8_t> & data, uint16_t value)
{
  data.push_back(static_cast<uint8_t>(value & 0xff));
  data.push_back(static_cast<uint8_t>((value >> 8) & 0xff));
}

void MavlinkDistanceSender::append_uint32(std::vector<uint8_t> & data, uint32_t value)
{
  data.push_back(static_cast<uint8_t>(value & 0xff));
  data.push_back(static_cast<uint8_t>((value >> 8) & 0xff));
  data.push_back(static_cast<uint8_t>((value >> 16) & 0xff));
  data.push_back(static_cast<uint8_t>((value >> 24) & 0xff));
}

void MavlinkDistanceSender::crc_accumulate(uint8_t data, uint16_t & crc)
{
  uint8_t tmp = data ^ static_cast<uint8_t>(crc & 0xff);
  tmp ^= static_cast<uint8_t>(tmp << 4);
  crc = static_cast<uint16_t>(
    (crc >> 8) ^ (static_cast<uint16_t>(tmp) << 8) ^
    (static_cast<uint16_t>(tmp) << 3) ^ (static_cast<uint16_t>(tmp) >> 4));
}

void MavlinkDistanceSender::open_serial()
{
  fd_ = ::open(serial_device_.c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::system_error(errno, std::generic_category(), "failed to open " + serial_device_);
  }

  termios options{};
  if (::tcgetattr(fd_, &options) < 0) {
    throw std::system_error(errno, std::generic_category(), "failed to read serial attributes");
  }

  ::cfmakeraw(&options);
  ::cfsetispeed(&options, B921600);
  ::cfsetospeed(&options, B921600);
  options.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  options.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  if (::tcsetattr(fd_, TCSANOW, &options) < 0) {
    throw std::system_error(errno, std::generic_category(), "failed to configure serial port");
  }
}

void MavlinkDistanceSender::write_all(const uint8_t * data, size_t length)
{
  size_t written_total = 0;
  while (written_total < length) {
    const ssize_t written = ::write(fd_, data + written_total, length - written_total);
    if (written < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      throw std::system_error(errno, std::generic_category(), "failed to write MAVLink packet");
    }
    written_total += static_cast<size_t>(written);
  }
}

uint32_t MavlinkDistanceSender::monotonic_ms() const
{
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<uint32_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

}  // namespace tfmini_i2c_ros
