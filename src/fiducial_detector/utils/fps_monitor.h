#pragma once
#include <chrono>
#include <deque>
#include <numeric>
#include <mutex>
#include <atomic>
namespace fiducial_detector {
class FpsMonitor {
public:
  explicit FpsMonitor(std::size_t window_size = 60);
  void tick();
  float getFps() const;
  double getLatencyMs() const;
  void reset();
private:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  std::size_t window_size_;
  mutable std::mutex mtx_;
  std::deque<TimePoint> timestamps_;
  double last_latency_ms_{0.0};
};
}
