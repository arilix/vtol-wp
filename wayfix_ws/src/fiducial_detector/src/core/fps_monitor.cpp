#include "utils/fps_monitor.h"
namespace fiducial_detector {
FpsMonitor::FpsMonitor(std::size_t window_size)
: window_size_(window_size) {}
void FpsMonitor::tick()
{
  auto now = Clock::now();
  std::lock_guard<std::mutex> lk(mtx_);
  if (!timestamps_.empty()) {
    last_latency_ms_ = std::chrono::duration<double, std::milli>(
      now - timestamps_.back()).count();
  }
  timestamps_.push_back(now);
  while (timestamps_.size() > window_size_) {
    timestamps_.pop_front();
  }
}
float FpsMonitor::getFps() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (timestamps_.size() < 2) return 0.0f;
  double span_s = std::chrono::duration<double>(
    timestamps_.back() - timestamps_.front()).count();
  if (span_s <= 0.0) return 0.0f;
  return static_cast<float>((timestamps_.size() - 1) / span_s);
}
double FpsMonitor::getLatencyMs() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return last_latency_ms_;
}
void FpsMonitor::reset()
{
  std::lock_guard<std::mutex> lk(mtx_);
  timestamps_.clear();
  last_latency_ms_ = 0.0;
}
}
