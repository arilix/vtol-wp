#include "utils/benchmark_runner.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <sstream>
#if defined(__linux__)
  #include <sys/resource.h>
  #include <unistd.h>
  #include <fstream>
#endif

namespace fiducial_detector {

std::string BenchmarkResult::toJson() const {
    char buf[512];
    std::snprintf(buf, sizeof(buf),
        "{\"dict\":\"%s\","
        "\"fps_avg\":%.2f,"
        "\"latency_avg_ms\":%.3f,"
        "\"latency_min_ms\":%.3f,"
        "\"latency_max_ms\":%.3f,"
        "\"latency_std_ms\":%.3f,"
        "\"cpu_percent\":%.1f,"
        "\"memory_mb\":%.1f,"
        "\"markers_found\":%d,"
        "\"pose_stability\":%.3f}",
        dict_name.c_str(),
        fps_avg, latency_avg_ms,
        latency_min_ms, latency_max_ms, latency_std_ms,
        cpu_percent, memory_mb,
        markers_found, pose_stability);
    return buf;
}

std::string BenchmarkResult::toTable() const {
    std::ostringstream ss;
    ss << std::string(60, '=') << "\n";
    ss << "  DICT_7X7_50 BENCHMARK RESULT\n";
    ss << std::string(60, '-') << "\n";
    ss << std::left
       << std::setw(24) << "FPS avg"       << fps_avg         << "\n"
       << std::setw(24) << "Latency avg"   << latency_avg_ms  << " ms\n"
       << std::setw(24) << "Latency min"   << latency_min_ms  << " ms\n"
       << std::setw(24) << "Latency max"   << latency_max_ms  << " ms\n"
       << std::setw(24) << "Latency std"   << latency_std_ms  << " ms\n"
       << std::setw(24) << "CPU%"          << cpu_percent      << "\n"
       << std::setw(24) << "Memory"        << memory_mb        << " MB\n"
       << std::setw(24) << "Markers found" << markers_found    << "\n";
    ss << std::string(60, '=') << "\n";
    return ss.str();
}

double BenchmarkRunner::getCpuUsagePercent() {
#if defined(__linux__)
    std::ifstream stat("/proc/self/stat");
    if (!stat.is_open()) return 0.0;
    std::string line;
    std::getline(stat, line);
    std::istringstream ss(line);
    std::string token;
    for (int i = 0; i < 13; ++i) ss >> token;
    long utime = 0, stime = 0;
    ss >> utime >> stime;
    long clk_tck = sysconf(_SC_CLK_TCK);
    if (clk_tck <= 0) clk_tck = 100;
    std::ifstream uptime_f("/proc/uptime");
    double uptime = 1.0;
    uptime_f >> uptime;
    double cpu_s = static_cast<double>(utime + stime) / static_cast<double>(clk_tck);
    return (uptime > 0) ? (cpu_s / uptime * 100.0) : 0.0;
#else
    return 0.0;
#endif
}

double BenchmarkRunner::getMemoryUsageMB() {
#if defined(__linux__)
    std::ifstream status("/proc/self/status");
    if (!status.is_open()) return 0.0;
    std::string line;
    while (std::getline(status, line)) {
        if (line.rfind("VmRSS:", 0) == 0) {
            std::istringstream ss(line.substr(6));
            long kb = 0;
            ss >> kb;
            return static_cast<double>(kb) / 1024.0;
        }
    }
    return 0.0;
#else
    return 0.0;
#endif
}

std::vector<double> BenchmarkRunner::measureLatencies(
    const cv::Mat& gray,
    cv::Ptr<cv::aruco::Dictionary> dict,
    const cv::Ptr<cv::aruco::DetectorParameters>& params,
    int n_frames)
{
    std::vector<double> latencies;
    latencies.reserve(n_frames);
    for (int i = 0; i < n_frames; ++i) {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        auto t0 = std::chrono::steady_clock::now();
        cv::aruco::detectMarkers(gray, dict, corners, ids, params, rejected);
        auto t1 = std::chrono::steady_clock::now();
        latencies.push_back(
            std::chrono::duration<double, std::milli>(t1 - t0).count());
    }
    return latencies;
}

double BenchmarkRunner::computeStdDev(
    const std::vector<double>& values, double mean)
{
    if (values.size() < 2) return 0.0;
    double var = 0.0;
    for (double v : values) var += (v - mean) * (v - mean);
    return std::sqrt(var / static_cast<double>(values.size() - 1));
}

BenchmarkResult BenchmarkRunner::run7x7Benchmark(
    const cv::Mat& gray,
    cv::Ptr<cv::aruco::Dictionary> dict,
    const cv::Ptr<cv::aruco::DetectorParameters>& params,
    int n_frames)
{
    BenchmarkResult r;
    r.dict_name = "DICT_7X7_50";

    auto latencies = measureLatencies(gray, dict, params, n_frames);
    if (latencies.empty()) return r;

    double sum          = std::accumulate(latencies.begin(), latencies.end(), 0.0);
    r.latency_avg_ms    = sum / static_cast<double>(latencies.size());
    r.latency_min_ms    = *std::min_element(latencies.begin(), latencies.end());
    r.latency_max_ms    = *std::max_element(latencies.begin(), latencies.end());
    r.latency_std_ms    = computeStdDev(latencies, r.latency_avg_ms);
    r.fps_avg           = (r.latency_avg_ms > 0.0) ? 1000.0 / r.latency_avg_ms : 0.0;

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(gray, dict, corners, ids, params);
    r.markers_found = static_cast<int>(ids.size());
    r.cpu_percent   = getCpuUsagePercent();
    r.memory_mb     = getMemoryUsageMB();

    last_result_ = r;
    return r;
}

double BenchmarkRunner::measureFPS(
    const cv::Mat& gray,
    cv::Ptr<cv::aruco::Dictionary> dict,
    const cv::Ptr<cv::aruco::DetectorParameters>& params,
    double duration_s)
{
    int frame_count = 0;
    auto t0       = std::chrono::steady_clock::now();
    auto deadline = t0 + std::chrono::duration<double>(duration_s);
    while (std::chrono::steady_clock::now() < deadline) {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(gray, dict, corners, ids, params);
        ++frame_count;
    }
    double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    return (elapsed > 0) ? static_cast<double>(frame_count) / elapsed : 0.0;
}

void BenchmarkRunner::printResult(const BenchmarkResult& result) {
    std::printf("%s\n", result.toTable().c_str());
}

} // namespace fiducial_detector
