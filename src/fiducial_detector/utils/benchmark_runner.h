#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <string>
#include <vector>
#include <chrono>

namespace fiducial_detector {

struct BenchmarkResult {
    std::string dict_name     {"DICT_7X7_50"};
    double fps_avg            {0.0};
    double latency_avg_ms     {0.0};
    double latency_min_ms     {0.0};
    double latency_max_ms     {0.0};
    double latency_std_ms     {0.0};
    double cpu_percent        {0.0};
    double memory_mb          {0.0};
    int    markers_found      {0};
    double pose_stability     {0.0};
    std::string toJson()  const;
    std::string toTable() const;
};

class BenchmarkRunner {
public:
    BenchmarkRunner() = default;

    BenchmarkResult run7x7Benchmark(
        const cv::Mat& gray,
        cv::Ptr<cv::aruco::Dictionary> dict,
        const cv::Ptr<cv::aruco::DetectorParameters>& params,
        int n_frames = 30);

    double measureFPS(
        const cv::Mat& gray,
        cv::Ptr<cv::aruco::Dictionary> dict,
        const cv::Ptr<cv::aruco::DetectorParameters>& params,
        double duration_s = 2.0);

    static double getCpuUsagePercent();
    static double getMemoryUsageMB();
    static void   printResult(const BenchmarkResult& result);

    const BenchmarkResult& lastResult() const { return last_result_; }

private:
    BenchmarkResult last_result_;

    static std::vector<double> measureLatencies(
        const cv::Mat& gray,
        cv::Ptr<cv::aruco::Dictionary> dict,
        const cv::Ptr<cv::aruco::DetectorParameters>& params,
        int n_frames);

    static double computeStdDev(const std::vector<double>& values, double mean);
};

} // namespace fiducial_detector
