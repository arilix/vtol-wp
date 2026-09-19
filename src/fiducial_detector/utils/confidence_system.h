#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <string>
#include <chrono>
#include <deque>
namespace fiducial_detector {
struct DetectionConfidence {
  float hamming_confidence    {0.f};
  float reprojection_confidence{0.f};
  float pose_stability        {0.f};
  float contour_confidence    {0.f};
  float decoding_confidence   {0.f};
  float aggregate             {0.f};
  double reprojection_error_px{0.0};
  int    hamming_distance     {0};
  std::string summary() const;
};
struct PoseStabilityState {
  static constexpr std::size_t kWindowSize = 10;
  std::deque<cv::Vec3d> rvec_history;
  std::deque<cv::Vec3d> tvec_history;
  bool is_valid() const { return !tvec_history.empty(); }
};
class ConfidenceCalculator {
public:
  ConfidenceCalculator() = default;
  DetectionConfidence compute(
    const std::vector<cv::Point2f>& corners,
    int   id,
    const cv::Vec3d& rvec,
    const cv::Vec3d& tvec,
    const cv::Mat&   K,
    const cv::Mat&   D,
    int   dict_size,
    int   marker_bits,
    int   hamming_dist,
    double marker_size);
  static float computeContourConfidence(
    const std::vector<cv::Point2f>& corners);
  static float computeReprojectionConfidence(
    const std::vector<cv::Point2f>& detected_corners,
    const cv::Vec3d& rvec,
    const cv::Vec3d& tvec,
    const cv::Mat&   K,
    const cv::Mat&   D,
    double marker_size,
    double* out_error_px = nullptr);
  static float computeHammingConfidence(int hamming_dist, int marker_bits);
  float computePoseStability(int id, const cv::Vec3d& rvec, const cv::Vec3d& tvec);
  void resetStability() { stability_states_.clear(); }
  void resetStability(int id) { stability_states_.erase(id); }
private:
  static constexpr float kWeightHamming       = 0.30f;
  static constexpr float kWeightReproj        = 0.35f;
  static constexpr float kWeightPoseStability = 0.20f;
  static constexpr float kWeightContour       = 0.15f;
  std::map<int, PoseStabilityState> stability_states_;
  static std::vector<cv::Point3f> makeObjectPoints(double marker_size);
};
}
