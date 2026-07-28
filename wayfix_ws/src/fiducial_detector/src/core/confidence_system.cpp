#include "utils/confidence_system.h"
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <numeric>
#include <sstream>
#include <iomanip>
namespace fiducial_detector {
std::string DetectionConfidence::summary() const {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2);
  ss << "Conf=" << (aggregate * 100.f) << "% "
     << "[Hmm=" << (hamming_confidence * 100.f) << "% "
     << "Rpr=" << (reprojection_confidence * 100.f) << "% "
     << "Stab=" << (pose_stability * 100.f) << "% "
     << "Cnt=" << (contour_confidence * 100.f) << "%]"
     << " Hamming=" << hamming_distance
     << " ReprErr=" << reprojection_error_px << "px";
  return ss.str();
}
std::vector<cv::Point3f> ConfidenceCalculator::makeObjectPoints(double marker_size) {
  float h = static_cast<float>(marker_size) * 0.5f;
  return {{-h,  h, 0.f}, { h,  h, 0.f}, { h, -h, 0.f}, {-h, -h, 0.f}};
}
float ConfidenceCalculator::computeHammingConfidence(int hamming_dist, int marker_bits) {
  if (hamming_dist < 0) return 0.f;
  if (hamming_dist == 0) return 1.f;
  int max_dist = std::max(1, marker_bits * marker_bits / 4);
  float ratio  = static_cast<float>(hamming_dist) / static_cast<float>(max_dist);
  return std::max(0.f, std::exp(-2.5f * ratio));
}
float ConfidenceCalculator::computeReprojectionConfidence(
  const std::vector<cv::Point2f>& detected,
  const cv::Vec3d& rvec,
  const cv::Vec3d& tvec,
  const cv::Mat&   K,
  const cv::Mat&   D,
  double marker_size,
  double* out_error_px)
{
  if (detected.size() != 4 || K.empty()) {
    if (out_error_px) *out_error_px = -1.0;
    return 0.f;
  }
  auto obj_pts = makeObjectPoints(marker_size);
  std::vector<cv::Point2f> projected;
  cv::projectPoints(obj_pts, rvec, tvec, K, D, projected);
  double total_err = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    double dx = projected[i].x - detected[i].x;
    double dy = projected[i].y - detected[i].y;
    total_err += std::sqrt(dx*dx + dy*dy);
  }
  double mean_err = total_err / 4.0;
  if (out_error_px) *out_error_px = mean_err;
  return static_cast<float>(1.0 / (1.0 + mean_err * mean_err * 0.25));
}
float ConfidenceCalculator::computeContourConfidence(
  const std::vector<cv::Point2f>& corners)
{
  if (corners.size() != 4) return 0.f;
  double area = 0.0;
  for (int i = 0; i < 4; ++i) {
    int j = (i + 1) % 4;
    area += corners[i].x * corners[j].y;
    area -= corners[j].x * corners[i].y;
  }
  area = std::abs(area) * 0.5;
  if (area < 100.0) return 0.f;
  bool all_positive = true, all_negative = true;
  for (int i = 0; i < 4; ++i) {
    int j = (i + 1) % 4, k = (i + 2) % 4;
    float dx1 = corners[j].x - corners[i].x;
    float dy1 = corners[j].y - corners[i].y;
    float dx2 = corners[k].x - corners[j].x;
    float dy2 = corners[k].y - corners[j].y;
    float cross = dx1 * dy2 - dy1 * dx2;
    if (cross < 0) all_positive = false;
    if (cross > 0) all_negative = false;
  }
  bool is_convex = all_positive || all_negative;
  if (!is_convex) return 0.1f;
  std::array<double, 4> sides;
  for (int i = 0; i < 4; ++i) {
    int j = (i + 1) % 4;
    double dx = corners[j].x - corners[i].x;
    double dy = corners[j].y - corners[i].y;
    sides[i] = std::sqrt(dx*dx + dy*dy);
  }
  double ratio1 = (sides[0] > 0 && sides[2] > 0) ?
    std::min(sides[0]/sides[2], sides[2]/sides[0]) : 0.0;
  double ratio2 = (sides[1] > 0 && sides[3] > 0) ?
    std::min(sides[1]/sides[3], sides[3]/sides[1]) : 0.0;
  double regularity = (ratio1 + ratio2) * 0.5;
  return static_cast<float>(0.7 * regularity + 0.3 * (is_convex ? 1.0 : 0.0));
}
float ConfidenceCalculator::computePoseStability(
  int id,
  const cv::Vec3d& rvec,
  const cv::Vec3d& tvec)
{
  auto& state = stability_states_[id];
  state.tvec_history.push_back(tvec);
  state.rvec_history.push_back(rvec);
  while (state.tvec_history.size() > PoseStabilityState::kWindowSize) {
    state.tvec_history.pop_front();
    state.rvec_history.pop_front();
  }
  if (state.tvec_history.size() < 3) return 0.5f;
  cv::Vec3d mean_t(0, 0, 0);
  for (const auto& t : state.tvec_history) mean_t += t;
  mean_t *= (1.0 / static_cast<double>(state.tvec_history.size()));
  double var_t = 0.0;
  for (const auto& t : state.tvec_history) {
    cv::Vec3d diff = t - mean_t;
    var_t += diff.dot(diff);
  }
  var_t /= static_cast<double>(state.tvec_history.size());
  double std_t = std::sqrt(var_t);
  float stability = static_cast<float>(
    std::max(0.0, 1.0 - std_t / 0.010));
  return stability;
}
DetectionConfidence ConfidenceCalculator::compute(
  const std::vector<cv::Point2f>& corners,
  int   id,
  const cv::Vec3d& rvec,
  const cv::Vec3d& tvec,
  const cv::Mat&   K,
  const cv::Mat&   D,
  int   dict_size,
  int   marker_bits,
  int   hamming_dist,
  double marker_size)
{
  (void)dict_size;
  DetectionConfidence conf;
  conf.hamming_distance = hamming_dist;
  conf.hamming_confidence = computeHammingConfidence(hamming_dist, marker_bits);
  if (!K.empty() && (rvec.val[0] != 0.0 || tvec.val[0] != 0.0)) {
    conf.reprojection_confidence = computeReprojectionConfidence(
      corners, rvec, tvec, K, D, marker_size, &conf.reprojection_error_px);
  } else {
    conf.reprojection_confidence = 0.5f;
    conf.reprojection_error_px   = -1.0;
  }
  conf.pose_stability = computePoseStability(id, rvec, tvec);
  conf.contour_confidence = computeContourConfidence(corners);
  conf.decoding_confidence = 0.7f * conf.hamming_confidence
                           + 0.3f * conf.contour_confidence;
  conf.aggregate =
    kWeightHamming       * conf.hamming_confidence
  + kWeightReproj        * conf.reprojection_confidence
  + kWeightPoseStability * conf.pose_stability
  + kWeightContour       * conf.contour_confidence;
  return conf;
}
}
