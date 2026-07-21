#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <map>
#include <string>
namespace fiducial_detector {
struct PoseResult {
  cv::Vec3d rvec;
  cv::Vec3d tvec;
  Eigen::Quaterniond quaternion;
  double distance{0.0};
  bool valid{false};
};
class PoseEstimator {
public:
  PoseEstimator(const cv::Mat& camera_matrix,
                const cv::Mat& dist_coeffs,
                double marker_size);
  PoseResult estimate(int id, const std::vector<cv::Point2f>& corners);
  std::vector<PoseResult> estimateBatch(
    const std::vector<int>& ids,
    const std::vector<std::vector<cv::Point2f>>& corners_vec);
  void setSmoothingAlpha(double alpha) { alpha_ = alpha; }
  double getSmoothingAlpha() const { return alpha_; }
  const cv::Mat& cameraMatrix() const { return K_; }
  const cv::Mat& distCoeffs()   const { return D_; }
private:
  void applySmoothing(int id, cv::Vec3d& rvec, cv::Vec3d& tvec);
  cv::Mat K_, D_;
  double  marker_size_;
  double  alpha_{0.4};
  struct SmoothState {
    cv::Vec3d rvec{0,0,0};
    cv::Vec3d tvec{0,0,0};
    bool initialised{false};
  };
  std::map<int, SmoothState> smooth_state_;
  std::vector<cv::Point3f> makeObjectPoints() const;
};
}
