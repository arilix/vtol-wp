#include "utils/pose_estimator.h"
namespace fiducial_detector {
PoseEstimator::PoseEstimator(const cv::Mat& K, const cv::Mat& D, double marker_size)
: K_(K.clone()), D_(D.clone()), marker_size_(marker_size)
{}
std::vector<cv::Point3f> PoseEstimator::makeObjectPoints() const
{
  float h = static_cast<float>(marker_size_) * 0.5f;
  return {{-h,  h, 0.f},
          { h,  h, 0.f},
          { h, -h, 0.f},
          {-h, -h, 0.f}};
}
void PoseEstimator::applySmoothing(int id, cv::Vec3d& rvec, cv::Vec3d& tvec)
{
  auto& s = smooth_state_[id];
  if (!s.initialised) {
    s.rvec = rvec;
    s.tvec = tvec;
    s.initialised = true;
    return;
  }
  for (int i = 0; i < 3; ++i) {
    s.rvec[i] = alpha_ * rvec[i] + (1.0 - alpha_) * s.rvec[i];
    s.tvec[i] = alpha_ * tvec[i] + (1.0 - alpha_) * s.tvec[i];
  }
  rvec = s.rvec;
  tvec = s.tvec;
}
PoseResult PoseEstimator::estimate(int id, const std::vector<cv::Point2f>& corners)
{
  PoseResult result;
  if (corners.size() != 4 || K_.empty()) return result;
  auto obj_pts = makeObjectPoints();
  cv::Vec3d rvec, tvec;
  bool ok = cv::solvePnP(obj_pts, corners, K_, D_, rvec, tvec,
                         false, cv::SOLVEPNP_IPPE_SQUARE);
  if (!ok) return result;
  applySmoothing(id, rvec, tvec);
  result.rvec     = rvec;
  result.tvec     = tvec;
  result.distance = cv::norm(tvec);
  cv::Mat rot_mat;
  cv::Rodrigues(rvec, rot_mat);
  Eigen::Matrix3d erot;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      erot(r, c) = rot_mat.at<double>(r, c);
  result.quaternion = Eigen::Quaterniond(erot).normalized();
  result.valid      = true;
  return result;
}
std::vector<PoseResult> PoseEstimator::estimateBatch(
  const std::vector<int>& ids,
  const std::vector<std::vector<cv::Point2f>>& corners_vec)
{
  std::vector<PoseResult> results;
  results.reserve(ids.size());
  for (std::size_t i = 0; i < ids.size(); ++i) {
    results.push_back(estimate(ids[i], corners_vec[i]));
  }
  return results;
}
}
