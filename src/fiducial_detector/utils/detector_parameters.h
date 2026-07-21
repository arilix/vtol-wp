#pragma once
#include <opencv2/aruco.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace fiducial_detector {

class DetectorParametersManager {
public:
    DetectorParametersManager();

    void declareAll(rclcpp::Node* node);
    void bind(rclcpp::Node* node);

    // Applies hardcoded 7x7 optimal profile — no runtime branching.
    void apply7x7Profile();

    cv::Ptr<cv::aruco::DetectorParameters> params() const { return params_; }

private:
    cv::Ptr<cv::aruco::DetectorParameters> params_;
};

} // namespace fiducial_detector
