#pragma once





#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <string>


#ifdef FIDUCIAL_USE_REALSENSE
#include <librealsense2/rs.hpp>
#endif

namespace fiducial_detector {

class CaptureNode : public rclcpp::Node {
public:
  explicit CaptureNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());
  ~CaptureNode() override;

private:

  void openWebcam();
  void loopWebcam();

#ifdef FIDUCIAL_USE_REALSENSE
  void openRealSense();
  void loopRealSense();
#endif

  sensor_msgs::msg::Image::SharedPtr matToMsg(
    const cv::Mat& bgr,
    const rclcpp::Time& stamp) const;


  std::string source_;
  int         device_id_;
  std::string device_path_;
  int         width_, height_;
  double      fps_limit_;
  std::string frame_id_;
  std::string output_topic_;


  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr     pub_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;


  cv::VideoCapture cap_;

#ifdef FIDUCIAL_USE_REALSENSE

  rs2::pipeline   rs_pipe_;
  rs2::config     rs_cfg_;
#endif

  std::thread      capture_thread_;
  std::atomic<bool> running_{false};
};

}
