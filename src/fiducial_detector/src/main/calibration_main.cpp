#include "utils/calibration_node.h"

#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

#include <thread>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fiducial_detector::CalibrationNode>();

  std::thread spin_thread([&node]() {
    rclcpp::spin(node);
  });

  while (node->displayLoop()) {
  }

  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  cv::destroyAllWindows();
  return 0;
}
