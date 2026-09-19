#include "utils/capture_node.h"
#include <cv_bridge/cv_bridge.hpp>
#include <chrono>

namespace fiducial_detector {


CaptureNode::CaptureNode(const rclcpp::NodeOptions& opts)
: rclcpp::Node("capture_node", opts)
{

  declare_parameter<std::string>("source",       "webcam");
  declare_parameter<int>        ("device_id",    6);
  declare_parameter<std::string>("device_path",  "");
  declare_parameter<int>        ("width",        640);
  declare_parameter<int>        ("height",       480);
  declare_parameter<double>     ("fps_limit",    30.0);
  declare_parameter<std::string>("frame_id",     "camera_optical_frame");
  declare_parameter<std::string>("output_topic", "/camera/image_raw");

  source_       = get_parameter("source").as_string();
  device_id_    = get_parameter("device_id").as_int();
  device_path_  = get_parameter("device_path").as_string();
  width_        = get_parameter("width").as_int();
  height_       = get_parameter("height").as_int();
  fps_limit_    = get_parameter("fps_limit").as_double();
  frame_id_     = get_parameter("frame_id").as_string();
  output_topic_ = get_parameter("output_topic").as_string();


  auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
  pub_image_ = create_publisher<sensor_msgs::msg::Image>(output_topic_, qos);

  RCLCPP_INFO(get_logger(), "CaptureNode: source='%s'  topic='%s'",
    source_.c_str(), output_topic_.c_str());


  running_ = true;

#ifdef FIDUCIAL_USE_REALSENSE
  if (source_ == "realsense") {
    openRealSense();
    capture_thread_ = std::thread(&CaptureNode::loopRealSense, this);
    return;
  }
#else
  if (source_ == "realsense") {
    RCLCPP_ERROR(get_logger(),
      "source='realsense' requested but package was compiled without "
      "FIDUCIAL_USE_REALSENSE. Rebuild with -DUSE_REALSENSE=ON or use source='webcam'.");
    rclcpp::shutdown();
    return;
  }
#endif


  openWebcam();
  capture_thread_ = std::thread(&CaptureNode::loopWebcam, this);
}

CaptureNode::~CaptureNode()
{
  running_ = false;
  if (capture_thread_.joinable()) capture_thread_.join();
  if (cap_.isOpened()) cap_.release();
}



void CaptureNode::openWebcam()
{

  if (!device_path_.empty()) {
    RCLCPP_INFO(get_logger(), "Opening webcam: %s", device_path_.c_str());
    cap_.open(device_path_, cv::CAP_V4L2);
  } else {
    RCLCPP_INFO(get_logger(), "Opening webcam: /dev/video%d", device_id_);
    cap_.open(device_id_, cv::CAP_V4L2);
  }

  if (!cap_.isOpened()) {
    RCLCPP_FATAL(get_logger(), "Failed to open camera. "
      "Check device_id or device_path parameter.");
    rclcpp::shutdown();
    return;
  }


  cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width_);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
  cap_.set(cv::CAP_PROP_FPS,          fps_limit_);

  double actual_w = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
  double actual_h = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
  double actual_fps = cap_.get(cv::CAP_PROP_FPS);

  RCLCPP_INFO(get_logger(), "Camera opened: %.0fx%.0f @ %.1f fps",
    actual_w, actual_h, actual_fps);
}

void CaptureNode::loopWebcam()
{
  const double min_interval_s = (fps_limit_ > 0) ? 1.0 / fps_limit_ : 0.0;
  cv::Mat frame;

  while (running_ && rclcpp::ok()) {
    auto t_start = std::chrono::steady_clock::now();

    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Camera read failed — retrying...");

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      cap_.release();
      openWebcam();
      continue;
    }

    auto stamp = now();
    auto msg   = matToMsg(frame, stamp);
    pub_image_->publish(std::move(*msg));


    auto elapsed = std::chrono::steady_clock::now() - t_start;
    double elapsed_s = std::chrono::duration<double>(elapsed).count();
    if (elapsed_s < min_interval_s) {
      std::this_thread::sleep_for(
        std::chrono::duration<double>(min_interval_s - elapsed_s));
    }
  }
}


#ifdef FIDUCIAL_USE_REALSENSE

void CaptureNode::openRealSense()
{
  RCLCPP_INFO(get_logger(), "Opening Intel RealSense color stream %dx%d @ %.0ffps",
    width_, height_, fps_limit_);

  rs_cfg_.enable_stream(RS2_STREAM_COLOR, width_, height_,
    RS2_FORMAT_BGR8, static_cast<int>(fps_limit_));

  try {
    rs_pipe_.start(rs_cfg_);
    RCLCPP_INFO(get_logger(), "RealSense pipeline started");
  } catch (const rs2::error& e) {
    RCLCPP_FATAL(get_logger(), "RealSense error: %s — is the camera connected?",
      e.what());
    rclcpp::shutdown();
  }
}

void CaptureNode::loopRealSense()
{
  while (running_ && rclcpp::ok()) {
    rs2::frameset frames;
    if (!rs_pipe_.try_wait_for_frames(&frames, 1000 /*ms timeout*/)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "RealSense: no frame received within 1s — check USB connection.");
      continue;
    }

    rs2::video_frame color = frames.get_color_frame();
    if (!color) continue;

    int w = color.get_width();
    int h = color.get_height();

    cv::Mat bgr(h, w, CV_8UC3,
      const_cast<void*>(color.get_data()), color.get_stride_in_bytes());

    auto stamp = now();
    auto msg   = matToMsg(bgr, stamp);
    pub_image_->publish(std::move(*msg));
  }

  rs_pipe_.stop();
}

#endif



sensor_msgs::msg::Image::SharedPtr CaptureNode::matToMsg(
  const cv::Mat& bgr,
  const rclcpp::Time& stamp) const
{
  auto msg = std::make_shared<sensor_msgs::msg::Image>();
  msg->header.stamp    = stamp;
  msg->header.frame_id = frame_id_;
  msg->height          = bgr.rows;
  msg->width           = bgr.cols;
  msg->encoding        = "bgr8";
  msg->is_bigendian    = false;
  msg->step            = bgr.step;
  msg->data.assign(bgr.datastart, bgr.dataend);
  return msg;
}

}
