#include "utils/calibration_node.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <condition_variable>
#include <fstream>
namespace fiducial_detector {
CalibrationNode::CalibrationNode(const rclcpp::NodeOptions& opts)
: rclcpp::Node("calibration_node", opts)
{
  declareParameters();
  loadParams();
  pub_status_ = create_publisher<std_msgs::msg::String>("/calibration/status", 10);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
  image_sub_ = image_transport::create_subscription(
    this, camera_topic_,
    std::bind(&CalibrationNode::imageCallback, this, std::placeholders::_1),
    "raw", qos.get_rmw_qos_profile());
  RCLCPP_INFO(get_logger(),"CalibrationNode started. SPACE=capture, c=calibrate, s=save, ESC=quit");
}
CalibrationNode::~CalibrationNode() {}
void CalibrationNode::declareParameters() {
  declare_parameter("camera_topic",   "/camera/image_raw");
  declare_parameter("output_yaml",    "/tmp/camera_calibration.yaml");
  declare_parameter("calib_mode",     "CHARUCO");
  declare_parameter("chess_cols",     9);
  declare_parameter("chess_rows",     6);
  declare_parameter("chess_square",   0.025);
  declare_parameter("charuco_cols",   7);
  declare_parameter("charuco_rows",   5);
  declare_parameter("charuco_sq",     0.035);
  declare_parameter("charuco_mk",     0.0175);
  declare_parameter("dict_name",      "DICT_5X5_50");
  declare_parameter("min_frames",     15);
  declare_parameter("show_window",    true);
}
void CalibrationNode::loadParams() {
  camera_topic_ = get_parameter("camera_topic").as_string();
  output_yaml_  = get_parameter("output_yaml").as_string();
  chess_cols_   = get_parameter("chess_cols").as_int();
  chess_rows_   = get_parameter("chess_rows").as_int();
  chess_square_ = (float)get_parameter("chess_square").as_double();
  charuco_cols_ = get_parameter("charuco_cols").as_int();
  charuco_rows_ = get_parameter("charuco_rows").as_int();
  charuco_sq_   = (float)get_parameter("charuco_sq").as_double();
  charuco_mk_   = (float)get_parameter("charuco_mk").as_double();
  dict_name_    = get_parameter("dict_name").as_string();
  min_frames_   = get_parameter("min_frames").as_int();
  show_window_  = get_parameter("show_window").as_bool();
  std::string mode_s = get_parameter("calib_mode").as_string();
  calib_mode_ = (mode_s == "CHESSBOARD") ? CalibMode::CHESSBOARD : CalibMode::CHARUCO;
}
void CalibrationNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  cv::Mat frame;
  try { frame = cv_bridge::toCvShare(msg,"bgr8")->image.clone(); }
  catch (...) { return; }
  if (frame.empty()) return;
  image_size_ = frame.size();
  cv::Mat gray; cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
  cv::Mat display = frame.clone();
  if (calib_mode_ == CalibMode::CHARUCO) processCharuco(gray,display);
  else processChessboard(gray,display);
  drawStatus(display);
  {
    std::lock_guard<std::mutex> lk(frame_mutex_);
    display_frame_ = display.clone();
    display_ready_ = true;
  }
  display_cv_.notify_one();
}
void CalibrationNode::processCharuco(const cv::Mat& gray, cv::Mat& display) {
  (void)gray; (void)display;
  RCLCPP_WARN_ONCE(get_logger(), "ChArUco mode removed. Use CHESSBOARD mode.");
}
void CalibrationNode::processChessboard(const cv::Mat& gray, cv::Mat& display) {
  cv::Size board_size(chess_cols_, chess_rows_);
  std::vector<cv::Point2f> corners;
  bool found = cv::findChessboardCorners(gray, board_size, corners,
    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
  cv::drawChessboardCorners(display, board_size, corners, found);
  if (!found) return;
  cv::TermCriteria tc(cv::TermCriteria::EPS|cv::TermCriteria::MAX_ITER,30,0.001);
  cv::cornerSubPix(gray,corners,{11,11},{-1,-1},tc);
  if (capture_requested_) {
    capture_requested_ = false;
    std::vector<cv::Point3f> obj;
    for (int r=0;r<chess_rows_;++r)
      for (int c=0;c<chess_cols_;++c)
        obj.emplace_back((float)c*chess_square_,(float)r*chess_square_,0.f);
    chess_corners_all_.push_back(corners);
    chess_obj_points_.push_back(obj);
    RCLCPP_INFO(get_logger(),"Chess frame: %zu",(std::size_t)chess_corners_all_.size());
  }
  if (calibrate_requested_) {
    calibrate_requested_ = false;
    if ((int)chess_corners_all_.size() >= min_frames_) {
      std::vector<cv::Mat> rvecs,tvecs;
      last_result_.camera_matrix = cv::Mat::eye(3,3,CV_64F);
      last_result_.dist_coeffs   = cv::Mat::zeros(5,1,CV_64F);
      last_result_.reprojection_error = cv::calibrateCamera(
        chess_obj_points_, chess_corners_all_, image_size_,
        last_result_.camera_matrix, last_result_.dist_coeffs, rvecs, tvecs);
      last_result_.valid = true;
      last_result_.n_frames_used = (int)chess_corners_all_.size();
      RCLCPP_INFO(get_logger(),"Chess calib done: reproj=%.3f",
        last_result_.reprojection_error);
    }
  }
}
void CalibrationNode::drawStatus(cv::Mat& frame) const {
  int fc = (calib_mode_ == CalibMode::CHARUCO)
    ? 0  // ChArUco mode removed
    : (int)chess_corners_all_.size();
  char buf[128];
  std::snprintf(buf,sizeof(buf),
    "Mode:%s  Frames:%d/%d  [SPACE]=capture [c]=calibrate [s]=save",
    (calib_mode_==CalibMode::CHARUCO)?"ChArUco":"Chess", fc, min_frames_);
  cv::putText(frame,buf,{8,frame.rows-10},cv::FONT_HERSHEY_SIMPLEX,
              0.48,{0,220,220},1,cv::LINE_AA);
  if (last_result_.valid) {
    std::snprintf(buf,sizeof(buf),"CALIBRATED  Reproj=%.3fpx  Frames=%d",
      last_result_.reprojection_error, last_result_.n_frames_used);
    cv::putText(frame,buf,{8,20},cv::FONT_HERSHEY_SIMPLEX,
                0.55,{0,255,0},2,cv::LINE_AA);
  }
}
bool CalibrationNode::displayLoop() {
  if (!show_window_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return rclcpp::ok();
  }
  cv::Mat local_frame;
  {
    std::unique_lock<std::mutex> lk(frame_mutex_);
    display_cv_.wait_for(lk, std::chrono::milliseconds(50),
                         [this] { return display_ready_.load(); });
    if (!display_ready_) return rclcpp::ok();
    local_frame    = display_frame_.clone();
    display_ready_ = false;
  }
  cv::imshow("Calibration", local_frame);
  int key = cv::waitKey(1);
  if (key == 27 || key == 'q') return false;
  if      (key == ' ') capture_requested_   = true;
  else if (key == 'c') calibrate_requested_ = true;
  else if (key == 's') {
    if (!last_result_.valid) {
      RCLCPP_WARN(get_logger(), "No calibration result yet — press [c] first.");
    } else {
      // 1. OpenCV FileStorage (for tools that read it natively)
      {
        cv::FileStorage fs(output_yaml_, cv::FileStorage::WRITE);
        fs << "camera_matrix"           << last_result_.camera_matrix;
        fs << "distortion_coefficients" << last_result_.dist_coeffs;
        fs << "reprojection_error"      << last_result_.reprojection_error;
        fs << "n_frames_used"           << last_result_.n_frames_used;
        fs.release();
        RCLCPP_INFO(get_logger(), "OpenCV YAML saved: %s", output_yaml_.c_str());
      }

      // 2. ROS params format — paste-ready for detector.yaml
      {
        std::string ros_path = output_yaml_;
        auto dot = ros_path.rfind('.');
        if (dot != std::string::npos) ros_path = ros_path.substr(0, dot);
        ros_path += "_ros_params.yaml";

        const double* K = reinterpret_cast<double*>(last_result_.camera_matrix.data);
        const double* D = reinterpret_cast<double*>(last_result_.dist_coeffs.data);
        const int dc    = last_result_.dist_coeffs.cols * last_result_.dist_coeffs.rows;

        std::ofstream f(ros_path);
        if (f.is_open()) {
          f << "# === Paste these values into config/detector.yaml ===\n";
          f << "# Reprojection error: " << last_result_.reprojection_error << " px\n";
          f << "# Frames used: " << last_result_.n_frames_used << "\n";
          f << "camera_matrix:\n";
          for (int i = 0; i < 9; ++i) f << "  - " << K[i] << "\n";
          f << "dist_coeffs: [";
          for (int i = 0; i < dc; ++i) { f << D[i]; if (i < dc-1) f << ", "; }
          f << "]\n";
          f.close();
          RCLCPP_INFO(get_logger(), "ROS params saved: %s", ros_path.c_str());
        }

        // Print to console for immediate copy-paste
        RCLCPP_INFO(get_logger(), "═══ Paste into config/detector.yaml ═══");
        RCLCPP_INFO(get_logger(), "camera_matrix:");
        for (int i = 0; i < 9; ++i) RCLCPP_INFO(get_logger(), "  - %.6f", K[i]);
        RCLCPP_INFO(get_logger(), "dist_coeffs: [%.6f, %.6f, %.6f, %.6f, %.6f]",
          dc>0?D[0]:0.0, dc>1?D[1]:0.0, dc>2?D[2]:0.0,
          dc>3?D[3]:0.0, dc>4?D[4]:0.0);
        RCLCPP_INFO(get_logger(), "# Reproj error: %.4f px  (good if < 1.0)",
          last_result_.reprojection_error);
        RCLCPP_INFO(get_logger(), "════════════════════════════════════════");
      }
    }
  }
  return rclcpp::ok();
}
}
