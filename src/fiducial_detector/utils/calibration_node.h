#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>

namespace fiducial_detector {

struct CalibrationResult {
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    double  reprojection_error{0.0};
    int     n_frames_used{0};
    bool    valid{false};
};

enum class CalibMode { CHESSBOARD, CHARUCO };

class CalibrationNode : public rclcpp::Node {
public:
    explicit CalibrationNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~CalibrationNode() override;
    bool displayLoop();

private:
    void declareParameters();
    void loadParams();
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void processChessboard(const cv::Mat& gray, cv::Mat& display);
    void processCharuco(const cv::Mat& gray, cv::Mat& display);
    void drawStatus(cv::Mat& frame) const;

    image_transport::Subscriber                          image_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  pub_status_;

    std::string camera_topic_;
    std::string output_yaml_;
    CalibMode   calib_mode_{CalibMode::CHESSBOARD};
    int         chess_cols_{9};
    int         chess_rows_{6};
    float       chess_square_{0.025f};
    int         charuco_cols_{7};
    int         charuco_rows_{5};
    float       charuco_sq_{0.035f};
    float       charuco_mk_{0.0175f};
    std::string dict_name_{"DICT_7X7_50"};
    int         min_frames_{15};
    bool        show_window_{true};

    std::vector<std::vector<cv::Point2f>> chess_corners_all_;
    std::vector<std::vector<cv::Point3f>> chess_obj_points_;
    cv::Size                              image_size_;
    CalibrationResult                     last_result_;
    bool                                  capture_requested_{false};
    bool                                  calibrate_requested_{false};

    std::mutex              frame_mutex_;
    cv::Mat                 display_frame_;
    std::atomic<bool>       display_ready_{false};
    std::condition_variable display_cv_;
};

} // namespace fiducial_detector
