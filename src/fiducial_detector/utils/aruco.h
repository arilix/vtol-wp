#pragma once
#include "utils/fps_monitor.h"
#include <opencv2/aruco.hpp>
#include "utils/pose_estimator.h"
#include "utils/detector_parameters.h"
#include "utils/dictionary_manager.h"
#include "utils/visualization.h"
#include "utils/confidence_system.h"
#include "utils/benchmark_runner.h"
#include "utils/gate_alignment.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "utils/marker_decoder.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace fiducial_detector {
class BenchmarkRunner;
class Visualizer;
enum class MarkerType : uint8_t;

struct DetectedMarker {
    int                      id{-1};
    MarkerType               type{MarkerType::ARUCO};
    std::vector<cv::Point2f> corners;
    cv::Point2f              center;
    PoseResult               pose;
    DetectionConfidence      confidence;
};

struct DetectionResult {
    std::vector<DetectedMarker>             markers;
    std::vector<std::vector<cv::Point2f>>   rejected;
    cv::Size                                frame_size;
};

class FiducialDetector : public rclcpp::Node {
public:
    explicit FiducialDetector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~FiducialDetector() override;
    bool displayLoop();

private:
    void declareParameters();
    void loadRosParams();
    void loadIntrinsics();
    void initDetectors();
    void initSubscriber();
    void initPublishers();

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void fpsTimerCallback();
    void watchdogCallback();
    void visionSourceCallback(const std_msgs::msg::String::SharedPtr msg);

    DetectionResult runDetection(const cv::Mat& frame);
    void detectAruco(const cv::Mat& gray, DetectionResult& result);
    void estimatePoses(DetectionResult& result);
    void computeConfidence(DetectionResult& result);
    void stabilizeDetections(DetectionResult& result);

    void publishAll(const DetectionResult& result,
                    const cv::Mat& annotated,
                    const GateError& gate_err,
                    const rclcpp::Time& stamp);
    void publishRvizMarkers(const DetectionResult& result,
                            const std::string& frame_id,
                            const rclcpp::Time& stamp);
    void logDetectedMarkers(const DetectionResult& result,
                            const GateError& gate_err) const;
    void reconnectCamera();
    void initInternalCapture();
    void loopInternalCapture();

    cv::Point2f computeCenter(const std::vector<cv::Point2f>& corners) const;
    cv::Mat     preprocessFrame(const cv::Mat& gray);
    void        enqueueDisplay(const cv::Mat& frame);

    image_transport::Subscriber                                    image_sub_;
    // Skip-inferensi (bukan start/stop proses) berdasar sinyal dari
    // mission_manager (px4) di /mission/vision_source_active — lihat
    // komentar di visionSourceCallback()/imageCallback().
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr         vision_source_sub_;
    std::string active_vision_source_{};
    rclcpp::Time last_active_signal_time_{};
    bool         active_signal_seen_{false};
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pub_pose_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_rviz_markers_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr  pub_marker_centers_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr          pub_debug_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_alignment_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr           pub_fps_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_rejected_;
    rclcpp::TimerBase::SharedPtr                                   fps_timer_;
    rclcpp::TimerBase::SharedPtr                                   watchdog_timer_;
    rclcpp::TimerBase::SharedPtr                                   capture_start_timer_;

    double      marker_size_{0.05};
    std::string camera_topic_{"/camera/image_raw"};
    std::string output_frame_id_{};
    int         alignment_tolerance_{50};
    double      smoothing_alpha_{0.4};
    double      min_detection_confidence_{0.70};
    int         max_missed_frames_{5};
    int         alignment_stable_frames_{10};

    bool   show_window_{false};
    bool   show_rejected_{true};
    bool   show_corner_labels_{true};
    bool   show_orientation_arrow_{true};
    bool   show_confidence_{true};
    bool   publish_debug_image_{true};
    bool   enable_clahe_{true};
    double clahe_clip_{2.0};
    bool   enable_sharpen_{false};
    bool   enable_blur_{false};

    struct CameraIntrinsics { cv::Mat K, D; bool valid{false}; } intrinsics_;

    cv::Ptr<cv::aruco::Dictionary>          aruco_dict_;
    std::unique_ptr<DetectorParametersManager> det_params_mgr_;
    std::unique_ptr<PoseEstimator>          pose_estimator_;
    std::unique_ptr<DictionaryManager>      dict_manager_;
    std::unique_ptr<Visualizer>             visualizer_;
    std::unique_ptr<ConfidenceCalculator>   confidence_calc_;
    std::unique_ptr<BenchmarkRunner>        benchmark_runner_;
    std::unique_ptr<GateAlignmentEngine>    gate_alignment_;
    cv::Ptr<cv::CLAHE>                      clahe_;
    bool                                    use_npu_enabled_{false};
    bool                                    npu_available_{false};
    FpsMonitor                              fps_monitor_;

    // Stabilisasi centroid untuk marker tergabung:
    // EMA (Exponential Moving Average) dari centroid semua marker
    cv::Point2f smoothed_center_{-1.f, -1.f};   // -1 = belum diinisialisasi
    bool        center_initialized_{false};
    GateError   last_gate_err_{};                // Simpan error terakhir saat marker hilang sementara

    struct MarkerTrack {
        DetectedMarker marker;
        int missed_frames{0};
    };
    std::unordered_map<std::string, MarkerTrack> marker_tracks_;

    std::atomic<bool> cam_connected_{false};
    std::atomic<bool> reconnect_pending_{false};
    rclcpp::Time      last_frame_time_;
    uint64_t          frame_count_{0};
    mutable std::mutex callback_mutex_;

    std::atomic<bool> show_cells_window_{false};
    std::atomic<bool> show_thresh_window_{false};
    std::atomic<bool> show_contour_window_{false};
    std::atomic<bool> show_rejected_window_{false};
    std::mutex        debug_mutex_;
    MarkerDebugImages last_debug_;

    bool        capture_internal_{false};
    int         capture_device_id_{0};
    std::string capture_device_path_{};
    int         capture_width_{640};
    int         capture_height_{480};
    double      capture_fps_{30.0};
    cv::VideoCapture cap_;
    std::thread      capture_thread_;
    std::atomic<bool> capture_running_{false};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_internal_cam_;

    cv::Mat                 display_frame_;
    std::mutex              display_mutex_;
    std::condition_variable display_cv_;
    std::atomic<bool>       display_ready_{false};
};

} // namespace fiducial_detector
