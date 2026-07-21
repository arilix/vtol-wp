#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include "utils/pose_estimator.h"
#include "utils/marker_decoder.h"
#include <string>
#include <vector>

namespace fiducial_detector {
struct RejectedCandidate;

extern const cv::Scalar CLR_ARUCO;
extern const cv::Scalar CLR_REJECTED;
extern const cv::Scalar CLR_LOCKED;
extern const cv::Scalar CLR_ALIGNED;
extern const cv::Scalar CLR_UNALIGNED;
extern const cv::Scalar CLR_WHITE;
extern const cv::Scalar CLR_BLACK;
extern const cv::Scalar CLR_GRAY;
extern const cv::Scalar CLR_CELL_0;
extern const cv::Scalar CLR_CELL_1;
extern const cv::Scalar CLR_CELL_BORDER;

enum class MarkerType : uint8_t { ARUCO = 0, UNKNOWN = 99 };

class Visualizer {
public:
    explicit Visualizer(int alignment_tolerance = 50);

    void setAlignmentTolerance(int tol) { alignment_tol_ = tol; }
    int  getAlignmentTolerance() const  { return alignment_tol_; }

    void drawUI(cv::Mat& frame, bool any_marker_locked) const;

    void drawDetectedMarkers(
        cv::Mat& frame,
        const std::vector<std::vector<cv::Point2f>>& corners,
        const std::vector<int>& ids,
        MarkerType type) const;

    void drawRejected(
        cv::Mat& frame,
        const std::vector<std::vector<cv::Point2f>>& rejected) const;

    void drawPoseAxis(
        cv::Mat& frame,
        const PoseResult& pose,
        const cv::Mat& K,
        const cv::Mat& D,
        float axis_length = 0.03f) const;

    void drawMarkerInfo(
        cv::Mat& frame,
        cv::Point2f center,
        int id,
        MarkerType type,
        const PoseResult& pose) const;

    void drawAlignment(
        cv::Mat& frame,
        cv::Point2f marker_center,
        const cv::Size& frame_size) const;

    void drawHUD(
        cv::Mat& frame,
        float fps,
        double latency_ms,
        uint64_t frame_count,
        bool cam_ok) const;

    bool        isAligned(cv::Point2f pt, const cv::Size& sz) const;
    bool        isLeft   (cv::Point2f pt, const cv::Size& sz) const;
    bool        isRight  (cv::Point2f pt, const cv::Size& sz) const;
    bool        isUp     (cv::Point2f pt, const cv::Size& sz) const;
    bool        isDown   (cv::Point2f pt, const cv::Size& sz) const;
    std::string alignmentString(cv::Point2f pt, const cv::Size& sz) const;

    void drawRejectedWithReason(
        cv::Mat& frame,
        const std::vector<RejectedCandidate>& rejected) const;

    void drawCornerLabels(
        cv::Mat& frame,
        const std::vector<cv::Point2f>& corners,
        cv::Scalar color = {255, 255, 255}) const;

    void drawOrientationArrow(
        cv::Mat& frame,
        const std::vector<cv::Point2f>& corners,
        cv::Scalar color = {0, 255, 0}) const;

    void drawPoseXYZLabeled(
        cv::Mat& frame,
        const PoseResult& pose,
        const cv::Mat& K,
        const cv::Mat& D,
        float axis_length = 0.05f) const;

    void drawAdvancedHUD(
        cv::Mat& frame,
        float fps,
        double latency_ms,
        uint64_t frame_count,
        bool cam_ok,
        float marker_size_m  = 0.f,
        float confidence_pct = -1.f) const;

    void drawCenterLockBox(cv::Mat& frame, bool any_locked, uint64_t frame_count = 0) const;

    void drawMarkerCells(
        cv::Mat& frame,
        const cv::Mat& norm_img,
        const std::vector<uint8_t>& bits,
        cv::Point pos,
        int size   = 200,
        int n_side = 7,
        int border = 1) const;

    void drawBitMatrix(
        cv::Mat& frame,
        const std::vector<uint8_t>& bits,
        int id,
        int hamming_dist,
        cv::Point pos,
        int cell_size = 20,
        int n_side    = 7) const;

    void drawThresholdOverlay(
        cv::Mat& frame,
        const cv::Mat& threshold,
        double alpha = 0.4) const;

private:
    int alignment_tol_;

    void alphaRect(cv::Mat& frame, cv::Rect rect,
                   cv::Scalar color, double alpha) const;
    void labelText(cv::Mat& frame, const std::string& text,
                   cv::Point origin, cv::Scalar color,
                   double font_scale = 0.55, int thickness = 1) const;
    void drawArrow(cv::Mat& frame, cv::Point from, cv::Point to,
                   cv::Scalar color, int thickness = 2, double frac = 0.15) const;
};

} // namespace fiducial_detector
