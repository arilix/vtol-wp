#pragma once
#include <opencv2/core.hpp>
#include <cstdint>
#include <string>

namespace fiducial_detector {

enum class AlignmentState : uint8_t {
    SEARCH,        // no marker visible
    DETECTED,      // first frame with marker
    TRACKING,      // stable for N consecutive frames
    CENTERING,     // error outside tolerance, UAV correcting
    ALIGNED,       // |error_x| <= tol AND |error_y| <= tol
    GATE_READY,    // aligned for M consecutive frames
    PASS_THROUGH,  // cleared for gate traversal
};

struct GateError {
    float          error_x{0.f};
    float          error_y{0.f};
    float          distance{0.f};
    AlignmentState state{AlignmentState::SEARCH};
    int            marker_id{-1};

    std::string stateName() const;
    std::string toJson()    const;
};

class GateAlignmentEngine {
public:
    explicit GateAlignmentEngine(int tolerance_px   = 50,
                                 int stable_frames  = 3,
                                 int aligned_frames = 10,
                                 int max_missed     = 5);

    GateError update(cv::Point2f marker_center,
                     const cv::Size& frame_size,
                     float distance,
                     int   marker_id);

    void reset();
    AlignmentState state() const { return state_; }

private:
    int tolerance_px_;
    int stable_frames_;
    int aligned_frames_;
    int max_missed_;

    AlignmentState state_{AlignmentState::SEARCH};
    int consecutive_detections_{0};
    int consecutive_aligned_{0};
    int missed_frames_{0};
};

} // namespace fiducial_detector
