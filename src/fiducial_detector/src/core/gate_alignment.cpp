#include "utils/gate_alignment.h"
#include <cmath>
#include <cstdio>

namespace fiducial_detector {

std::string GateError::stateName() const {
    switch (state) {
        case AlignmentState::SEARCH:       return "SEARCH";
        case AlignmentState::DETECTED:     return "DETECTED";
        case AlignmentState::TRACKING:     return "TRACKING";
        case AlignmentState::CENTERING:    return "CENTERING";
        case AlignmentState::ALIGNED:      return "ALIGNED";
        case AlignmentState::GATE_READY:   return "GATE_READY";
        case AlignmentState::PASS_THROUGH: return "PASS_THROUGH";
    }
    return "UNKNOWN";
}

std::string GateError::toJson() const {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "{\"error_x\":%.1f,\"error_y\":%.1f,"
        "\"distance\":%.3f,\"alignment_state\":\"%s\","
        "\"marker_id\":%d}",
        error_x, error_y, distance, stateName().c_str(), marker_id);
    return buf;
}

GateAlignmentEngine::GateAlignmentEngine(
    int tolerance_px, int stable_frames, int aligned_frames, int max_missed)
    : tolerance_px_(tolerance_px)
    , stable_frames_(stable_frames)
    , aligned_frames_(aligned_frames)
    , max_missed_(max_missed)
{}

GateError GateAlignmentEngine::update(
    cv::Point2f marker_center,
    const cv::Size& frame_size,
    float distance,
    int marker_id)
{
    const float cx = frame_size.width  * 0.5f;
    const float cy = frame_size.height * 0.5f;

    const float ex = marker_center.x - cx;
    const float ey = marker_center.y - cy;

    missed_frames_ = 0;
    ++consecutive_detections_;

    const bool within_tol = (std::abs(ex) <= tolerance_px_)
                         && (std::abs(ey) <= tolerance_px_);

    // State machine transitions
    switch (state_) {
        case AlignmentState::SEARCH:
            state_ = AlignmentState::DETECTED;
            consecutive_aligned_ = 0;
            break;

        case AlignmentState::DETECTED:
            if (consecutive_detections_ >= stable_frames_)
                state_ = within_tol ? AlignmentState::ALIGNED
                                    : AlignmentState::TRACKING;
            break;

        case AlignmentState::TRACKING:
            if (within_tol) {
                ++consecutive_aligned_;
                state_ = (consecutive_aligned_ >= aligned_frames_)
                         ? AlignmentState::GATE_READY
                         : AlignmentState::ALIGNED;
            } else {
                consecutive_aligned_ = 0;
                state_ = AlignmentState::CENTERING;
            }
            break;

        case AlignmentState::CENTERING:
            if (within_tol) {
                consecutive_aligned_ = 0;
                state_ = AlignmentState::ALIGNED;
            }
            break;

        case AlignmentState::ALIGNED:
            if (within_tol) {
                ++consecutive_aligned_;
                if (consecutive_aligned_ >= aligned_frames_)
                    state_ = AlignmentState::GATE_READY;
            } else {
                consecutive_aligned_ = 0;
                state_ = AlignmentState::CENTERING;
            }
            break;

        case AlignmentState::GATE_READY:
            if (!within_tol) {
                consecutive_aligned_ = 0;
                state_ = AlignmentState::CENTERING;
            }
            // PASS_THROUGH transition reserved for external controller signal
            break;

        case AlignmentState::PASS_THROUGH:
            break;
    }

    GateError err;
    err.error_x   = ex;
    err.error_y   = ey;
    err.distance  = distance;
    err.state     = state_;
    err.marker_id = marker_id;
    return err;
}

void GateAlignmentEngine::reset() {
    state_                   = AlignmentState::SEARCH;
    consecutive_detections_  = 0;
    consecutive_aligned_     = 0;
    missed_frames_           = 0;
}

} // namespace fiducial_detector
