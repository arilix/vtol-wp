#include "utils/vision_lock.h"

#include <cmath>

namespace px4
{

VisionLock::VisionLock(const Config & config)
: config_(config)
{}

void VisionLock::update(
    double body_up, double body_right,
    const PositionNED & drone_pos, double drone_yaw,
    double now_s)
{
    const double theta = config_.camera_mount_yaw_deg * 3.14159265358979323846 / 180.0;
    const double ct = std::cos(theta);
    const double st = std::sin(theta);

    // Body up/right (kamera, sudah dikoreksi mount yaw) -> body forward/right.
    const double body_forward = body_up * ct - body_right * st;
    const double body_right2  = body_up * st + body_right * ct;

    // Body forward/right -> NED, pakai yaw drone sekarang (bentuk rotasi
    // sama seperti rotateToTrueNed() di mission_manager.cpp).
    const double cy = std::cos(drone_yaw);
    const double sy = std::sin(drone_yaw);
    double offset_north = body_forward * cy - body_right2 * sy;
    double offset_east  = body_forward * sy + body_right2 * cy;

    // Clamp magnitude — koreksi yang valid untuk masalah ini berskala
    // cm, jadi mendekati batas ini adalah tanda ada yang salah, bukan
    // koreksi besar yang sah.
    const double mag = std::hypot(offset_north, offset_east);
    last_sample_clamped_ = false;
    if (mag > config_.max_correction_m && mag > 1e-9) {
        const double scale = config_.max_correction_m / mag;
        offset_north *= scale;
        offset_east  *= scale;
        last_sample_clamped_ = true;
    }

    PositionNED candidate;
    candidate.north = drone_pos.north + offset_north;
    candidate.east  = drone_pos.east  + offset_east;
    candidate.down  = drone_pos.down;

    // Debounce: reset ke 1 (bukan 0) kalau jeda terlalu lama ATAU
    // sample meloncat terlalu jauh dari sample raw sebelumnya —
    // /fiducial/pose tidak bawa marker_id, jadi lompatan besar adalah
    // satu-satunya sinyal "kemungkinan ganti marker".
    if (!has_prev_raw_) {
        consecutive_samples_ = 1;
    } else {
        const double gap  = now_s - prev_raw_time_;
        const double jump = std::hypot(
            candidate.north - prev_raw_north_,
            candidate.east  - prev_raw_east_);

        if (gap >= config_.max_sample_gap_s || jump > config_.max_jump_m) {
            consecutive_samples_ = 1;
        } else {
            ++consecutive_samples_;
        }
    }

    prev_raw_north_ = candidate.north;
    prev_raw_east_  = candidate.east;
    prev_raw_time_  = now_s;
    has_prev_raw_   = true;

    if (consecutive_samples_ >= config_.min_consecutive_samples) {
        locked_target_ = candidate;
        last_update_s_ = now_s;
        has_update_    = true;
    }
}

bool VisionLock::isEngaged(double now_s) const
{
    if (!has_update_) return false;
    if (consecutive_samples_ < config_.min_consecutive_samples) return false;
    return (now_s - last_update_s_) < config_.max_sample_gap_s;
}

}  // namespace px4
