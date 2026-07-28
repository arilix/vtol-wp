#include "utils/ground_lock.h"

#include <cmath>

namespace px4
{

GroundLock::GroundLock(const Config & config)
: config_(config), vision_lock_(config.vision_cfg)
{}

void GroundLock::update(
    double cx_px, double cy_px,
    double frame_width_px, double frame_height_px,
    double altitude_m,
    const PositionNED & drone_pos, double drone_yaw,
    double now_s)
{
    // Altitude tidak valid -> proyeksi ground-plane tidak berarti
    // (skala x_m/y_m langsung sebanding altitude_m). Diamkan sample
    // ini, jangan sampai dorong lock ke offset nol/salah arah.
    if (!(altitude_m > 0.0) || !std::isfinite(altitude_m)) {
        return;
    }
    if (!(config_.camera_fx_px > 0.0) || !(config_.camera_fy_px > 0.0)) {
        return;
    }

    const double cx0_px = frame_width_px * 0.5;
    const double cy0_px = frame_height_px * 0.5;

    const double x_m = (cx_px - cx0_px) / config_.camera_fx_px * altitude_m;
    const double y_m = (cy_px - cy0_px) / config_.camera_fy_px * altitude_m;

    // Sama seperti onMarkerPoseUpdate() di mission_manager.cpp: frame
    // optical OpenCV x=kanan, y=bawah -> "atas gambar" = -y.
    const double body_up    = -y_m;
    const double body_right = x_m;

    vision_lock_.update(body_up, body_right, drone_pos, drone_yaw, now_s);
}

}  // namespace px4
