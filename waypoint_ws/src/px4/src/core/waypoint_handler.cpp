 
#include "utils/waypoint_handler.h"

#include <algorithm>

namespace px4
{

// ── Konfigurasi internal ────────────────────────────────────────────
// Nilai ini dulunya konstanta global di Python, sekarang jadi
// konstanta lokal kelas supaya WaypointHandler self-contained.

namespace
{
// YAW_THRESHOLD dilonggarkan dari 0.05 (~3°) ke 0.26 (~15°). Nilai lama
// terlalu ketat dipakai bareng gate biner lama (facing?full:stop) —
// sekarang gate biner sudah diganti skala kontinu (yawAlignmentFactor),
// jadi threshold ini cuma dipakai untuk field YawResult::facing (info),
// tidak lagi menentukan stop-total. Tetap dilonggarkan supaya "facing"
// tidak terlalu cepat berubah status akibat noise kecil.
constexpr double YAW_THRESHOLD    = 0.26;  // radian (~15°)
constexpr double WAYPOINT_RADIUS  = 0.8;   // meter toleransi horizontal
constexpr double ALT_THRESHOLD    = 0.15;  // meter toleransi altitude
constexpr double SPEED_MS         = 1.5;   // m/s horizontal max
constexpr double SPEED_VERT       = 0.4;   // m/s vertikal max
constexpr double PURE_ALT_RADIUS  = 0.3;   // meter — dianggap pure altitude
constexpr double YAW_FREEZE_RADIUS = 0.8;  // meter — kunci target_yaw dekat target (redam bearing liar)
}  // namespace

WaypointHandler::WaypointHandler(std::vector<Waypoint> waypoints)
: waypoints_(std::move(waypoints))
{
    labels_ = {
        "WP1-marker1",
        "WP2-markerbox",
        "WP3-marker3",
        "WP4-marker4",
        "WP4b-lewat_gate",
        "WP4c-climb_5m",
        "WP5-final_approach",
    };
}

size_t WaypointHandler::size() const
{
    return waypoints_.size();
}

const Waypoint & WaypointHandler::at(size_t idx) const
{
    return waypoints_.at(idx);
}

const std::vector<Waypoint> & WaypointHandler::all() const
{
    return waypoints_;
}

std::string WaypointHandler::labelAt(size_t idx) const
{
    if (idx < labels_.size()) return labels_[idx];
    return "WP" + std::to_string(idx + 1);
}

double WaypointHandler::horizontalDistance(
    double drone_x, double drone_y,
    double target_n, double target_e)
{
    const double err_n = target_n - drone_x;
    const double err_e = target_e - drone_y;
    return std::hypot(err_n, err_e);
}

double WaypointHandler::altitudeError(double drone_z, double target_d)
{
    return target_d - drone_z;
}

bool WaypointHandler::isPureAltitude(double dist) const
{
    return dist < PURE_ALT_RADIUS;
}

bool WaypointHandler::isAltitudeReached(double alt_error) const
{
    return std::abs(alt_error) < ALT_THRESHOLD;
}

bool WaypointHandler::isWaypointReached(double dist, bool alt_reached) const
{
    return dist < WAYPOINT_RADIUS && alt_reached;
}

YawResult WaypointHandler::computeYaw(
    double drone_x, double drone_y, double drone_yaw,
    double target_n, double target_e)
{
    const double err_n = target_n - drone_x;
    const double err_e = target_e - drone_y;
    const double dist  = std::hypot(err_n, err_e);

    double target_yaw;

    // FIX: jika drone sudah sangat dekat target secara horizontal,
    // atan2(~0, ~0) akan menghasilkan yaw acak/noise tiap tick.
    // Solusinya: pakai yaw terakhir yang masih valid (dist cukup jauh).
    // Radius freeze SENGAJA dipisah dari PURE_ALT_RADIUS (lihat komentar
    // di header) — dulu dua-duanya pakai PURE_ALT_RADIUS (0.3m), terlalu
    // sempit untuk meredam bearing yang liar saat drone masih 0.5-2m
    // dari target (persis rentang jarak drone orbit di WP2 lapangan).
    if (dist < YAW_FREEZE_RADIUS) {
        target_yaw = last_valid_yaw_;
    } else {
        target_yaw      = std::atan2(err_e, err_n);
        last_valid_yaw_ = target_yaw;
    }

    const double yaw_error = std::atan2(
        std::sin(target_yaw - drone_yaw),
        std::cos(target_yaw - drone_yaw)
    );

    return {
        target_yaw,
        yaw_error,
        std::abs(yaw_error) < YAW_THRESHOLD
    };
}

void WaypointHandler::computeApproachVelocity(
    double err_n, double err_e, double dist,
    double & vx, double & vy) const
{
    const double arah  = std::atan2(err_e, err_n);
    const double speed = std::min(SPEED_MS, std::max(0.15, dist * 0.05));
    vx = speed * std::cos(arah);
    vy = speed * std::sin(arah);
}

double WaypointHandler::computeVerticalVelocity(double alt_error) const
{
    const double vz = alt_error * 0.4;
    return std::max(-SPEED_VERT, std::min(SPEED_VERT, vz));
}

double WaypointHandler::yawAlignmentFactor(double yaw_error) const
{
    // max(0, cos(yaw_error)): 1.0 saat pas menghadap target, turun
    // smooth ke 0 saat error mendekati ±90°, dan tetap 0 (bukan
    // negatif/mundur) kalau target ada di belakang drone.
    return std::max(0.0, std::cos(yaw_error));
}

}  // namespace px4
