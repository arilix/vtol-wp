#include "utils/waypoint_handler.h"

#include <algorithm>

namespace px4
{

namespace
{
// YAW_THRESHOLD dilonggarkan dari 0.05 (~3°) ke 0.26 (~15°). Nilai lama
// terlalu ketat dipakai bareng gate biner lama (facing?full:stop) —
// sekarang gate biner sudah diganti skala kontinu (yawAlignmentFactor),
// jadi threshold ini cuma dipakai untuk field YawResult::facing (info),
// tidak lagi menentukan stop-total. Tetap dilonggarkan supaya "facing"
// tidak terlalu cepat berubah status akibat noise kecil.
constexpr double YAW_THRESHOLD    = 0.26;  // radian (~15°)
// Threshold ketat untuk gate "boleh mulai maju" (isYawAligned) — drone
// dikunci berputar di tempat sampai yaw error di bawah ini, baru boleh
// gerak horizontal. Sengaja jauh lebih ketat dari YAW_THRESHOLD supaya
// tidak mulai maju saat masih miring beberapa belas derajat.
constexpr double YAW_LOCK_THRESHOLD = 0.09;  // radian (~5°)
// Radius ini adalah syarat waypoint boleh dinyatakan selesai. Lebih kecil
// dari batas operasi 0.5 m supaya fase position-hold masih punya margin
// untuk memusatkan drone sebelum lanjut ke waypoint berikutnya.
constexpr double WAYPOINT_RADIUS  = 0.30;  // meter toleransi horizontal
constexpr double ALT_THRESHOLD    = 0.15;  // meter toleransi altitude
constexpr double SPEED_MS         = 1.8;   // m/s horizontal max
constexpr double SPEED_VERT       = 0.4;   // m/s vertikal max
constexpr double PURE_ALT_RADIUS  = 0.3;   // meter — dianggap pure altitude
constexpr double YAW_FREEZE_RADIUS = 0.8;  // meter — kunci target_yaw dekat target (redam bearing liar)
constexpr double YAW_DEADBAND_RAD = 0.012;  // ~0.7deg — abaikan noise kecil tanpa membuat setpoint kaku
constexpr double YAW_MAX_STEP_RAD  = 0.022;  // ~1.3deg/tick @10Hz, cukup lembut tapi tetap responsif

double wrapPi(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

Quaternion quatFromYaw(double yaw)
{
    const double half = yaw * 0.5;
    return {std::cos(half), 0.0, 0.0, std::sin(half)};
}

double quatToYaw(const Quaternion & q)
{
    return std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

Quaternion quatNormalize(const Quaternion & q)
{
    const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm < 1e-12) {
        return {1.0, 0.0, 0.0, 0.0};
    }
    return {q.w / norm, q.x / norm, q.y / norm, q.z / norm};
}

Quaternion quatSlerp(const Quaternion & a, const Quaternion & b, double t)
{
    if (t <= 0.0) return a;
    if (t >= 1.0) return b;

    double cosOmega = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    Quaternion b2 = b;
    if (cosOmega < 0.0) {
        b2 = {-b.w, -b.x, -b.y, -b.z};
        cosOmega = -cosOmega;
    }

    if (cosOmega > 0.9995) {
        const Quaternion lerp_q = {
            a.w + t * (b2.w - a.w),
            a.x + t * (b2.x - a.x),
            a.y + t * (b2.y - a.y),
            a.z + t * (b2.z - a.z)};
        return quatNormalize(lerp_q);
    }

    const double omega = std::acos(std::max(-1.0, std::min(1.0, cosOmega)));
    const double sinOmega = std::sin(omega);
    const double s0 = std::sin((1.0 - t) * omega) / sinOmega;
    const double s1 = std::sin(t * omega) / sinOmega;

    return quatNormalize({
        a.w * s0 + b2.w * s1,
        a.x * s0 + b2.x * s1,
        a.y * s0 + b2.y * s1,
        a.z * s0 + b2.z * s1});
}

Quaternion smoothYawQuaternion(const Quaternion & current, const Quaternion & target)
{
    const double current_yaw = quatToYaw(current);
    const double target_yaw = quatToYaw(target);
    const double delta = wrapPi(target_yaw - current_yaw);

    if (std::abs(delta) < YAW_DEADBAND_RAD) {
        return current;
    }

    const double normalized = std::min(1.0, std::abs(delta) / (0.6));
    const double alpha = 0.16 + 0.24 * normalized;
    const double limited_delta = std::max(
        -YAW_MAX_STEP_RAD,
        std::min(YAW_MAX_STEP_RAD, delta));

    const double smoothed_yaw = current_yaw + alpha * limited_delta;
    return quatFromYaw(smoothed_yaw);
}
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

bool WaypointHandler::isYawAligned(double yaw_error) const
{
    return std::abs(yaw_error) < YAW_LOCK_THRESHOLD;
}

YawResult WaypointHandler::computeYaw(
    double drone_x, double drone_y, double drone_yaw,
    double target_n, double target_e)
{
    const double err_n = target_n - drone_x;
    const double err_e = target_e - drone_y;
    const double dist  = std::hypot(err_n, err_e);

    double raw_target_yaw;

    // FIX: jika drone sudah sangat dekat target secara horizontal,
    // atan2(~0, ~0) akan menghasilkan yaw acak/noise tiap tick.
    // Solusinya: pakai yaw terakhir yang masih valid (dist cukup jauh).
    // Radius freeze SENGAJA dipisah dari PURE_ALT_RADIUS (lihat komentar
    // di header) — dulu dua-duanya pakai PURE_ALT_RADIUS (0.3m), terlalu
    // sempit untuk meredam bearing yang liar saat drone masih 0.5-2m
    // dari target (persis rentang jarak drone orbit di WP2 lapangan).
    if (dist < YAW_FREEZE_RADIUS) {
        raw_target_yaw = has_valid_yaw_ ? last_valid_yaw_ : drone_yaw;
    } else {
        raw_target_yaw  = std::atan2(err_e, err_n);
        last_valid_yaw_ = raw_target_yaw;
        has_valid_yaw_  = true;
    }

    const Quaternion raw_target_quat = quatFromYaw(raw_target_yaw);

    if (!has_smoothed_yaw_) {
        smoothed_target_quaternion_ = raw_target_quat;
        smoothed_target_yaw_ = raw_target_yaw;
        has_smoothed_yaw_ = true;
    } else {
        smoothed_target_quaternion_ = smoothYawQuaternion(
            smoothed_target_quaternion_, raw_target_quat);
        smoothed_target_yaw_ = quatToYaw(smoothed_target_quaternion_);
    }

    const double yaw_error = wrapPi(smoothed_target_yaw_ - drone_yaw);

    return {
        smoothed_target_yaw_,
        yaw_error,
        std::abs(yaw_error) < YAW_THRESHOLD
    };
}

void WaypointHandler::resetYawSmoothing(double current_yaw)
{
    has_valid_yaw_ = false;
    last_valid_yaw_ = current_yaw;
    smoothed_target_quaternion_ = quatFromYaw(current_yaw);
    smoothed_target_yaw_ = current_yaw;
    has_smoothed_yaw_ = true;
}

void WaypointHandler::computeApproachVelocity(
    double err_n, double err_e, double dist,
    double & vx, double & vy) const
{
    const double arah  = std::atan2(err_e, err_n);
    // Gain lama (0.05) baru mencapai SPEED_MS di jarak ~30m — tidak
    // realistis untuk leg beberapa meter (drone jadi kelihatan "lambat/
    // tidak nemu-nemu"). Gain 0.35 mencapai SPEED_MS(1.8) di ~5m,
    // masih landai mendekati target (radius reached 0.30m) untuk stop mulus.
    const double speed = std::min(SPEED_MS, std::max(0.3, dist * 0.35));
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
