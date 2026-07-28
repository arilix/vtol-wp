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
constexpr double SPEED_VERT       = 0.4;   // m/s vertikal max
constexpr double PURE_ALT_RADIUS  = 0.3;   // meter — dianggap pure altitude
constexpr double YAW_FREEZE_RADIUS = 0.8;  // meter — kunci target_yaw dekat target (redam bearing liar)
constexpr double YAW_DEADBAND_RAD = 0.012;  // ~0.7deg — abaikan noise kecil tanpa membuat setpoint kaku
constexpr double YAW_MAX_RATE_RAD_S = 0.35;  // ~20deg/s
constexpr double YAW_ACCEL_RAD_S2   = 0.70;  // ~40deg/s2, ramp naik/turun halus
constexpr double CONTROL_DT_S       = 0.10;  // timer MissionManager 10Hz

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

Quaternion smoothYawQuaternion(
    const Quaternion & current, const Quaternion & target,
    double & yaw_rate_rad_s)
{
    const double current_yaw = quatToYaw(current);
    const double target_yaw = quatToYaw(target);
    const double delta = wrapPi(target_yaw - current_yaw);

    if (std::abs(delta) < YAW_DEADBAND_RAD) {
        yaw_rate_rad_s = 0.0;
        return current;
    }

    // Profil trapezoidal: akselerasi menuju 20deg/s, lalu otomatis deselerasi
    // berdasarkan sisa sudut. Ini menghindari hentakan torsi di awal/akhir yaw
    // yang dapat menggeser badan walau position setpoint tetap.
    const double stopping_limited_rate = std::sqrt(
        2.0 * YAW_ACCEL_RAD_S2 * std::abs(delta));
    const double desired_rate = std::copysign(
        std::min(YAW_MAX_RATE_RAD_S, stopping_limited_rate), delta);
    const double max_rate_change = YAW_ACCEL_RAD_S2 * CONTROL_DT_S;
    yaw_rate_rad_s += std::clamp(
        desired_rate - yaw_rate_rad_s,
        -max_rate_change, max_rate_change);

    double step = yaw_rate_rad_s * CONTROL_DT_S;
    if (std::abs(step) > std::abs(delta)) {
        step = delta;
        yaw_rate_rad_s = 0.0;
    }
    const double smoothed_yaw = current_yaw + step;
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

void WaypointHandler::translateWaypointsFrom(
    size_t start_idx, double delta_n, double delta_e)
{
    if (start_idx >= waypoints_.size()) return;
    for (size_t i = start_idx; i < waypoints_.size(); ++i) {
        waypoints_[i].n += delta_n;
        waypoints_[i].e += delta_e;
    }
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
            smoothed_target_quaternion_, raw_target_quat,
            smoothed_yaw_rate_rad_s_);
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
    smoothed_yaw_rate_rad_s_ = 0.0;
    has_smoothed_yaw_ = true;
}

void WaypointHandler::computeApproachVelocity(
    double err_n, double err_e, double dist,
    double & vx, double & vy) const
{
    const double arah  = std::atan2(err_e, err_n);
    // Gain lama (0.05) baru mencapai cap lama (1.8 m/s) di jarak ~30m —
    // tidak realistis untuk leg beberapa meter (drone jadi kelihatan
    // "lambat/tidak nemu-nemu"). Gain 0.35 mencapai cap itu di ~5m, masih
    // landai mendekati target (radius reached 0.30m) untuk stop mulus.
    //
    // Zona pengereman (dist <= BRAKE_ZONE_M) SENGAJA memakai formula lama
    // persis (gain 0.35, floor 0.3) — tidak diubah sama sekali — supaya
    // perilaku approach akhir ke waypoint (yang mencegah overshoot) tetap
    // identik. Hanya kecepatan cruise DI LUAR zona itu yang dipercepat,
    // dengan gain lebih curam menuju cap CRUISE_SPEED_MS yang lebih tinggi
    // dari cap lama (1.8). Kedua formula bersambung mulus (tanpa lompatan
    // kecepatan) tepat di batas BRAKE_ZONE_M.
    constexpr double BRAKE_ZONE_M   = 2.0;   // meter — di bawah ini, formula lama
    constexpr double BRAKE_GAIN     = 0.35;  // sama seperti sebelumnya
    constexpr double CRUISE_SPEED_MS = 2.2;  // m/s — naik dari SPEED_MS lama (1.8)
    constexpr double CRUISE_GAIN    = 0.45;  // m/s per meter di luar zona rem

    double speed;
    if (dist <= BRAKE_ZONE_M) {
        speed = std::max(0.3, dist * BRAKE_GAIN);
    } else {
        const double brake_zone_speed = BRAKE_ZONE_M * BRAKE_GAIN;
        speed = std::min(CRUISE_SPEED_MS,
            brake_zone_speed + (dist - BRAKE_ZONE_M) * CRUISE_GAIN);
    }
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
