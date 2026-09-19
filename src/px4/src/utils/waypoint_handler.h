#pragma once

#include <cmath>
#include <string>
#include <vector>

namespace px4
{

// ==================================================================
// WaypointHandler
//
// Kelas murni (tanpa rclcpp) yang menangani semua matematika navigasi:
//   - struktur data waypoint NED
//   - hitung yaw target, error yaw, status "facing"
//   - hitung jarak horizontal dan error altitude
//   - deteksi waypoint "pure altitude" (climb/descend di tempat)
//   - deteksi "reached" dengan toleransi
//
// Tidak ada side effect ke ROS sama sekali — supaya gampang di-unit-test
// dan supaya MissionManager tidak perlu tahu detail matematika.
// ==================================================================

struct Waypoint
{
    double n;   // North (meter)
    double e;   // East  (meter)
    double d;   // Down  (meter, negatif = naik)
};

struct YawResult
{
    double target_yaw;   // radian
    double yaw_error;    // radian, [-pi, pi]
    bool   facing;       // true jika yaw_error < threshold
};

struct Quaternion
{
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

class WaypointHandler
{
public:
    explicit WaypointHandler(std::vector<Waypoint> waypoints);

    // ── Akses waypoint ─────────────────────────────────────────────
    size_t size() const;
    const Waypoint & at(size_t idx) const;
    const std::vector<Waypoint> & all() const;
    std::string labelAt(size_t idx) const;
    void translateWaypointsFrom(size_t start_idx, double delta_n, double delta_e);

    // ── Hitung yaw ke target ─────────────────────────────────────────
    // dist_to_target: jarak horizontal drone ke target (untuk deteksi
    // pure-altitude / mencegah yaw acak saat sudah dekat)
    YawResult computeYaw(
        double drone_x, double drone_y, double drone_yaw,
        double target_n, double target_e);

    // Reset filter saat pindah waypoint. Filter dimulai dari yaw aktual,
    // sehingga gate waypoint baru tidak dapat lolos memakai yaw lama.
    void resetYawSmoothing(double current_yaw);

    // ── Hitung jarak horizontal ───────────────────────────────────────
    static double horizontalDistance(
        double drone_x, double drone_y,
        double target_n, double target_e);

    // ── Hitung error altitude ─────────────────────────────────────────
    static double altitudeError(double drone_z, double target_d);

    // ── Status check ───────────────────────────────────────────────
    bool isPureAltitude(double dist) const;
    bool isAltitudeReached(double alt_error) const;
    bool isWaypointReached(double dist, bool alt_reached) const;

    // ── Gate "boleh mulai maju"? ──────────────────────────────────
    // Threshold ketat (~5°), BEDA dari YAW_THRESHOLD (~15°, dipakai
    // untuk field YawResult::facing yang cuma informatif). Dipakai
    // MissionManager untuk kunci gerak horizontal sampai yaw benar-benar
    // pas menghadap target — supaya drone berputar dulu di tempat,
    // baru maju, tidak oleng/maju sambil masih muter.
    bool isYawAligned(double yaw_error) const;

    // ── Hitung velocity command (vx, vy, vz) ──────────────────────────
    // dipanggil saat bukan pure-altitude. Skala hasilnya dengan
    // yawAlignmentFactor() supaya transisi rotate->maju smooth, bukan
    // on/off (lihat komentar di yawAlignmentFactor).
    void computeApproachVelocity(
        double err_n, double err_e, double dist,
        double & vx, double & vy) const;
    double computeApproachSpeed(double dist) const;

    double computeVerticalVelocity(double alt_error) const;

    // ── Faktor skala kecepatan maju berdasar seberapa pas heading ───
    // FIX: dulu ada gate biner (facing ? full-speed : stop-total-lalu-
    // muter), yang di lapangan terbukti menyebabkan drone orbit/loiter
    // di sekitar waypoint yang butuh belok besar (mis. WP2 yang butuh
    // putar ~90°) — begitu drone maju sedikit, bearing ke target
    // bergeser, yaw_error nyebrang threshold, drone berhenti total,
    // muter lagi, siklus berulang tanpa pernah konvergen.
    //
    // Fix: skala kecepatan maju secara KONTINU dengan cos(yaw_error)
    // (dibatasi >=0 supaya tidak pernah mundur). Hasilnya: makin pas
    // heading-nya, makin cepat maju; makin meleset, makin pelan sambil
    // tetap berputar — transisi mulus, tidak ada stop-go mendadak.
    double yawAlignmentFactor(double yaw_error) const;

private:
    std::vector<Waypoint> waypoints_;
    std::vector<std::string> labels_;

    // yaw terakhir yang valid — dipakai saat drone terlalu dekat target
    // secara horizontal (mencegah atan2(~0,~0) menghasilkan yaw acak).
    // Radius freeze ini SENGAJA dipisah dari PURE_ALT_RADIUS (constant
    // internal di .cpp) — PURE_ALT_RADIUS untuk deteksi "waypoint climb
    // di tempat", radius freeze-yaw untuk redam bearing yang liar dekat
    // target. Dua tujuan berbeda, jangan digabung jadi satu konstanta.
    double last_valid_yaw_ {0.0};
    bool has_valid_yaw_ {false};

    // Setpoint yaw yang sudah dilembutkan. Kita simpan sebagai quaternion
    // agar smoothing orientasi tetap halus saat sudut melintasi wrapping
    // dan berubah secara bertahap dari yaw saat ini ke yaw target.
    Quaternion smoothed_target_quaternion_ {};
    double smoothed_target_yaw_ {0.0};
    double smoothed_yaw_rate_rad_s_ {0.0};
    bool has_smoothed_yaw_ {false};
};

}  // namespace px4
