 
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include "utils/waypoint_handler.h"
#include "utils/control_module.h"
#include "utils/vehicle_state.h"

#include <memory>
#include <vector>

namespace px4
{

// ==================================================================
// MissionManager
//
// Node ROS2 utama — state machine misi drone.
// Mengkoordinasikan WaypointHandler (logic navigasi) dan
// ControlModule (komunikasi PX4), tapi tidak melakukan keduanya
// sendiri.
//
// Fase:
//   INIT        → kirim setpoint diam 5 tick, lalu set offboard + arm
//   TAKEOFF     → naik ke altitude waypoint pertama
//   HOVER       → stabilisasi 3 detik (30 tick @ 10Hz) setelah takeoff
//   MISSION     → kunjungi semua waypoint berurutan
//   LAND_CMD    → kirim perintah land (dengan heartbeat tetap jalan)
//   WAIT_DISARM → tunggu PX4 konfirmasi disarmed, lalu selesai
// ==================================================================

enum class Phase
{
    INIT,
    WAIT_ARM,
    TAKEOFF,
    HOVER,
    MISSION,
    LAND_CMD,
    WAIT_DISARM,
};

class MissionManager : public rclcpp::Node
{
public:
    MissionManager();

private:
    void loop();

    // ── Setiap fase punya method sendiri agar loop() mudah dibaca ───
    void runInit();
    void runWaitArm();
    void runTakeoff();
    void runHover();
    void runMission();
    void runLandCmd();
    void runWaitDisarm();

    double takeoffTargetDown() const;
    double currentAltitudeDown() const;
    double currentAltitudeAgl() const;
    double toPx4DownForAltitudeTarget(double mission_down) const;
    double toPx4North(double mission_north) const;
    double toPx4East(double mission_east) const;
    double toPx4Down(double mission_down) const;
    bool ensureMissionWaypointsReady();

    // ── State drone (diperbarui via callback ControlModule) ──────────
    VehicleState vehicle_;

    // ── Callback dari ControlModule ──────────────────────────────────
    void onPositionUpdate(const ControlModule::PositionSample & s);
    void onGlobalPositionUpdate(const ControlModule::GlobalPositionSample & s);
    void onRangeUpdate(const ControlModule::RangeSample & s);
    void onStatusUpdate(uint8_t arming_state);

    // Debug: hitung berapa kali onStatusUpdate benar-benar terpanggil,
    // untuk membedakan "callback tidak pernah jalan" vs "jalan tapi
    // nilainya memang 0/tidak berubah".
    uint32_t status_update_count_ {0};

    // ── Kompensasi reset EKF ──────────────────────────────────────
    // Lihat komentar di ControlModule::PositionSample. Origin digeser
    // sebesar delta setiap kali reset counter berubah, supaya posisi
    // relatif (vehicle_.position) tetap kontinu walau EKF PX4 melompat.
    bool    have_reset_counters_ {false};
    uint8_t last_xy_reset_counter_ {0};
    uint8_t last_z_reset_counter_  {0};
    uint8_t last_heading_reset_counter_ {0};

    // ── Deteksi data posisi basi (link uXRCE-DDS macet/lambat) ──────
    // Terbukti di lapangan: kalau bandwidth link serial (MicroXRCEAgent)
    // jenuh — misal setelah offboard mulai streaming heartbeat 10Hz —
    // topik /fmu/out/vehicle_local_position bisa berhenti update total
    // padahal drone tetap terbang. Tanpa guard ini, MissionManager akan
    // terus mengira altitude belum tercapai selamanya (data yang dipakai
    // untuk cek "reached" sudah beku), padahal ini murni masalah data
    // stale, bukan drone belum sampai.
    rclcpp::Time last_position_time_;
    bool isPositionStale(double threshold_s) const;

    // ── Altitude dari TFmini lidar (/range) ─────────────────────────
    rclcpp::Time last_range_time_;
    double lidar_altitude_m_ {0.0};
    bool got_lidar_altitude_ {false};
    bool isLidarStale(double threshold_s) const;

    // ── Komponen ───────────────────────────────────────────────────
    std::unique_ptr<ControlModule>  control_;
    std::unique_ptr<WaypointHandler> waypoints_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ── Counter & fase ────────────────────────────────────────────
    int    counter_         {0};
    int    hover_counter_   {0};
    int    wp_hold_counter_ {0};
    size_t current_wp_      {0};
    Phase  phase_           {Phase::INIT};

    // ── Konfigurasi ────────────────────────────────────────────────
    double origin_north_ {0.0};
    double origin_east_  {0.0};
    double origin_down_  {0.0};
    bool   got_origin_   {false};

    // ── Origin GPS untuk konversi LLA -> NED misi ──────────────────
    double origin_lat_deg_ {0.0};
    double origin_lon_deg_ {0.0};
    double origin_alt_m_   {0.0};
    bool   got_global_origin_ {false};
    bool   mission_waypoints_ready_ {false};

    // ── Stabilisasi origin ────────────────────────────────────────
    // BUG NYATA yang terjadi di lapangan: origin dulu di-lock dari
    // SATU sample pertama saja. Sample pertama EKF (terutama z/altitude)
    // sering belum konvergen (baru boot, GPS/baro belum settle), bisa
    // meleset puluhan meter dari nilai stabilnya. Akibatnya seluruh
    // target altitude relatif ikut meleset sejauh itu -> drone terus
    // "mengejar" target yang sebenarnya puluhan meter di atas yang
    // diinginkan operator (persis kejadian "terbang sangat tinggi").
    //
    // Fix: origin baru dikunci setelah posisi mentah PX4 terbukti
    // stabil (tidak berubah lebih dari toleransi) selama beberapa
    // ratus ms berturut-turut.
    PositionNED origin_candidate_;
    rclcpp::Time candidate_since_;
    bool         has_candidate_ {false};

    // Tambahan command altitude ke arah atas (meter).
    // 0.0 = tidak ada kompensasi. Coba 0.3-0.5 hanya jika log raw PX4
    // membuktikan drone memang berhenti di bawah target relatif.
    double altitude_command_bias_m_ {0.0};
};

}  // namespace px4

