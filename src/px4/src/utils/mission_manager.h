#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <std_msgs/msg/string.hpp>

#include "utils/waypoint_handler.h"
#include "utils/control_module.h"
#include "utils/vehicle_state.h"
#include "utils/vision_lock.h"
#include "utils/ground_lock.h"
#include "utils/gate_centering_lock.h"

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
    TAKEOFF_MARKER,
    MISSION,
    GATE_PASS,
    LAND_CMD,
    WAIT_DISARM,
};

// Dua tahap runGatePassMission(): CENTER (diam, koreksi lateral sampai
// stabil di tengah gerbang) lalu ADVANCE (maju lurus menembus gerbang
// sambil terus dikoreksi lateral) — persis dua fungsi terpisah di versi
// SITL/MAVROS asli (centeringGateLivoxSimple lalu
// advanceThroughLivoxCorridor), disatukan jadi satu state machine tick-
// based di sini.
enum class GatePassStage
{
    CENTER,
    ADVANCE,
    DONE,
};

enum class WaypointPhase
{
    APPROACH,
    SEARCH_MARKER,
    CENTER_MARKER,
    ALIGN_MARKER_HEADING,
};

enum class GripperDropState
{
    IDLE,
    OPEN_SENT,
    CLOSE_SENT,
    COMPLETE,
};

enum class StartMode
{
    TAKEOFF,
    AIRBORNE_HANDOFF,
    // Bypass total TAKEOFF_MARKER/MISSION (ArUco/YOLO/gripper) — setelah
    // takeoff+hover, langsung masuk Phase::GATE_PASS. Lihat
    // runGatePassMission() dan docs/LIVOX_MID360_INTEGRATION.md §9.
    GATE_PASS,
};

class MissionManager : public rclcpp::Node
{
public:
    MissionManager();
    // Dipanggil main saat SIGINT agar Ctrl+C mengirim LAND sebelum node
    // berhenti dan tidak mengandalkan offboard-loss failsafe PX4.
    void requestGracefulLand();

private:
    void loop();

    // ── Setiap fase punya method sendiri agar loop() mudah dibaca ───
    void runInit();
    void runWaitArm();
    void runTakeoff();
    void runHover();
    void runTakeoffMarker();
    void runMission();
    void runGatePassMission();
    bool runMissionGateAssist(
        double leg_bearing, double along_track_remaining,
        const std::string & waypoint_label);
    void runLandCmd();
    void runWaitDisarm();

    double takeoffTargetDown() const;
    double currentAltitudeDown() const;
    double currentAltitudeAgl() const;
    double missionReferenceYaw() const;
    const char * altitudeSourceLabel() const;

    // Sumber vision yang seharusnya AKTIF sekarang ("ARUCO"/"YOLO"/"NONE"),
    // dipublish tiap tick ke /mission/vision_source_active supaya
    // aruco_node/yolo_camera_node bisa skip inferensi kalau bukan
    // gilirannya (hemat CPU/NPU) tanpa perlu start/stop proses. Lihat
    // PROGRAM_OVERVIEW.md bagian "Ground Lock YOLO (WP1)".
    const char * activeVisionSourceLabel() const;
    double toPx4DownForAltitudeTarget(double mission_down) const;
    double toPx4North(double mission_north) const;
    double toPx4East(double mission_east) const;
    double toPx4Down(double mission_down) const;
    bool ensureMissionWaypointsReady();
    void rebaseMissionOriginToCurrentPosition();

    // Terapkan koreksi vision-lock (kalau engaged) ke target N/E mission-
    // relative, dengan ramp-in linear supaya tidak ada lompatan setpoint
    // mendadak saat lock baru engage. Kalau tidak engaged/disabled,
    // out_north/east == base_north/east (no-op, perilaku sama seperti
    // sebelum vision lock ada).
    void applyVisionLock(
        double base_north, double base_east,
        double & out_north, double & out_east);
    void resetWaypointVisionState();
    void commitCurrentWaypointAnchor();
    void applyLatchedMarkerTarget(
        double base_north, double base_east,
        double & out_north, double & out_east);
    bool runGripperDropIfNeeded(
        double hold_north, double hold_east, double hold_down, double hold_yaw,
        const std::string & waypoint_label);
    void publishGripperCommand(const std::string & command);

    // ── State drone (diperbarui via callback ControlModule) ──────────
    VehicleState vehicle_;

    // ── Callback dari ControlModule ──────────────────────────────────
    void onPositionUpdate(const ControlModule::PositionSample & s);
    void onRangeUpdate(const ControlModule::RangeSample & s);
    void onStatusUpdate(uint8_t arming_state);
    void onMarkerPoseUpdate(const ControlModule::MarkerPoseSample & s);
    void onMarkerCentersUpdate(const ControlModule::MarkerCentersSample & s);
    void onTargetCenterUpdate(const ControlModule::TargetCenterSample & s);
    void onLivoxUpdate(const ControlModule::LivoxSample & s);

    // Debug: hitung berapa kali onStatusUpdate benar-benar terpanggil,
    // untuk membedakan "callback tidak pernah jalan" vs "jalan tapi
    // nilainya memang 0/tidak berubah".
    uint32_t status_update_count_ {0};
    uint32_t position_update_count_ {0};
    bool got_raw_position_ {false};

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
    bool use_lidar_altitude_ {true};
    bool isLidarStale(double threshold_s) const;

    // ── Komponen ───────────────────────────────────────────────────
    std::unique_ptr<ControlModule>  control_;
    std::unique_ptr<WaypointHandler> waypoints_;
    std::unique_ptr<VisionLock>      vision_lock_;
    std::unique_ptr<GroundLock>      ground_lock_;
    std::unique_ptr<GateCenteringLock> gate_centering_lock_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vision_source_pub_;

    // ── Vision lock (koreksi posisi via marker ArUco kamera nadir) ───
    // Hanya dipakai untuk mengoreksi target N/E di dua fase "hold
    // posisi tetap" (HOVER dan dwell setelah waypoint REACHED) — TIDAK
    // menyentuh logic yaw-align/approach-velocity yang sudah stabil
    // (lihat BUGFIX_ALTITUDE_STUCK.md). vision_lock_enable_ adalah
    // kill-switch lapangan: false = perilaku identik sebelum fitur ini.
    bool   vision_lock_enable_     {true};
    double camera_mount_yaw_deg_   {0.0};
    double vision_max_correction_m_{0.4};
    double marker_center_tolerance_m_{0.10};
    double marker_search_timeout_s_{20.0};
    double marker_search_altitude_m_{0.0};
    int    marker_search_direction_{1};
    double marker_search_horizontal_m_{0.0};
    int    marker_search_horizontal_direction_{1};
    PositionNED latched_marker_target_{};
    bool marker_target_latched_{false};
    int    vision_engage_ticks_    {0};
    bool   vision_engaged_prev_    {false};

    // ── Ground lock (koreksi posisi via box YOLO, khusus WP1) ─────
    // Sama seperti vision_lock_enable_/camera_mount_yaw_deg_ tapi
    // untuk jalur YOLO — pakai kamera fisik yang sama dengan ArUco
    // (mount yaw sama, reuse camera_mount_yaw_deg_), tapi intrinsik
    // fokal panjang punya param sendiri karena dipakai proyeksi
    // ground-plane, bukan solvePnP. Kill-switch terpisah dari
    // vision_lock_enable_ supaya dua jalur bisa dites independen.
    // Dipakai state machine untuk centering WP1 sebelum gripper — lihat
    // "yolo_centering_wp" di runMission().
    bool   ground_lock_enable_     {true};
    double yolo_camera_fx_px_      {640.0};
    double yolo_camera_fy_px_      {640.0};

    // ── Gate centering (Livox MID360s, /livox/points) ────────────────
    // Kill-switch lapangan, default FALSE. Subscriber point cloud hanya
    // dibuat hanya bila parameter ini true. Karena default false, mode
    // takeoff/airborne_handoff lama tidak punya callback, copy cloud,
    // maupun koreksi Livox. Bila sengaja true pada misi normal, gate assist
    // opt-in bekerja sekali pada setiap leg maju.
    bool   gate_centering_enable_        {false};
    float  gate_centering_roi_forward_min_m_{1.0f};
    float  gate_centering_roi_forward_max_m_{10.0f};
    float  gate_centering_roi_lateral_m_    {3.0f};
    float  gate_centering_gate_width_m_     {1.5f};
    float  gate_centering_tolerance_m_      {0.2f};
    float  gate_centering_target_forward_distance_m_{1.75f};
    int    gate_centering_min_cluster_points_{5};
    GateCenteringLock::Result gate_centering_latest_{};
    double last_gate_sample_s_{-1.0};

    // Debounce/freeze untuk hasil GateCenteringLock — reuse VisionLock
    // (sudah teruji: debounce sample berturut-turut, clamp magnitude,
    // freeze saat sumber stale) alih-alih menulis logic baru. body_up
    // selalu 0 (Livox tidak punya offset vertikal yang relevan di sini),
    // body_right = gate_centering_latest_.lateral_error_m. camera_mount_yaw_deg
    // di-set 0 karena sensor sudah dikonfirmasi body-aligned (lihat
    // docs/LIVOX_MID360_INTEGRATION.md §4) — beda dari kamera yang punya
    // camera_mount_yaw_deg_ terpisah karena tidak diasumsikan body-aligned.
    std::unique_ptr<VisionLock> gate_centering_debounce_;

    // ── Gate pass mission (start_mode:=gate_pass) ─────────────────────
    // Lihat runGatePassMission(). Kill-switch ganda: hanya aktif kalau
    // start_mode_==GATE_PASS DAN gate_centering_enable_==true (dicek di
    // awal runGatePassMission(), gagal -> LAND_CMD, tidak pernah terbang
    // buta tanpa koreksi lateral).
    GatePassStage gate_pass_stage_ {GatePassStage::CENTER};
    int    gate_pass_centered_ticks_ {0};
    bool   gate_pass_engaged_prev_   {false};
    PositionNED gate_pass_center_target_ {};
    PositionNED gate_pass_advance_start_position_ {};
    double gate_pass_advance_start_yaw_ {0.0};
    rclcpp::Time gate_pass_stage_started_at_;

    float  gate_pass_forward_velocity_m_s_ {1.0f};
    float  gate_pass_proportional_gain_    {0.5f};
    float  gate_pass_max_lateral_velocity_m_s_{0.3f};
    float  gate_pass_distance_m_           {3.5f};
    int    gate_pass_required_centered_ticks_{10};
    double gate_pass_center_timeout_s_     {15.0};
    double gate_pass_advance_timeout_s_    {12.0};

    // Gate assist opsional di dalam RelativePath normal. Satu gate per leg:
    // detector menginterupsi APPROACH, CENTER, ADVANCE, lalu jalur nominal
    // digeser lateral ke center gate terbaru sebelum approach dilanjutkan.
    bool mission_gate_active_ {false};
    bool mission_gate_completed_for_wp_ {false};
    GatePassStage mission_gate_stage_ {GatePassStage::CENTER};
    PositionNED mission_gate_center_target_ {};
    PositionNED mission_gate_advance_start_ {};
    double mission_gate_heading_ {0.0};
    double mission_gate_stage_started_s_ {0.0};
    int mission_gate_centered_ticks_ {0};
    bool leg_bearing_override_valid_ {false};
    double leg_bearing_override_ {0.0};

    // Feedback centering ArUco (closed-loop, tidak pakai averaging).
    bool   marker_feedback_active_{false};
    bool   marker_feedback_locked_{false};
    int    marker_stable_frames_{0};
    bool   marker_latest_sample_available_{false};
    double marker_latest_offset_north_{0.0};
    double marker_latest_offset_east_{0.0};
    bool   marker_offset_filter_ready_{false};
    double last_marker_sample_s_{-1.0};
    PositionNED marker_center_target_{};
    ControlModule::MarkerCentersSample marker_centers_latest_{};
    bool   marker_centers_available_{false};
    double last_marker_centers_s_{-1.0};
    double marker_heading_best_error_rad_{0.0};
    double marker_heading_best_yaw_{0.0};
    double marker_heading_align_started_s_{0.0};
    int    marker_heading_aligned_ticks_{0};

    rclcpp::TimerBase::SharedPtr timer_;

    // ── Counter & fase ────────────────────────────────────────────
    int    counter_         {0};
    int    hover_counter_   {0};
    int    initial_altitude_stable_ticks_ {0};
    int    wp_hold_counter_ {0};
    // Khusus mode tanpa ArUco: saat horizontal sudah masuk radius akhir,
    // tahan satu anchor tetap sambil altitude menyusul. Setelah centering
    // dikomit, anchor kedua menjaga posisi selama urutan gripper.
    bool off_mode_final_hold_anchor_valid_ {false};
    PositionNED off_mode_final_hold_anchor_ {};
    bool off_mode_waypoint_committed_ {false};
    size_t current_wp_      {0};
    Phase  phase_           {Phase::INIT};
    WaypointPhase waypoint_phase_ {WaypointPhase::APPROACH};
    rclcpp::Time marker_search_started_at_;

    // Drop barang setelah WP1, begitu box YOLO/ground-lock sudah centered
    // (WP1 memakai YOLO, bukan ArUco — lihat "yolo_centering_wp" di
    // runMission()). WP2 tetap ArUco seperti waypoint biasa, tanpa gripper.
    bool gripper_drop_enable_ {true};
    std::string gripper_cmd_topic_ {"/gripper_cmd"};
    size_t gripper_drop_after_wp_ {0};
    int gripper_open_wait_ticks_ {15};
    int gripper_close_wait_ticks_ {15};
    int gripper_drop_counter_ {0};
    bool gripper_drop_completed_ {false};
    GripperDropState gripper_drop_state_ {GripperDropState::IDLE};

    // Gate "boleh mulai maju" per waypoint — drone berputar di tempat
    // (vx=vy=0, hanya yaw+vz jalan) sampai yaw benar-benar pas
    // (WaypointHandler::isYawAligned), baru boleh gerak horizontal.
    // Reset ke false tiap current_wp_ naik ke waypoint berikutnya.
    bool   yaw_locked_for_current_wp_ {false};
    int    yaw_aligned_ticks_ {0};
    PositionNED yaw_hold_position_ {};
    bool   yaw_hold_position_valid_ {false};
    double yaw_command_unwrapped_ {0.0};
    double yaw_target_unwrapped_ {0.0};
    double yaw_profile_rate_rad_s_ {0.0};
    double yaw_profile_direction_ {1.0};
    int    yaw_hold_settle_ticks_ {0};
    double yaw_hold_start_yaw_ {0.0};
    bool   post_yaw_correction_active_ {false};
    PositionNED post_yaw_correction_target_ {};
    int    post_yaw_correction_stable_ticks_ {0};
    double post_yaw_correction_started_s_ {0.0};

    // ── Konfigurasi ────────────────────────────────────────────────
    double origin_north_ {0.0};
    double origin_east_  {0.0};
    double origin_down_  {0.0};
    bool   got_origin_   {false};

    // Yaw drone (radian, konvensi NED PX4) saat origin dikunci — dipakai
    // untuk memutar waypoint RelativePath (yang dihitung relatif "arah
    // hidung drone saat itu") menjadi NED sungguhan sebelum dibandingkan
    // dengan vehicle_.position. Tanpa ini, forward()/turnRight() di
    // RelativePath sebenarnya bergerak relatif UTARA KOMPAS ASLI, bukan
    // relatif arah hidung drone — cuma benar kalau kebetulan drone
    // menghadap utara pas armed.
    double origin_yaw_ {0.0};

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

    // Titik horizontal yang dipakai sebagai hold selama fase takeoff
    // dan hover awal. Disimpan sekali saat takeoff mulai supaya drone
    // naik lurus di atas titik awal, walau ada noise/ekf drift.
    double takeoff_hold_north_ {0.0};
    double takeoff_hold_east_  {0.0};
    double takeoff_hold_yaw_   {0.0};

    StartMode start_mode_ {StartMode::TAKEOFF};
    std::string start_mode_param_ {"takeoff"};

    // Default false: launch hanya takeoff vertikal lalu hover permanen.
    // Set true lewat parameter launch bila rangkaian ArUco/misi sudah siap.
    bool start_mission_after_hover_ {false};

    // Heading misi default mengikuti yaw drone saat origin dikunci, sehingga
    // RelativePath forward() berarti "arah hidung drone". Untuk handoff dari
    // RC, operator bisa mengunci heading absolut atau memberi koreksi derajat
    // kecil jika yaw awal sengaja/terpaksa tidak persis lurus arena.
    bool   override_mission_heading_ {false};
    double mission_heading_deg_ {0.0};
    double mission_heading_correction_deg_ {0.0};
    bool   marker_heading_align_enable_ {true};
    int    marker_heading_back_id_ {0};
    int    marker_heading_front_id_ {1};
    double marker_heading_tolerance_deg_ {5.0};
    double marker_heading_timeout_s_ {8.0};
    double marker_heading_yaw_sign_ {1.0};

    // Tambahan command altitude ke arah atas (meter).
    // 0.0 = tidak ada kompensasi. Coba 0.3-0.5 hanya jika log raw PX4
    // membuktikan drone memang berhenti di bawah target relatif.
    double altitude_command_bias_m_ {0.0};
};

}  // namespace px4
