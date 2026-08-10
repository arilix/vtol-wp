#include "utils/mission_manager.h"

#include <algorithm>
#include <cmath>
#include <string>

using namespace std::chrono_literals;

namespace px4
{

namespace
{
// ==================================================================
// WAYPOINT MISI (NED)
// Format internal: North [m], East [m], Down [m] (negatif = naik).
//
// Offset diukur relatif terhadap POSISI HOVER (titik & altitude saat
// drone selesai takeoff dan stabil) — origin_north_/east_/down_ dikunci
// dari posisi drone sebelum arm, yang sama dengan posisi horizontal
// hover karena takeoff murni naik vertikal di tempat.
//
// Ganti isi defaultMissionWaypoints() dengan offset NED arena kamu, ATAU
// pakai RelativePath di bawah supaya tidak perlu tahu utara/timur sama
// sekali — cukup "maju sekian meter", "belok kanan/kiri sekian derajat".
// ==================================================================
constexpr double PI = 3.14159265358979323846;

double degToRad(double deg)
{
    return deg * PI / 180.0;
}

double radToDeg(double rad)
{
    return rad * 180.0 / PI;
}

double nedBearingDeg(double north_m, double east_m)
{
    return radToDeg(std::atan2(east_m, north_m));
}

const char * phaseLabel(Phase phase)
{
    switch (phase) {
        case Phase::INIT: return "INIT";
        case Phase::WAIT_ARM: return "WAIT_ARM";
        case Phase::TAKEOFF: return "TAKEOFF";
        case Phase::HOVER: return "HOVER";
        case Phase::PILOT_ARUCO_SEARCH: return "PILOT_ARUCO_SEARCH";
        case Phase::TAKEOFF_MARKER: return "ARUCO_LOCK";
        case Phase::MISSION: return "MISSION";
        case Phase::GATE_PASS: return "GATE_TEST";
        case Phase::LAND_CMD: return "LAND_CMD";
        case Phase::WAIT_DISARM: return "WAIT_DISARM";
    }
    return "UNKNOWN";
}

const char * gateStatusLabel(bool active, GatePassStage stage)
{
    if (!active) return "IDLE";
    return stage == GatePassStage::CENTER ? "CENTER" : "ADVANCE";
}

// Waypoint dari RelativePath dihitung dalam "frame lokal drone" (heading
// builder 0 = arah hidung drone saat hover, BUKAN utara asli). Putar ke
// NED sungguhan pakai yaw drone yang sebenarnya saat origin dikunci,
// supaya forward()/turnRight() konsisten dengan hidung drone di lapangan.
Waypoint rotateToTrueNed(const Waypoint & wp, double origin_yaw_rad)
{
    const double c = std::cos(origin_yaw_rad);
    const double s = std::sin(origin_yaw_rad);
    return {
        wp.n * c - wp.e * s,
        wp.n * s + wp.e * c,
        wp.d
    };
}

// ==================================================================
// RelativePath
//
// Builder untuk mendefinisikan waypoint sebagai urutan gerakan relatif
// terhadap arah hadap drone saat hover (bukan arah mata angin asli),
// supaya operator tidak perlu tahu kompas sama sekali.
//
// heading 0 = "arah depan" drone saat mulai (heading_rad_ di sini
// murni variabel internal builder, TIDAK ada hubungannya dengan yaw
// PX4 — dia cuma menentukan sumbu N/E mana yang dipakai tiap forward()
// dipanggil). turnRight/turnLeft memutar sumbu itu untuk gerakan
// forward() berikutnya, tanpa membuat waypoint baru (drone tidak
// diperintah berputar di tempat sebelum bergerak — ini murni matematika
// offset, bukan perintah yaw).
// ==================================================================
class RelativePath
{
public:
    explicit RelativePath(double start_altitude_agl_m)
    : d_(-start_altitude_agl_m)
    {}

    // Maju sejauh dist_m searah heading builder saat ini (mundur:
    // dist_m negatif). Menambahkan satu waypoint baru.
    RelativePath & forward(double dist_m)
    {
        n_ += std::cos(heading_rad_) * dist_m;
        e_ += std::sin(heading_rad_) * dist_m;
        waypoints_.push_back({n_, e_, d_});
        return *this;
    }

    // Geser tegak lurus arah hadap TANPA mengubah heading builder
    // (mis. strafe kanan/kiri sambil tetap "menghadap" arah semula).
    // Menambahkan satu waypoint baru.
    RelativePath & strafeRight(double dist_m)
    {
        const double right_rad = heading_rad_ + degToRad(90.0);
        n_ += std::cos(right_rad) * dist_m;
        e_ += std::sin(right_rad) * dist_m;
        waypoints_.push_back({n_, e_, d_});
        return *this;
    }

    RelativePath & strafeLeft(double dist_m)
    {
        return strafeRight(-dist_m);
    }

    // Ubah arah "depan" untuk forward()/strafeRight() berikutnya.
    // Tidak menambah waypoint (murni belok logika, bukan gerak nyata).
    RelativePath & turnRight(double deg)
    {
        heading_rad_ += degToRad(deg);
        return *this;
    }

    RelativePath & turnLeft(double deg)
    {
        heading_rad_ -= degToRad(deg);
        return *this;
    }

    // Naik/turun altitude di posisi N/E sekarang. Menambahkan satu
    // waypoint baru (climb/descend murni, WaypointHandler otomatis
    // mendeteksi ini sebagai "pure altitude" kalau jarak horizontalnya 0).
    RelativePath & climbTo(double altitude_agl_m)
    {
        d_ = -altitude_agl_m;
        waypoints_.push_back({n_, e_, d_});
        return *this;
    }

    std::vector<Waypoint> build() const { return waypoints_; }

private:
    double heading_rad_ {0.0};
    double n_ {0.0};
    double e_ {0.0};
    double d_;
    std::vector<Waypoint> waypoints_;
};

std::vector<Waypoint> defaultMissionWaypoints()
{
    // Contoh: altitude awal 1.0m AGL, maju 3m, belok kanan 90 derajat,
    // maju 5m lagi (hasilnya identik dengan waypoint N/E manual
    // sebelumnya: {3,0,-1} lalu {3,5,-1}).
    RelativePath path(/*start_altitude_agl_m=*/1.00);
    path.forward(4.9)
        .turnLeft(91.0)
        .forward(6.0);
    return path.build();

    // Alternatif: isi manual langsung dalam NED kalau lebih nyaman.
    // return {
    //     // Format: { north_m, east_m, down_m }
    //     { 3.0, 0.0, -1.0 },
    //     { 3.0, 5.0, -1.0 },
    // };
}
}  // namespace

// ==================================================================
// Constructor
// ==================================================================

MissionManager::MissionManager()
: rclcpp::Node("mission_manager")
{
    // Inisialisasi eksplisit pakai clock node (RCL_ROS_TIME) — kalau
    // dibiarkan default-construct, clock_type-nya beda dengan this->now()
    // dan rclcpp bisa throw saat dikurangi di isPositionStale().
    last_position_time_ = this->now();
    last_range_time_ = this->now();

    altitude_command_bias_m_ = this->declare_parameter<double>(
        "altitude_command_bias_m", 0.0);
    use_lidar_altitude_ = this->declare_parameter<bool>(
        "use_lidar_altitude", true);
    start_mode_param_ = this->declare_parameter<std::string>(
        "start_mode", "takeoff");
    if (start_mode_param_ == "airborne_handoff") {
        start_mode_ = StartMode::AIRBORNE_HANDOFF;
    } else if (start_mode_param_ == "takeoff") {
        start_mode_ = StartMode::TAKEOFF;
    } else if (start_mode_param_ == "gate_pass") {
        start_mode_ = StartMode::GATE_PASS;
    } else {
        RCLCPP_WARN(this->get_logger(),
            "start_mode='%s' tidak dikenal; pakai default 'takeoff'. "
            "Pilihan valid: takeoff, airborne_handoff, gate_pass.",
            start_mode_param_.c_str());
        start_mode_param_ = "takeoff";
        start_mode_ = StartMode::TAKEOFF;
    }
    start_mission_after_hover_ = this->declare_parameter<bool>(
        "start_mission_after_hover", false);
    override_mission_heading_ = this->declare_parameter<bool>(
        "override_mission_heading", false);
    mission_heading_deg_ = this->declare_parameter<double>(
        "mission_heading_deg", 0.0);
    mission_heading_correction_deg_ = this->declare_parameter<double>(
        "mission_heading_correction_deg", 0.0);
    marker_heading_align_enable_ = this->declare_parameter<bool>(
        "marker_heading_align_enable", true);
    marker_heading_back_id_ = this->declare_parameter<int>(
        "marker_heading_back_id", 0);
    marker_heading_front_id_ = this->declare_parameter<int>(
        "marker_heading_front_id", 1);
    marker_heading_tolerance_deg_ = this->declare_parameter<double>(
        "marker_heading_tolerance_deg", 5.0);
    marker_heading_timeout_s_ = this->declare_parameter<double>(
        "marker_heading_timeout_s", 8.0);
    marker_heading_yaw_sign_ = this->declare_parameter<double>(
        "marker_heading_yaw_sign", 1.0);

    vision_lock_enable_ = this->declare_parameter<bool>(
        "vision_lock_enable", true);
    camera_mount_yaw_deg_ = this->declare_parameter<double>(
        "camera_mount_yaw_deg", 0.0);
    vision_max_correction_m_ = this->declare_parameter<double>(
        "vision_max_correction_m", 0.4);
    marker_center_tolerance_m_ = this->declare_parameter<double>(
        "marker_center_tolerance_m", 0.10);
    marker_search_timeout_s_ = this->declare_parameter<double>(
        "marker_search_timeout_s", 20.0);
    ground_lock_enable_ = this->declare_parameter<bool>(
        "ground_lock_enable", true);
    yolo_camera_fx_px_ = this->declare_parameter<double>(
        "yolo_camera_fx_px", 640.0);
    yolo_camera_fy_px_ = this->declare_parameter<double>(
        "yolo_camera_fy_px", 640.0);
    yolo_ui_center_tolerance_px_ = this->declare_parameter<double>(
        "yolo_ui_center_tolerance_px", 50.0);
    gripper_drop_enable_ = this->declare_parameter<bool>(
        "gripper_drop_enable", true);
    gripper_cmd_topic_ = this->declare_parameter<std::string>(
        "gripper_cmd_topic", "/gripper_cmd");
    gripper_open_wait_ticks_ = this->declare_parameter<int>(
        "gripper_open_wait_ticks", 15);
    gripper_close_wait_ticks_ = this->declare_parameter<int>(
        "gripper_close_wait_ticks", 15);

    gate_centering_enable_ = this->declare_parameter<bool>(
        "gate_centering_enable", false);
    gate_centering_roi_forward_min_m_ = this->declare_parameter<double>(
        "gate_centering.detection_range_min", 1.0);
    gate_centering_roi_forward_max_m_ = this->declare_parameter<double>(
        "gate_centering.detection_range_max", 10.0);
    gate_centering_roi_lateral_m_ = this->declare_parameter<double>(
        "gate_centering.roi_lateral", 3.0);
    gate_centering_gate_width_m_ = this->declare_parameter<double>(
        "gate_centering.gate_width", 1.5);
    gate_centering_tolerance_m_ = this->declare_parameter<double>(
        "gate_centering.centering_tolerance", 0.2);
    gate_centering_target_forward_distance_m_ = this->declare_parameter<double>(
        "gate_centering.target_gate_distance", 1.75);
    gate_centering_min_cluster_points_ = this->declare_parameter<int>(
        "gate_centering.min_cluster_points", 5);
    gate_centering_heading_tolerance_rad_ = degToRad(this->declare_parameter<double>(
        "gate_centering.heading_tolerance_deg", 5.0));
    gate_centering_max_heading_correction_rad_ = degToRad(
        this->declare_parameter<double>(
            "gate_centering.max_heading_correction_deg", 12.0));
    mission_gate_required_centered_ticks_ = this->declare_parameter<int>(
        "mission_gate.required_centered_ticks", 5);
    mission_gate_search_distance_m_ = this->declare_parameter<double>(
        "mission_gate.search_distance_m", 0.40);
    mission_gate_search_speed_m_s_ = this->declare_parameter<double>(
        "mission_gate.search_speed_m_s", 0.25);

    gate_pass_forward_velocity_m_s_ = this->declare_parameter<double>(
        "gate_pass.forward_velocity_m_s", 1.0);
    gate_pass_proportional_gain_ = this->declare_parameter<double>(
        "gate_pass.proportional_gain", 0.5);
    gate_pass_max_lateral_velocity_m_s_ = this->declare_parameter<double>(
        "gate_pass.max_lateral_velocity_m_s", 0.3);
    gate_pass_distance_m_ = this->declare_parameter<double>(
        "gate_pass.pass_distance_m", 3.5);
    gate_pass_required_centered_ticks_ = this->declare_parameter<int>(
        "gate_pass.required_centered_ticks", 10);
    gate_pass_center_timeout_s_ = this->declare_parameter<double>(
        "gate_pass.center_timeout_s", 15.0);
    gate_pass_advance_timeout_s_ = this->declare_parameter<double>(
        "gate_pass.advance_timeout_s", 12.0);

    waypoints_ = std::make_unique<WaypointHandler>(std::vector<Waypoint>{});
    // Isolasi keras: tanpa gate_centering_enable tidak ada subscription
    // ataupun copy cloud. Bila true, Livox boleh dipakai oleh gate_pass
    // standalone atau gate assist pada setiap leg misi normal.
    const bool livox_mode_active = gate_centering_enable_;
    control_   = std::make_unique<ControlModule>(this, livox_mode_active);
    gripper_cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
        gripper_cmd_topic_, 10);
    vision_source_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/mission/vision_source_active", 10);

    VisionLock::Config vision_cfg;
    vision_cfg.camera_mount_yaw_deg    = camera_mount_yaw_deg_;
    vision_cfg.max_correction_m        = vision_max_correction_m_;
    // Satu deteksi valid langsung menghentikan sweep. Setelah itu kontrol
    // bekerja iteratif: hitung offset -> maju satu langkah kecil -> hold
    // bila marker hilang -> hitung ulang saat marker terlihat kembali.
    vision_cfg.min_consecutive_samples = 1;
    vision_lock_ = std::make_unique<VisionLock>(vision_cfg);

    GroundLock::Config ground_cfg;
    ground_cfg.camera_fx_px = yolo_camera_fx_px_;
    ground_cfg.camera_fy_px = yolo_camera_fy_px_;
    // Kamera fisik sama dengan ArUco (keputusan desain "gantian") ->
    // mount yaw sama, jadi reuse camera_mount_yaw_deg_ apa adanya.
    ground_cfg.vision_cfg.camera_mount_yaw_deg    = camera_mount_yaw_deg_;
    ground_cfg.vision_cfg.max_correction_m        = vision_max_correction_m_;
    ground_cfg.vision_cfg.min_consecutive_samples = 1;
    ground_lock_ = std::make_unique<GroundLock>(ground_cfg);

    GateCenteringLock::Config gate_centering_cfg;
    gate_centering_cfg.roi_forward_min_m = gate_centering_roi_forward_min_m_;
    gate_centering_cfg.roi_forward_max_m = gate_centering_roi_forward_max_m_;
    gate_centering_cfg.roi_lateral_m = gate_centering_roi_lateral_m_;
    gate_centering_cfg.gate_width_m = gate_centering_gate_width_m_;
    gate_centering_cfg.centering_tolerance_m = gate_centering_tolerance_m_;
    gate_centering_cfg.target_forward_distance_m = gate_centering_target_forward_distance_m_;
    gate_centering_cfg.min_cluster_points = gate_centering_min_cluster_points_;
    gate_centering_lock_ = std::make_unique<GateCenteringLock>(gate_centering_cfg);

    VisionLock::Config gate_centering_debounce_cfg;
    // Sensor sudah dikonfirmasi body-aligned (lihat
    // docs/LIVOX_MID360_INTEGRATION.md §4) -> tidak perlu mount-yaw.
    gate_centering_debounce_cfg.camera_mount_yaw_deg = 0.0;
    gate_centering_debounce_cfg.max_correction_m =
        gate_centering_gate_width_m_ * 0.5 + 0.5;
    gate_centering_debounce_cfg.min_consecutive_samples = 1;
    gate_centering_debounce_ = std::make_unique<VisionLock>(gate_centering_debounce_cfg);

    // Pasang callback supaya MissionManager dapat update posisi/status
    // tanpa ControlModule perlu tahu apa pun soal logic misi.
    control_->setPositionCallback(
        [this](const ControlModule::PositionSample & s) {
            onPositionUpdate(s);
        });

    control_->setRangeCallback(
        [this](const ControlModule::RangeSample & s) {
            onRangeUpdate(s);
        });

    control_->setStatusCallback(
        [this](uint8_t arming_state, uint8_t nav_state) {
            onStatusUpdate(arming_state, nav_state);
        });

    control_->setMarkerPoseCallback(
        [this](const ControlModule::MarkerPoseSample & s) {
            onMarkerPoseUpdate(s);
        });

    control_->setMarkerCentersCallback(
        [this](const ControlModule::MarkerCentersSample & s) {
            onMarkerCentersUpdate(s);
        });

    control_->setTargetCenterCallback(
        [this](const ControlModule::TargetCenterSample & s) {
            onTargetCenterUpdate(s);
        });

    if (livox_mode_active) {
        control_->setLivoxCallback(
            [this](const ControlModule::LivoxSample & s) {
                onLivoxUpdate(s);
            });
    }

    timer_ = this->create_wall_timer(100ms, [this]() { loop(); });

    RCLCPP_INFO(this->get_logger(), "=== NODE SIAP ===");
    RCLCPP_INFO(this->get_logger(),
        "Altitude command bias: %.2fm", altitude_command_bias_m_);
    RCLCPP_INFO(this->get_logger(),
        "Use lidar altitude: %s", use_lidar_altitude_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(),
        "Start mode: %s", start_mode_param_.c_str());
    RCLCPP_INFO(this->get_logger(),
        "Start mission after hover: %s",
        start_mission_after_hover_ ? "true" : "false (takeoff + hover only)");
    RCLCPP_INFO(this->get_logger(),
        "Mission heading: %s %.1fdeg, correction %.1fdeg",
        override_mission_heading_ ? "override" : "origin/current",
        mission_heading_deg_, mission_heading_correction_deg_);
    RCLCPP_INFO(this->get_logger(),
        "Marker heading align: %s back_id=%d front_id=%d tol=%.1fdeg timeout=%.1fs sign=%.1f",
        marker_heading_align_enable_ ? "enabled" : "disabled",
        marker_heading_back_id_, marker_heading_front_id_,
        marker_heading_tolerance_deg_, marker_heading_timeout_s_,
        marker_heading_yaw_sign_);
    RCLCPP_INFO(this->get_logger(),
        "Vision lock: %s (camera_mount_yaw=%.1fdeg, max_correction=%.2fm)",
        vision_lock_enable_ ? "enabled" : "disabled",
        camera_mount_yaw_deg_, vision_max_correction_m_);
    RCLCPP_INFO(this->get_logger(),
        "Ground lock (YOLO): %s (fx=%.1fpx, fy=%.1fpx, ui_tol=%.0fpx) — dipakai state machine untuk centering WP1 sebelum gripper",
        ground_lock_enable_ ? "enabled" : "disabled",
        yolo_camera_fx_px_, yolo_camera_fy_px_, yolo_ui_center_tolerance_px_);
    RCLCPP_INFO(this->get_logger(),
        "ArUco wajib di setiap WP: center <= %.2fm, warning timeout %.1fs",
        marker_center_tolerance_m_, marker_search_timeout_s_);
    RCLCPP_INFO(this->get_logger(),
        "Gripper drop: %s topic=%s open_wait=%.1fs close_wait=%.1fs",
        gripper_drop_enable_ ? "enabled" : "disabled",
        gripper_cmd_topic_.c_str(),
        gripper_open_wait_ticks_ * 0.1,
        gripper_close_wait_ticks_ * 0.1);
    RCLCPP_INFO(this->get_logger(),
        "Waypoint misi NED, relatif terhadap posisi hover.");
}

void MissionManager::requestGracefulLand()
{
    if (!control_) return;
    control_->publishHeartbeat(true);
    control_->sendLandCommand();
    phase_ = Phase::WAIT_DISARM;
}

// ==================================================================
// Callback dari ControlModule
// ==================================================================

namespace
{
// Toleransi & durasi stabilisasi origin horizontal. Altitude utama pakai lidar,
// jadi noise z PX4 tidak boleh memblokir origin lock.
constexpr double ORIGIN_STABLE_TOL_M = 0.20;   // meter
constexpr double ORIGIN_STABLE_DUR_S = 0.5;    // detik
}  // namespace

void MissionManager::onPositionUpdate(const ControlModule::PositionSample & s)
{
    const double x = s.x, y = s.y, z = s.z, yaw = s.yaw;

    last_position_time_ = this->now();
    ++position_update_count_;
    if (!got_raw_position_) {
        got_raw_position_ = true;
        RCLCPP_INFO(this->get_logger(),
            "Sample posisi PX4 pertama diterima: x=%.2f y=%.2f z=%.2f yaw=%.1fdeg",
            x, y, z, radToDeg(yaw));
    }

    if (!got_origin_) {
        const auto now = this->now();

        if (!has_candidate_) {
            origin_candidate_ = {x, y, z};
            candidate_since_  = now;
            has_candidate_    = true;
        } else {
            const double dn = x - origin_candidate_.north;
            const double de = y - origin_candidate_.east;

            if (std::abs(dn) > ORIGIN_STABLE_TOL_M ||
                std::abs(de) > ORIGIN_STABLE_TOL_M)
            {
                // Estimasi masih bergeser (EKF belum konvergen) —
                // reset jendela stabilisasi, jangan kunci origin dulu.
                origin_candidate_ = {x, y, z};
                candidate_since_  = now;
            } else if ((now - candidate_since_).seconds() >= ORIGIN_STABLE_DUR_S) {
                origin_north_ = x;
                origin_east_  = y;
                origin_down_  = z;
                origin_yaw_   = yaw;
                got_origin_   = true;

                // Baseline reset counter mulai dari titik ini — reset
                // SEBELUM origin terkunci tidak relevan (origin toh
                // masih bergerak-gerak saat itu).
                last_xy_reset_counter_      = s.xy_reset_counter;
                last_z_reset_counter_       = s.z_reset_counter;
                last_heading_reset_counter_ = s.heading_reset_counter;
                have_reset_counters_        = true;

                vehicle_.position.north = 0.0;
                vehicle_.position.east  = 0.0;
                vehicle_.position.down  = 0.0;
                vehicle_.yaw = yaw;
                vehicle_.got_position = true;

                RCLCPP_INFO(this->get_logger(),
                    "PX4 local origin captured (stabil %.1fs, toleransi %.2fm): "
                    "x=%.2f y=%.2f z=%.2f yaw=%.1fdeg",
                    ORIGIN_STABLE_DUR_S, ORIGIN_STABLE_TOL_M,
                    origin_north_, origin_east_, origin_down_, radToDeg(origin_yaw_));
                RCLCPP_INFO(this->get_logger(),
                    "Mission frame is now relative to this start position.");
            }
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Sample posisi PX4 masuk (%u), menunggu origin stabil: "
            "raw x=%.2f y=%.2f z=%.2f",
            position_update_count_, x, y, z);
        return;
    }

    // ── Kompensasi reset EKF ──────────────────────────────────────
    // Kalau PX4 melakukan reset internal (xy/z/heading_reset_counter
    // berubah), geser origin sebesar delta yang sama supaya posisi
    // RELATIF terhadap origin tetap kontinu — bukan melompat seolah
    // drone berpindah tempat sungguhan. Ini yang selama ini hilang dan
    // menyebabkan drone "mengejar" lompatan EKF sejauh beberapa meter.
    if (have_reset_counters_) {
        if (s.xy_reset_counter != last_xy_reset_counter_) {
            origin_north_ += s.delta_x;
            origin_east_  += s.delta_y;
            RCLCPP_WARN(this->get_logger(),
                "EKF XY RESET terdeteksi (counter %u->%u, delta=%.2f,%.2f) "
                "— origin disesuaikan supaya posisi relatif tetap kontinu.",
                last_xy_reset_counter_, s.xy_reset_counter, s.delta_x, s.delta_y);
            last_xy_reset_counter_ = s.xy_reset_counter;
        }
        if (s.z_reset_counter != last_z_reset_counter_) {
            origin_down_ += s.delta_z;
            RCLCPP_WARN(this->get_logger(),
                "EKF Z RESET terdeteksi (counter %u->%u, delta=%.2f) "
                "— origin altitude disesuaikan.",
                last_z_reset_counter_, s.z_reset_counter, s.delta_z);
            last_z_reset_counter_ = s.z_reset_counter;
        }
        if (s.heading_reset_counter != last_heading_reset_counter_) {
            RCLCPP_WARN(this->get_logger(),
                "EKF HEADING RESET terdeteksi (counter %u->%u, delta=%.3f rad) "
                "— yaw akan menyesuaikan otomatis tick berikutnya.",
                last_heading_reset_counter_, s.heading_reset_counter, s.delta_heading);
            last_heading_reset_counter_ = s.heading_reset_counter;
        }
    }

    vehicle_.position.north = x - origin_north_;
    vehicle_.position.east  = y - origin_east_;
    vehicle_.position.down  = z - origin_down_;
    vehicle_.velocity.north = s.vx;
    vehicle_.velocity.east  = s.vy;
    vehicle_.velocity.down  = s.vz;

    vehicle_.yaw = yaw;

    vehicle_.got_position = true;
}

void MissionManager::onRangeUpdate(const ControlModule::RangeSample & s)
{
    if (!s.valid) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "TFmini range tidak valid: %.2fm (batas %.2f..%.2fm)",
            s.range_m, s.min_range_m, s.max_range_m);
        return;
    }

    lidar_altitude_m_ = s.range_m;
    last_range_time_ = this->now();
    got_lidar_altitude_ = true;
}

bool MissionManager::ensureMissionWaypointsReady()
{
    if (mission_waypoints_ready_) {
        return true;
    }

    // RelativePath menghasilkan offset dalam frame lokal drone (heading
    // builder 0 = arah hidung drone saat hover). Putar dengan origin_yaw_
    // (yaw sungguhan yang tercatat saat origin dikunci) supaya hasilnya
    // NED sungguhan yang bisa dibandingkan langsung dengan vehicle_.position.
    auto local_waypoints = defaultMissionWaypoints();
    std::vector<Waypoint> ned_waypoints;
    ned_waypoints.reserve(local_waypoints.size());
    const double reference_yaw = missionReferenceYaw();
    for (const auto & wp : local_waypoints) {
        ned_waypoints.push_back(rotateToTrueNed(wp, reference_yaw));
    }

    RCLCPP_INFO(this->get_logger(),
        "=== WAYPOINT MISI (relatif heading misi %.1fdeg; yaw origin=%.1fdeg) ===",
        radToDeg(reference_yaw), radToDeg(origin_yaw_));
    for (size_t i = 0; i < ned_waypoints.size(); ++i) {
        const auto & wp = ned_waypoints[i];

        RCLCPP_INFO(this->get_logger(),
            "WP%zu N=%.3fm E=%.3fm Alt=%.2fm",
            i + 1, wp.n, wp.e, -wp.d);

        if (i > 0) {
            const auto & prev = ned_waypoints[i - 1];
            const double dn = wp.n - prev.n;
            const double de = wp.e - prev.e;
            RCLCPP_INFO(this->get_logger(),
                "  Leg WP%zu->WP%zu: dN=%.3fm dE=%.3fm Dist=%.3fm Bearing=%.1fdeg",
                i, i + 1, dn, de, std::hypot(dn, de), nedBearingDeg(dn, de));
        }
    }

    waypoints_ = std::make_unique<WaypointHandler>(std::move(ned_waypoints));
    mission_waypoints_ready_ = true;
    return true;
}

void MissionManager::rebaseMissionOriginToCurrentPosition()
{
    // Waypoint RelativePath menyatakan jarak dari posisi saat misi mulai,
    // bukan dari posisi sebelum takeoff. Pencarian dan centering ArUco bisa
    // menggeser drone beberapa puluh sentimeter dari origin awal. Jika origin
    // tidak dipindah di sini, WP1 tetap berada 5 m dari titik sebelum takeoff,
    // sehingga jarak maju aktual setelah ArUco berubah-ubah dan bisa kurang.
    const double shift_n = vehicle_.position.north;
    const double shift_e = vehicle_.position.east;

    origin_north_ += shift_n;
    origin_east_  += shift_e;
    vehicle_.position.north = 0.0;
    vehicle_.position.east  = 0.0;

    yaw_locked_for_current_wp_ = false;
    yaw_aligned_ticks_ = 0;
    yaw_hold_position_valid_ = false;
    yaw_hold_settle_ticks_ = 0;
    altitude_hold_after_yaw_ = false;
    post_yaw_correction_active_ = false;
    post_yaw_return_to_drop_anchor_ = false;
    post_yaw_correction_stable_ticks_ = 0;
    waypoints_->resetYawSmoothing(vehicle_.yaw);

    RCLCPP_INFO(this->get_logger(),
        "Mission origin dikunci ulang setelah ArUco: geser N=%.3fm E=%.3fm. "
        "WP1 sekarang diukur dari posisi aktual ini.",
        shift_n, shift_e);
}

void MissionManager::applyVisionLock(
    double base_north, double base_east,
    double & out_north, double & out_east)
{
    // Ramp-in 1.5s (15 tick @ 10Hz) saat lock baru engage — supaya
    // setpoint tidak melompat mendadak ke target vision (step input ke
    // position controller PX4), konsisten dengan filosofi transisi
    // smooth yang sudah ada di WaypointHandler (yawAlignmentFactor dll).
    constexpr int VISION_RAMP_TICKS = 15;

    if (!vision_lock_enable_) {
        out_north = base_north;
        out_east  = base_east;
        return;
    }

    const double now_s   = this->now().seconds();
    const bool   engaged = vision_lock_->isEngaged(now_s);

    if (!engaged) {
        if (vision_engaged_prev_) {
            RCLCPP_WARN(this->get_logger(),
                "VISION LOCK LOST — marker hilang/stale, kembali ke target NED.");
        }
        vision_engaged_prev_ = false;
        vision_engage_ticks_ = 0;
        out_north = base_north;
        out_east  = base_east;
        return;
    }

    if (!vision_engaged_prev_) {
        RCLCPP_INFO(this->get_logger(), "VISION LOCK ENGAGED — mengoreksi target ke marker.");
    }
    vision_engaged_prev_ = true;
    if (vision_engage_ticks_ < VISION_RAMP_TICKS) {
        ++vision_engage_ticks_;
    }
    if (vision_lock_->lastSampleClamped()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Vision lock: koreksi terpotong ke batas %.2fm — cek kalibrasi/deteksi.",
            vision_max_correction_m_);
    }

    const double blend  = static_cast<double>(vision_engage_ticks_) / VISION_RAMP_TICKS;
    const auto   target = vision_lock_->lockedTarget();

    out_north = base_north * (1.0 - blend) + target.north * blend;
    out_east  = base_east  * (1.0 - blend) + target.east  * blend;
}

void MissionManager::resetWaypointVisionState()
{
    wp_hold_counter_ = 0;
    off_mode_final_hold_anchor_valid_ = false;
    off_mode_waypoint_committed_ = false;
    vision_engage_ticks_ = 0;
    vision_engaged_prev_ = false;
    yaw_locked_for_current_wp_ = false;
    yaw_aligned_ticks_ = 0;
    yaw_hold_position_valid_ = false;
    yaw_hold_settle_ticks_ = 0;
    altitude_hold_after_yaw_ = false;
    post_yaw_correction_active_ = false;
    post_yaw_correction_stable_ticks_ = 0;
    if (waypoints_) {
        waypoints_->resetYawSmoothing(vehicle_.yaw);
    }
    waypoint_phase_ = WaypointPhase::APPROACH;
    marker_search_altitude_m_ = 0.0;
    marker_search_direction_ = 1;
    marker_search_horizontal_m_ = 0.0;
    marker_search_horizontal_direction_ = 1;
    marker_target_latched_ = false;
    marker_feedback_active_ = false;
    marker_feedback_locked_ = false;
    marker_stable_frames_ = 0;
    marker_latest_sample_available_ = false;
    marker_latest_offset_north_ = 0.0;
    marker_latest_offset_east_ = 0.0;
    marker_offset_filter_ready_ = false;
    last_marker_sample_s_ = -1.0;
    marker_latest_tf_yaw_available_ = false;
    marker_tf_yaw_filter_ready_ = false;
    last_marker_tf_yaw_s_ = -1.0;
    initial_centered_samples_ = 0;
    initial_position_locked_ = false;
    initial_position_locked_s_ = -1.0;
    initial_tf_correction_used_ = false;
    initial_tf_fallback_used_ = false;
    marker_center_target_ = {};
    yolo_center_best_offset_m_ = std::numeric_limits<double>::infinity();
    yolo_center_diverging_ticks_ = 0;
    yolo_correction_sign_ = 1.0;
    yolo_direction_reversed_ = false;
    yolo_ui_centered_ = false;
    yolo_ui_center_error_x_px_ = 0.0;
    yolo_ui_center_error_y_px_ = 0.0;
    last_yolo_ui_center_s_ = -1.0;
    yolo_center_stage_ = YoloCenterStage::SAMPLE;
    yolo_center_anchor_ = {};
    yolo_step_target_ = {};
    yolo_step_started_s_ = 0.0;
    yolo_accept_sample_after_s_ = 0.0;
    yolo_step_stable_ticks_ = 0;
    yolo_step_count_ = 0;
    yolo_approach_brake_active_ = false;
    yolo_approach_brake_anchor_ = {};
    yolo_approach_interlock_consumed_ = false;
    forward_drift_recovery_active_ = false;
    forward_drift_recovery_stable_ticks_ = 0;
    forward_drift_recovery_target_ = {};
    initial_marker_center_available_ = false;
    initial_marker_center_new_ = false;
    last_initial_marker_center_s_ = -1.0;
    marker_heading_aligned_ticks_ = 0;
    marker_heading_best_error_rad_ = 0.0;
    marker_heading_best_yaw_ = vehicle_.yaw;
    marker_heading_align_started_s_ = 0.0;
    mission_gate_active_ = false;
    mission_gate_completed_for_wp_ = false;
    mission_gate_stage_ = GatePassStage::CENTER;
    mission_gate_centered_ticks_ = 0;
    mission_gate_advance_target_m_ = 0.0;
    mission_gate_heading_target_valid_ = false;
    mission_gate_center_lock_captured_ = false;
    mission_gate_entry_heading_ = vehicle_.yaw;
    mission_gate_search_anchor_valid_ = false;
    mission_gate_search_anchor_ = {};
    leg_bearing_override_valid_ = false;
}

void MissionManager::commitCurrentWaypointAnchor()
{
    if (!waypoints_ || current_wp_ >= waypoints_->size()) {
        return;
    }

    // ArUco, YOLO, dan gate dapat menggeser drone dari koordinat waypoint
    // nominal. RelativePath menyatakan gerakan berantai ("maju sekian,
    // yaw sekian, maju lagi"), jadi seluruh waypoint sesudah titik ini
    // harus ikut digeser dengan delta yang sama. Dengan begitu:
    //   - waypoint sekarang tepat jatuh di posisi centering aktual;
    //   - panjang dan bearing leg berikutnya tetap persis sesuai rencana;
    //   - drone tidak mencoba kembali diagonal ke garis nominal lama.
    const auto & nominal = waypoints_->at(current_wp_);
    const double delta_n = vehicle_.position.north - nominal.n;
    const double delta_e = vehicle_.position.east - nominal.e;
    waypoints_->translateWaypointsFrom(current_wp_, delta_n, delta_e);

    RCLCPP_INFO(this->get_logger(),
        "[%s] ANCHOR COMMIT: posisi aktual menjadi origin leg berikutnya "
        "(shift path dN=%.3fm dE=%.3fm).",
        waypoints_->labelAt(current_wp_).c_str(), delta_n, delta_e);
}

void MissionManager::applyLatchedMarkerTarget(
    double base_north, double base_east,
    double & out_north, double & out_east)
{
    // Target marker dibekukan dari sampel deteksi pertama. Ramp 1.5 detik
    // mencegah setpoint melompat, tetapi noise frame berikutnya tidak lagi
    // menggeser target sehingga centering tidak menggertak.
    constexpr int LATCH_RAMP_TICKS = 15;
    if (!marker_target_latched_) {
        out_north = base_north;
        out_east = base_east;
        return;
    }
    if (vision_engage_ticks_ < LATCH_RAMP_TICKS) {
        ++vision_engage_ticks_;
    }
    const double blend =
        static_cast<double>(vision_engage_ticks_) / LATCH_RAMP_TICKS;
    out_north = base_north * (1.0 - blend) + latched_marker_target_.north * blend;
    out_east = base_east * (1.0 - blend) + latched_marker_target_.east * blend;
}

void MissionManager::publishGripperCommand(const std::string & command)
{
    if (!gripper_cmd_pub_) {
        RCLCPP_ERROR(this->get_logger(),
            "Publisher gripper belum siap; command '%s' tidak terkirim.",
            command.c_str());
        return;
    }

    std_msgs::msg::String message;
    message.data = command;
    gripper_cmd_pub_->publish(message);
    RCLCPP_INFO(this->get_logger(),
        "[GRIPPER] publish '%s' ke %s",
        command.c_str(), gripper_cmd_topic_.c_str());
}

bool MissionManager::runGripperDropIfNeeded(
    double hold_north, double hold_east, double hold_down, double hold_yaw,
    const std::string & waypoint_label)
{
    // Signature dipertahankan agar semua call-site centering tetap sama;
    // selama drop Z memakai lock EKF, bukan target lidar hold_down.
    (void)hold_down;

    if (!gripper_drop_enable_ ||
        gripper_drop_completed_ ||
        current_wp_ != gripper_drop_after_wp_)
    {
        return false;
    }

    // Capture sebelum command OPEN pertama. Setelah payload/gripper bergerak,
    // TFmini bisa melihat objek itu sendiri dan meloncat puluhan sentimeter;
    // jangan lagi mengubah Z setpoint berdasarkan range sampai urutan drop
    // dan post-drop hold selesai.
    if (!gripper_hold_down_valid_) {
        gripper_hold_down_px4_ = origin_down_ + vehicle_.position.down;
        gripper_hold_down_valid_ = true;
        RCLCPP_INFO(this->get_logger(),
            "[%s][GRIPPER] ALTITUDE LOCK: PX4 down=%.3f sebelum OPEN.",
            waypoint_label.c_str(), gripper_hold_down_px4_);
    }

    control_->publishHeartbeat(true);
    control_->sendPositionSetpoint(
        toPx4North(hold_north),
        toPx4East(hold_east),
        gripper_hold_down_px4_,
        hold_yaw);

    switch (gripper_drop_state_) {
        case GripperDropState::IDLE:
            publishGripperCommand("open");
            payload_released_at_s_ = this->now().seconds();
            gripper_drop_state_ = GripperDropState::OPEN_SENT;
            gripper_drop_counter_ = 0;
            RCLCPP_INFO(this->get_logger(),
                "[%s] DROP BARANG: buka gripper, hold posisi sebelum yaw 90deg.",
                waypoint_label.c_str());
            return true;

        case GripperDropState::OPEN_SENT:
            ++gripper_drop_counter_;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "[%s] DROP BARANG: gripper open wait %d/%d",
                waypoint_label.c_str(), gripper_drop_counter_, gripper_open_wait_ticks_);
            if (gripper_drop_counter_ >= gripper_open_wait_ticks_) {
                publishGripperCommand("close");
                gripper_drop_state_ = GripperDropState::CLOSE_SENT;
                gripper_drop_counter_ = 0;
                RCLCPP_INFO(this->get_logger(),
                    "[%s] DROP BARANG: tutup gripper kembali.",
                    waypoint_label.c_str());
            }
            return true;

        case GripperDropState::CLOSE_SENT:
            ++gripper_drop_counter_;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "[%s] DROP BARANG: gripper close wait %d/%d",
                waypoint_label.c_str(), gripper_drop_counter_, gripper_close_wait_ticks_);
            if (gripper_drop_counter_ >= gripper_close_wait_ticks_) {
                gripper_drop_state_ = GripperDropState::COMPLETE;
                gripper_drop_completed_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "[%s] DROP BARANG selesai; lanjut yaw 90deg lalu maju 5.7m.",
                    waypoint_label.c_str());
            }
            return true;

        case GripperDropState::COMPLETE:
            return false;
    }

    return false;
}

double MissionManager::takeoffTargetDown() const
{
    if (waypoints_ && waypoints_->size() > 0) {
        return waypoints_->at(0).d;
    }
    return -1.5;
}

double MissionManager::currentAltitudeDown() const
{
    if (use_lidar_altitude_ && got_lidar_altitude_) {
        return -lidar_altitude_m_;
    }
    return vehicle_.position.down;
}

double MissionManager::currentAltitudeAgl() const
{
    return -currentAltitudeDown();
}

double MissionManager::missionReferenceYaw() const
{
    if (override_mission_heading_) {
        return degToRad(mission_heading_deg_);
    }
    return origin_yaw_ + degToRad(mission_heading_correction_deg_);
}

const char * MissionManager::altitudeSourceLabel() const
{
    if (use_lidar_altitude_) {
        return got_lidar_altitude_ ? "lidar" : "lidar-wait";
    }
    return "local";
}

const char * MissionManager::activeVisionSourceLabel() const
{
    switch (phase_) {
        case Phase::HOVER:
        case Phase::PILOT_ARUCO_SEARCH:
        case Phase::TAKEOFF_MARKER:
            // HOVER: ArUco dipakai menahan posisi terhadap marker (anti
            // drift EKF, lihat runHover()). TAKEOFF_MARKER: marker awal
            // wajib ditemukan lewat ArUco sebelum origin misi direbase.
            return "ARUCO";
        case Phase::MISSION:
            if (mission_gate_active_) {
                // Selama Livox memegang kontrol CENTER/ADVANCE, hentikan
                // inferensi kamera berat. Setelah gate selesai, sumber WP
                // normal otomatis aktif lagi.
                return "NONE";
            }
            // WP1 (current_wp_==0) pakai YOLO untuk centering ke box
            // sebelum gripper, WP lain (termasuk WP2) tetap ArUco — sama
            // seperti "yolo_centering_wp" di runMission().
            return current_wp_ == 0 ? "YOLO" : "ARUCO";
        case Phase::INIT:
        case Phase::WAIT_ARM:
        case Phase::TAKEOFF:
        case Phase::LAND_CMD:
        case Phase::WAIT_DISARM:
        default:
            // Belum/tidak ada koreksi vision yang dikonsumsi di fase ini.
            return "NONE";
    }
}

double MissionManager::toPx4DownForAltitudeTarget(double mission_down) const
{
    if (use_lidar_altitude_ && got_lidar_altitude_) {
        const double target_altitude_agl = -mission_down;
        const double altitude_error = target_altitude_agl - lidar_altitude_m_;
        return origin_down_ + vehicle_.position.down - altitude_error - altitude_command_bias_m_;
    }

    return toPx4Down(mission_down);
}

double MissionManager::toPx4North(double mission_north) const
{
    return origin_north_ + mission_north;
}

double MissionManager::toPx4East(double mission_east) const
{
    return origin_east_ + mission_east;
}

double MissionManager::toPx4Down(double mission_down) const
{
    return origin_down_ + mission_down - altitude_command_bias_m_;
}

void MissionManager::onStatusUpdate(uint8_t arming_state, uint8_t nav_state)
{
    ++status_update_count_;
    vehicle_.arming_state = arming_state;
    vehicle_.nav_state = nav_state;
}

void MissionManager::onMarkerPoseUpdate(const ControlModule::MarkerPoseSample & s)
{
    if (!vision_lock_enable_) return;

    // Selama takeoff jangan mengoreksi lateral. Saat HOVER sample justru
    // dipakai untuk menahan posisi fisik terhadap marker (anti drift EKF).
    if (phase_ == Phase::TAKEOFF) {
        return;
    }

    // WP1 (current_wp_==0 di MISSION) pakai YOLO/GroundLock sebagai sumber
    // centering ke box sebelum gripper, bukan ArUco — lihat
    // onTargetCenterUpdate() dan "yolo_centering_wp" di runMission().
    // Kamera nadir dipakai gantian (satu device, satu topic
    // /camera/image_raw) jadi aruco_node mestinya tidak jalan saat WP1,
    // tapi guard ini tetap dipasang untuk berjaga-jaga kalau kebetulan
    // masih ada data ArUco basi masuk — jangan sampai menimpa
    // marker_latest_*_ yang sedang dipakai jalur YOLO.
    if (phase_ == Phase::MISSION && current_wp_ == 0) {
        return;
    }

    // Frame kamera (OpenCV optical): x = kanan-gambar, y = bawah-gambar.
    // "Atas gambar" = -y. Konversi ke body-frame di sini (bukan di
    // ControlModule) supaya mount-yaw/rotasi tetap satu tempat di
    // VisionLock — lihat komentar di vision_lock.h soal kalibrasi
    // camera_mount_yaw_deg_.
    const double body_up    = -s.y;
    const double body_right = s.x;

    const double theta = camera_mount_yaw_deg_ * 3.14159265358979323846 / 180.0;
    const double ct = std::cos(theta);
    const double st = std::sin(theta);
    const double body_forward = body_up * ct - body_right * st;
    const double body_right2  = body_up * st + body_right * ct;

    const double cy = std::cos(vehicle_.yaw);
    const double sy = std::sin(vehicle_.yaw);
    const double raw_north = body_forward * cy - body_right2 * sy;
    const double raw_east  = body_forward * sy + body_right2 * cy;

    constexpr double OFFSET_FILTER_ALPHA = 0.25;
    if (!marker_offset_filter_ready_) {
        marker_latest_offset_north_ = raw_north;
        marker_latest_offset_east_  = raw_east;
        marker_offset_filter_ready_ = true;
    } else {
        marker_latest_offset_north_ +=
            OFFSET_FILTER_ALPHA * (raw_north - marker_latest_offset_north_);
        marker_latest_offset_east_ +=
            OFFSET_FILTER_ALPHA * (raw_east - marker_latest_offset_east_);
    }
    marker_latest_sample_available_ = true;
    last_marker_sample_s_ = this->now().seconds();
    marker_feedback_active_ = true;

    // Arah +Y marker adalah sumbu hijau pada debug ArUco. Fiducial detector
    // sudah mengoreksi mounting kamera terbalik lewat pose_yaw_offset_deg,
    // jadi proyeksi sumbu hijau ini dapat langsung dipakai sebagai error yaw
    // relatif terhadap arah depan badan. Hanya fase handoff awal yang akan
    // mengonsumsi nilai ini; fase ArUco lain tetap memakai perilaku lama.
    // Jangan hapus event orientasi valid yang belum dikonsumsi hanya karena
    // callback pose berikutnya tidak lolos proyeksi orientasi. Event valid
    // di-clear oleh loop alignment setelah dipakai tepat satu kali.
    if (s.orientation_valid) {
        const double q_norm = std::sqrt(
            s.qx * s.qx + s.qy * s.qy + s.qz * s.qz + s.qw * s.qw);
        const double qx = s.qx / q_norm;
        const double qy = s.qy / q_norm;
        const double qz = s.qz / q_norm;
        const double qw = s.qw / q_norm;

        // Kolom kedua rotation matrix = arah sumbu +Y (hijau) marker
        // dalam optical frame kamera: x kanan-gambar, y bawah-gambar.
        const double green_optical_right = 2.0 * (qx * qy - qz * qw);
        const double green_optical_down =
            1.0 - 2.0 * (qx * qx + qz * qz);
        const double green_optical_up = -green_optical_down;
        const double green_body_forward =
            green_optical_up * ct - green_optical_right * st;
        const double green_body_right =
            green_optical_up * st + green_optical_right * ct;
        const double projected_norm = std::hypot(
            green_body_forward, green_body_right);
        if (projected_norm > 0.20) {
            const double raw_yaw_error = std::atan2(
                green_body_right, green_body_forward);
            constexpr double YAW_FILTER_ALPHA = 0.40;
            if (!marker_tf_yaw_filter_ready_) {
                marker_latest_tf_yaw_error_rad_ = raw_yaw_error;
                marker_tf_yaw_filter_ready_ = true;
            } else {
                const double delta = std::atan2(
                    std::sin(raw_yaw_error - marker_latest_tf_yaw_error_rad_),
                    std::cos(raw_yaw_error - marker_latest_tf_yaw_error_rad_));
                marker_latest_tf_yaw_error_rad_ = std::atan2(
                    std::sin(marker_latest_tf_yaw_error_rad_ +
                        YAW_FILTER_ALPHA * delta),
                    std::cos(marker_latest_tf_yaw_error_rad_ +
                        YAW_FILTER_ALPHA * delta));
            }
            marker_latest_tf_yaw_available_ = true;
            last_marker_tf_yaw_s_ = this->now().seconds();
        }
    }

    // Waktu kedatangan callback (this->now()), BUKAN header.stamp pesan
    // — konsisten dengan last_position_time_/last_range_time_ di file
    // ini, menghindari mismatch clock-domain dengan stempel waktu
    // capture gambar dari aruco_node.
    vision_lock_->update(
        body_up, body_right,
        vehicle_.position, vehicle_.yaw,
        this->now().seconds());
}

void MissionManager::onMarkerCentersUpdate(
    const ControlModule::MarkerCentersSample & s)
{
    marker_centers_latest_ = s;
    marker_centers_available_ = true;
    last_marker_centers_s_ = this->now().seconds();

    // Simpan hanya DETEKSI marker besar yang valid untuk alignment handoff
    // awal. Pesan berikutnya yang markers-nya kosong tidak menghapus sampel
    // ini; umur sampelnya sendiri yang menentukan kapan ia stale.
    const ControlModule::MarkerCenter * initial_marker = nullptr;
    for (const auto & marker : s.markers) {
        if (marker.id == marker_heading_back_id_) {
            initial_marker = &marker;
            break;
        }
    }
    if (!initial_marker && s.markers.size() == 1) {
        initial_marker = &s.markers.front();
    }
    if (initial_marker &&
        s.frame_width_px > 0.0 && s.frame_height_px > 0.0)
    {
        initial_marker_center_x_px_ = initial_marker->x_px;
        initial_marker_center_y_px_ = initial_marker->y_px;
        initial_marker_frame_width_px_ = s.frame_width_px;
        initial_marker_frame_height_px_ = s.frame_height_px;
        last_initial_marker_center_s_ = this->now().seconds();
        initial_marker_center_available_ = true;
        initial_marker_center_new_ = true;
    }
}

void MissionManager::onTargetCenterUpdate(
    const ControlModule::TargetCenterSample & s)
{
    if (!ground_lock_enable_ || !s.valid) return;

    // Sama seperti onMarkerPoseUpdate(): jangan koreksi lateral selama
    // TAKEOFF.
    if (phase_ == Phase::TAKEOFF) return;

    // GroundLock butuh altitude AGL valid untuk proyeksi ground-plane
    // (lihat komentar GroundLock::update) — tanpa lidar, sample ini
    // tidak berarti apa-apa, diamkan saja daripada kirim altitude 0
    // yang akan langsung ditolak GroundLock.
    if (!(use_lidar_altitude_ && got_lidar_altitude_)) return;

    ground_lock_->update(
        s.cx_px, s.cy_px,
        s.frame_width_px, s.frame_height_px,
        lidar_altitude_m_,
        vehicle_.position, vehicle_.yaw,
        this->now().seconds());

    const double now_s = this->now().seconds();
    yolo_ui_center_error_x_px_ = s.cx_px - 0.5 * s.frame_width_px;
    yolo_ui_center_error_y_px_ = s.cy_px - 0.5 * s.frame_height_px;
    yolo_ui_centered_ =
        std::abs(yolo_ui_center_error_x_px_) <= yolo_ui_center_tolerance_px_ &&
        std::abs(yolo_ui_center_error_y_px_) <= yolo_ui_center_tolerance_px_;
    last_yolo_ui_center_s_ = now_s;

    // YOLO/GroundLock cuma jadi sumber centering untuk WP1 (current_wp_==0,
    // lihat "yolo_centering_wp" di runMission()) — WP lain (termasuk WP2)
    // tetap pakai ArUco. Di luar WP1, ground_lock_ tetap ter-update di atas
    // (state debounce-nya hidup, siap dipakai begitu WP1 mulai) tapi
    // variabel marker_latest_*_ yang dibaca CENTER_MARKER TIDAK disentuh,
    // supaya tidak menimpa data ArUco milik WP lain.
    if (!(phase_ == Phase::MISSION && current_wp_ == 0)) {
        return;
    }

    // min_consecutive_samples=1 di ground_cfg (lihat konstruktor node) ->
    // lockedTarget() sudah mencerminkan update() barusan di atas, sama
    // seperti pola onMarkerPoseUpdate() untuk ArUco.
    const auto target = ground_lock_->lockedTarget();
    const double raw_north = target.north - vehicle_.position.north;
    const double raw_east  = target.east  - vehicle_.position.east;

    constexpr double OFFSET_FILTER_ALPHA = 0.25;
    if (!marker_offset_filter_ready_) {
        marker_latest_offset_north_ = raw_north;
        marker_latest_offset_east_  = raw_east;
        marker_offset_filter_ready_ = true;
    } else {
        marker_latest_offset_north_ +=
            OFFSET_FILTER_ALPHA * (raw_north - marker_latest_offset_north_);
        marker_latest_offset_east_ +=
            OFFSET_FILTER_ALPHA * (raw_east - marker_latest_offset_east_);
    }
    marker_latest_sample_available_ = true;
    last_marker_sample_s_ = now_s;
    marker_feedback_active_ = true;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
        "[YOLO-RX] box valid conf=%.2f center=(%.0f,%.0f)px "
        "ui_err=(%+.0f,%+.0f)px ui=%s offset=(N%+.3f,E%+.3f)m.",
        s.confidence, s.cx_px, s.cy_px,
        yolo_ui_center_error_x_px_, yolo_ui_center_error_y_px_,
        yolo_ui_centered_ ? "CENTER" : "NOT_CENTER",
        marker_latest_offset_north_, marker_latest_offset_east_);
}

bool MissionManager::isPositionStale(double threshold_s) const
{
    return (this->now() - last_position_time_).seconds() > threshold_s;
}

bool MissionManager::isLidarStale(double threshold_s) const
{
    return (this->now() - last_range_time_).seconds() > threshold_s;
}

void MissionManager::onLivoxUpdate(const ControlModule::LivoxSample & s)
{
    // Callback dan subscriber hanya dipasang ketika kill-switch aktif.
    // Guard ini tetap dipertahankan sebagai lapisan keselamatan kedua.
    if (!gate_centering_enable_) return;

    gate_centering_latest_ = gate_centering_lock_->update(s.points);

    if (!gate_centering_latest_.valid) return;
    last_gate_sample_s_ = this->now().seconds();

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[LIVOX] gate=%s lat_err=%.3fm forward=%.2fm width=%.2fm yaw_err=%.1fdeg centered=%s",
        gate_centering_latest_.one_side_only ? "one-side" : "two-pole",
        gate_centering_latest_.lateral_error_m,
        gate_centering_latest_.forward_distance_m,
        gate_centering_latest_.detected_width_m,
        radToDeg(gate_centering_latest_.heading_error_rad),
        gate_centering_latest_.centered ? "yes" : "no");

    // Feed debounce/freeze wrapper (dipakai runGatePassMission()). body_up
    // selalu 0 — lihat komentar gate_centering_debounce_ di
    // mission_manager.h.
    gate_centering_debounce_->update(
        0.0, gate_centering_latest_.lateral_error_m,
        vehicle_.position, vehicle_.yaw, this->now().seconds());
}

// ==================================================================
// Loop utama — dipanggil 10Hz
// ==================================================================

void MissionManager::loop()
{
    // Publish di awal, sebelum guard/return apa pun, supaya aruco_node/
    // yolo_camera_node selalu punya sinyal terbaru untuk gating skip-
    // inferensi mereka — termasuk selama INIT/WAIT_ARM sebelum origin
    // terkunci.
    {
        std_msgs::msg::String vision_source_msg;
        vision_source_msg.data = activeVisionSourceLabel();
        vision_source_pub_->publish(vision_source_msg);
    }

    // Satu baris ringkas per detik untuk membedakan kontrol pilot dan
    // autonomous, sumber kamera yang dipakai, serta interupsi gate tanpa
    // membanjiri log 10Hz kontrol detail.
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[MISSION][STATUS] control=%s phase=%s wp=%zu vision=%s gate=%s",
        phase_ == Phase::PILOT_ARUCO_SEARCH ? "PILOT/MANUAL" : "AUTONOMOUS",
        phaseLabel(phase_), current_wp_ + 1, activeVisionSourceLabel(),
        gateStatusLabel(mission_gate_active_, mission_gate_stage_));

    if (!got_raw_position_) {
        control_->publishHeartbeat(true);
        control_->sendPositionSetpoint(0.0, 0.0, -1.5, 0.0);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
            1000, "Menunggu sample /fmu/out/vehicle_local_position dari PX4...");
        return;
    }

    if (!vehicle_.got_position) {
        control_->publishHeartbeat(true);
        control_->sendPositionSetpoint(0.0, 0.0, -1.5, 0.0);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
            1000, "Posisi PX4 sudah masuk, menunggu local origin stabil...");
        return;
    }

    if (!ensureMissionWaypointsReady()) {
        control_->publishHeartbeat(true);
        control_->sendPositionSetpoint(
            toPx4North(vehicle_.position.north),
            toPx4East(vehicle_.position.east),
            origin_down_ + vehicle_.position.down,
            vehicle_.yaw);
        return;
    }

    if (use_lidar_altitude_ && (!got_lidar_altitude_ || isLidarStale(0.5))) {
        if (phase_ == Phase::INIT || phase_ == Phase::WAIT_ARM) {
            control_->publishHeartbeat(true);
            control_->sendPositionSetpoint(
                toPx4North(vehicle_.position.north),
                toPx4East(vehicle_.position.east),
                origin_down_ + vehicle_.position.down,
                vehicle_.yaw);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Menunggu data altitude TFmini /range yang valid sebelum ARM/TAKEOFF...");
            return;
        }

        if (phase_ != Phase::LAND_CMD && phase_ != Phase::WAIT_DISARM) {
            RCLCPP_ERROR(this->get_logger(),
                "TFmini altitude stale/tidak valid saat misi berjalan — FAILSAFE: kirim LAND.");
            control_->publishHeartbeat(true);
            control_->sendLandCommand();
            phase_ = Phase::WAIT_DISARM;
            return;
        }
    }

    // ── Guard data posisi basi ────────────────────────────────────
    // Peringatan dini di 0.3s, failsafe LAND otomatis di 1.5s tanpa
    // update posisi sama sekali. Ini mencegah drone "terbang buta"
    // tanpa batas waktu ketika link uXRCE-DDS (mis. serial baud rate
    // companion<->FC) jenuh/putus saat offboard streaming aktif —
    // kejadian nyata yang terekam di log: posisi beku total selama
    // >80 detik walau drone tetap armed & terbang.
    if (isPositionStale(0.3) && phase_ != Phase::WAIT_DISARM) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "POSISI STALE (>%.1fs sejak update terakhir dari PX4) — "
            "cek bandwidth link uXRCE-DDS (baud MicroXRCEAgent).", 0.3);
    }
    if (isPositionStale(1.5) && phase_ != Phase::LAND_CMD && phase_ != Phase::WAIT_DISARM) {
        RCLCPP_ERROR(this->get_logger(),
            "POSISI STALE >1.5s — FAILSAFE: kirim LAND. Link uXRCE-DDS "
            "kemungkinan jenuh (naikkan baud rate serial companion<->FC).");
        control_->publishHeartbeat(true);
        control_->sendLandCommand();
        phase_ = Phase::WAIT_DISARM;
        return;
    }

    // ── Guard altitude tidak masuk akal ───────────────────────────
    // Defense-in-depth terhadap KEJADIAN NYATA "terbang sangat tinggi":
    // origin altitude sempat ter-capture keliru puluhan meter (root
    // cause sudah diperbaiki di onPositionUpdate/stabilisasi origin),
    // menyebabkan target relatif salah total dan drone terus naik
    // mengejar target semu. Berapa pun waypoint yang dipakai, altitude
    // misi realistis untuk arena ini ada di orde single-digit meter —
    // kalau altitude relatif (dari origin yang sudah terkunci) melebihi
    // batas aman, itu tanda ada yang salah (origin, EKF, atau input
    // waypoint keliru), bukan situasi normal yang harus terus dikejar.
    constexpr double MAX_SANE_ALT_M = 8.0;
    if (std::abs(currentAltitudeDown()) > MAX_SANE_ALT_M &&
        phase_ != Phase::LAND_CMD && phase_ != Phase::WAIT_DISARM)
    {
        RCLCPP_ERROR(this->get_logger(),
            "ALTITUDE TIDAK MASUK AKAL (%.1fm dari origin, batas %.1fm) — "
            "FAILSAFE: kirim LAND.", currentAltitudeAgl(), MAX_SANE_ALT_M);
        control_->publishHeartbeat(true);
        control_->sendLandCommand();
        phase_ = Phase::WAIT_DISARM;
        return;
    }

    switch (phase_) {
        case Phase::INIT:        runInit();        break;
        case Phase::WAIT_ARM:    runWaitArm();      break;
        case Phase::TAKEOFF:     runTakeoff();      break;
        case Phase::HOVER:       runHover();        break;
        case Phase::PILOT_ARUCO_SEARCH: runPilotArucoSearch(); break;
        case Phase::TAKEOFF_MARKER: runTakeoffMarker(); break;
        case Phase::MISSION:     runMission();      break;
        case Phase::GATE_PASS:   runGatePassMission(); break;
        case Phase::LAND_CMD:    runLandCmd();      break;
        case Phase::WAIT_DISARM: runWaitDisarm();   break;
    }
}

// ==================================================================
// INIT — stream setpoint 1 detik, lalu set offboard mode + arm
// ==================================================================

void MissionManager::runInit()
{
    if (start_mode_ == StartMode::AIRBORNE_HANDOFF) {
        // Keselamatan handoff: paksa Position mode pada tick PERTAMA,
        // sebelum heartbeat/setpoint Offboard apa pun dipublish. Ini
        // memutus Offboard yang mungkin masih aktif dari run sebelumnya dan
        // memastikan stick RC tetap memegang kendaraan.
        if (counter_ == 0) {
            control_->setPositionMode();
            marker_centers_available_ = false;
            marker_centers_latest_.markers.clear();
            last_marker_centers_s_ = -1.0;
            initial_marker_center_available_ = false;
            initial_marker_center_new_ = false;
            last_initial_marker_center_s_ = -1.0;
            marker_latest_tf_yaw_available_ = false;
            marker_tf_yaw_filter_ready_ = false;
            last_marker_tf_yaw_s_ = -1.0;
            marker_stable_frames_ = 0;
            handoff_marker_ready_ = false;
            handoff_vision_aligned_ = false;
            handoff_offboard_prestream_ticks_ = 0;
            RCLCPP_WARN(this->get_logger(),
                "=== HANDOFF SAFE INIT === Position mode dikirim; program "
                "tidak publish Offboard selama pilot belum menemukan ArUco.");
        }

        ++counter_;
        if (counter_ >= 10) {
            // Ulangi perintah setelah jeda satu detik, lalu mulai menerima
            // hanya sample ArUco BARU milik fase pencarian pilot.
            control_->setPositionMode();
            takeoff_hold_north_ = vehicle_.position.north;
            takeoff_hold_east_ = vehicle_.position.east;
            takeoff_hold_yaw_ = vehicle_.yaw;
            vehicle_.hover_position = vehicle_.position;
            marker_center_target_ = vehicle_.position;
            marker_centers_available_ = false;
            marker_centers_latest_.markers.clear();
            last_marker_centers_s_ = -1.0;
            initial_marker_center_available_ = false;
            initial_marker_center_new_ = false;
            last_initial_marker_center_s_ = -1.0;
            marker_latest_tf_yaw_available_ = false;
            marker_tf_yaw_filter_ready_ = false;
            last_marker_tf_yaw_s_ = -1.0;
            marker_stable_frames_ = 0;
            phase_ = Phase::PILOT_ARUCO_SEARCH;
            counter_ = 0;
            RCLCPP_WARN(this->get_logger(),
                "=== HANDOFF PILOT SEARCH === Pilot tetap mengendalikan; "
                "menunggu dua sample ArUco baru.");
        }
        return;
    }

    control_->publishHeartbeat(true);

    control_->sendPositionSetpoint(
        toPx4North(vehicle_.position.north),
        toPx4East(vehicle_.position.east),
        origin_down_ + vehicle_.position.down,
        vehicle_.yaw);

    ++counter_;
    RCLCPP_INFO(this->get_logger(), "INIT prestream... (%d/10)", counter_);

    if (counter_ >= 10) {
        // Kunci titik takeoff SATU KALI sebelum arm. Jangan memperbarui
        // target ini selama naik: drift/gerak kecil harus dikoreksi kembali
        // ke titik awal, bukan dijadikan setpoint baru pada tick berikutnya.
        takeoff_hold_north_ = vehicle_.position.north;
        takeoff_hold_east_  = vehicle_.position.east;
        takeoff_hold_yaw_   = missionReferenceYaw();

        control_->setOffboardMode();
        control_->arm();
        phase_   = Phase::WAIT_ARM;
        counter_ = 0;
        RCLCPP_INFO(this->get_logger(),
            "Menunggu konfirmasi ARM dari PX4... hold yaw %.1fdeg (%s)",
            radToDeg(takeoff_hold_yaw_), start_mode_param_.c_str());
    }
}

// ==================================================================
// WAIT_ARM — jeda singkat setelah arm() sebelum lanjut TAKEOFF.
//
// RIWAYAT: awalnya fase ini didesain untuk BLOKIR sampai
// vehicle_.arming_state == ARMED terkonfirmasi, dan ABORT kalau tidak
// terkonfirmasi dalam 3 detik. Terbukti di lapangan (diverifikasi
// dengan counter debug) bahwa `status_update_count_` selalu 0 —
// callback status dari topik "/fmu/out/vehicle_status_v1" TIDAK PERNAH
// terpanggil sama sekali (kemungkinan mismatch versi skema pesan;
// topik ber-suffix "_v1" vs px4_msgs::msg::VehicleStatus versi
// terbaru yang di-compile di workspace ini), padahal callback posisi
// tetap normal. Akibatnya guard lama SELALU membatalkan misi dalam
// 3 detik terlepas dari apakah PX4 benar-benar arm atau tidak — false
// abort, bukan deteksi kegagalan nyata.
//
// FIX: jangan blokir/batalkan misi berdasarkan data yang terbukti
// tidak pernah tersedia. Tunggu jeda singkat (1 detik, cukup untuk
// PX4 memproses arm) lalu lanjut ke TAKEOFF apa pun kondisinya — ini
// mengembalikan perilaku lama yang TERBUKTI sukses di penerbangan
// nyata. Kalau di masa depan topik status ternyata mulai memberi data
// (status_update_count_ > 0), kode ini akan otomatis memanfaatkannya:
// lanjut cepat kalau sudah ARMED, atau batalkan dengan aman kalau
// setelah jeda ternyata benar-benar masih DISARMED.
// ==================================================================

void MissionManager::runWaitArm()
{
    control_->publishHeartbeat(true);

    control_->sendPositionSetpoint(
        toPx4North(takeoff_hold_north_),
        toPx4East(takeoff_hold_east_),
        origin_down_ + vehicle_.position.down,   // tahan posisi sekarang, jangan naik dulu
        takeoff_hold_yaw_);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "WAIT_ARM: raw arming_state = %u (ARMED=%u, DISARMED=%u) | "
        "status callback terpanggil %u kali",
        vehicle_.arming_state,
        px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED,
        px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED,
        status_update_count_);

    // Data status PX4 pernah/sedang tersedia -> boleh dipercaya.
    if (status_update_count_ > 0 &&
        vehicle_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
    {
        // Ambil koordinat estimator terbaru SETELAH arm/spool-up, lalu
        // bekukan. Pergeseran estimator sebelum titik ini tidak menjadi
        // error position, sedangkan gerak fisik setelah mulai naik tetap
        // dikoreksi kembali ke anchor ini.
        takeoff_hold_north_ = vehicle_.position.north;
        takeoff_hold_east_ = vehicle_.position.east;
        phase_   = Phase::TAKEOFF;
        counter_ = 0;
        RCLCPP_INFO(this->get_logger(),
            "ARM terkonfirmasi. === INITIAL ALTITUDE ALIGN (%s) ===",
            start_mode_param_.c_str());
        return;
    }

    ++counter_;
    if (counter_ >= 2) {   // prestream sudah 1 detik; hanya beri 0.2s sesudah ARM
        if (status_update_count_ == 0) {
            RCLCPP_WARN(this->get_logger(),
                "Topik status PX4 tidak memberi data (callback 0x) — lanjut "
                "TAKEOFF tanpa verifikasi arm eksplisit.");
        } else if (vehicle_.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
            RCLCPP_ERROR(this->get_logger(),
                "Status PX4 tersedia dan menunjukkan BUKAN armed setelah "
                "jeda ARM — misi dibatalkan, drone tidak pernah lepas landas.");
            timer_->cancel();
            return;
        }
        takeoff_hold_north_ = vehicle_.position.north;
        takeoff_hold_east_ = vehicle_.position.east;
        phase_   = Phase::TAKEOFF;
        counter_ = 0;
        RCLCPP_INFO(this->get_logger(),
            "=== INITIAL ALTITUDE ALIGN (%s) ===",
            start_mode_param_.c_str());
    }
}

// ==================================================================
// TAKEOFF / AIRBORNE_HANDOFF — sejajarkan altitude ke waypoint pertama
// ==================================================================

void MissionManager::runTakeoff()
{
    control_->publishHeartbeat(true);

    // Anchor N/E sudah dikunci sebelum ARM di runInit(). Pertahankan titik
    // itu tanpa syarat sejak spool-up, sepanjang naik, sampai hover selesai.
    // Dengan demikian setiap drift fisik saat liftoff tetap menjadi error
    // posisi yang dikoreksi PX4 kembali ke titik awal; drift tidak pernah
    // diadopsi sebagai anchor takeoff baru.
    const double altitude_error =
        WaypointHandler::altitudeError(currentAltitudeDown(), takeoffTargetDown());
    constexpr double TAKEOFF_CLIMB_SPEED_M_S = 0.35;
    constexpr double POSITION_CAPTURE_BAND_M = 0.15;
    const bool takeoff_velocity_climb =
        start_mode_ == StartMode::TAKEOFF &&
        altitude_error < -POSITION_CAPTURE_BAND_M;

    if (takeoff_velocity_climb) {
        // NED: vz negatif berarti naik. X/Y tetap position-hold pada anchor
        // setelah ARM, tetapi Z memakai velocity agar tidak terbentuk error
        // altitude besar saat kaki drone masih menyentuh tanah.
        control_->sendTakeoffSetpoint(
            toPx4North(takeoff_hold_north_),
            toPx4East(takeoff_hold_east_),
            -TAKEOFF_CLIMB_SPEED_M_S,
            takeoff_hold_yaw_);
    } else {
        // Di 15 cm terakhir (atau seluruh airborne_handoff), tangkap target
        // dengan position controller agar berhenti tepat dan stabil.
        control_->sendPositionSetpoint(
            toPx4North(takeoff_hold_north_),
            toPx4East(takeoff_hold_east_),
            toPx4DownForAltitudeTarget(takeoffTargetDown()),
            takeoff_hold_yaw_);
    }

    const bool altitude_ready =
        start_mode_ == StartMode::AIRBORNE_HANDOFF
        ? std::abs(altitude_error) <= 0.15
        : currentAltitudeDown() <= takeoffTargetDown() + 0.15;

    const double handoff_xy_error = std::hypot(
        takeoff_hold_north_ - vehicle_.position.north,
        takeoff_hold_east_ - vehicle_.position.east);
    const double handoff_xy_speed = std::hypot(
        vehicle_.velocity.north, vehicle_.velocity.east);
    const double handoff_vertical_speed = std::abs(vehicle_.velocity.down);
    const double handoff_yaw_error = std::abs(std::atan2(
        std::sin(takeoff_hold_yaw_ - vehicle_.yaw),
        std::cos(takeoff_hold_yaw_ - vehicle_.yaw)));
    constexpr double HANDOFF_LOCK_POSITION_M = 0.06;
    constexpr double HANDOFF_LOCK_SPEED_M_S = 0.08;
    constexpr double HANDOFF_LOCK_VERTICAL_SPEED_M_S = 0.12;
    constexpr double HANDOFF_LOCK_YAW_RAD = 3.0 * PI / 180.0;
    const bool handoff_lock_ready = altitude_ready &&
        handoff_xy_error <= HANDOFF_LOCK_POSITION_M &&
        handoff_xy_speed <= HANDOFF_LOCK_SPEED_M_S &&
        handoff_vertical_speed <= HANDOFF_LOCK_VERTICAL_SPEED_M_S &&
        handoff_yaw_error <= HANDOFF_LOCK_YAW_RAD;

    if (start_mode_ == StartMode::AIRBORNE_HANDOFF &&
        handoff_vision_aligned_)
    {
        // Setelah center+heading ArUco sudah dikunci, fase ini adalah hover
        // BERBATAS WAKTU, bukan gate stabilisasi yang dapat reset selamanya.
        // Selama tepat 3 detik PX4 tetap menahan anchor/yaw dan menyesuaikan
        // altitude program; tick tidak di-reset oleh noise EKF/lidar.
        initial_altitude_stable_ticks_ = std::min(
            initial_altitude_stable_ticks_ + 1, 30);
    } else if (start_mode_ == StartMode::AIRBORNE_HANDOFF) {
        if (handoff_lock_ready) {
            ++initial_altitude_stable_ticks_;
        } else {
            initial_altitude_stable_ticks_ = 0;
        }
    } else {
        if (altitude_ready) {
            initial_altitude_stable_ticks_ = std::min(
                initial_altitude_stable_ticks_ + 1, 10);
        } else {
            initial_altitude_stable_ticks_ = 0;
        }
    }

    const int required_altitude_ticks =
        start_mode_ == StartMode::AIRBORNE_HANDOFF && handoff_vision_aligned_
        ? 30
        : (start_mode_ == StartMode::AIRBORNE_HANDOFF ? 10 : 5);
    RCLCPP_INFO(this->get_logger(),
        "Altitude align[%s/%s/%s]: %.2fm -> %.2fm err=%.2fm stable=%d/%d | "
        "yaw=%.1fdeg | hold=(%.3f,%.3f) actual=(%.3f,%.3f) "
        "xy_err=(%.3f,%.3f) vxy=(%.3f,%.3f)",
        start_mode_param_.c_str(),
        altitudeSourceLabel(),
        takeoff_velocity_climb ? "VEL-Z" : "POS-CAPTURE",
        currentAltitudeAgl(), -takeoffTargetDown(),
        altitude_error,
        initial_altitude_stable_ticks_, required_altitude_ticks,
        radToDeg(takeoff_hold_yaw_),
        takeoff_hold_north_, takeoff_hold_east_,
        vehicle_.position.north, vehicle_.position.east,
        takeoff_hold_north_ - vehicle_.position.north,
        takeoff_hold_east_ - vehicle_.position.east,
        vehicle_.velocity.north, vehicle_.velocity.east);

    if (start_mode_ == StartMode::AIRBORNE_HANDOFF) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[MISSION][HANDOFF] %s: pos_err=%.3fm speed=%.3fm/s "
            "vz=%.3fm/s yaw_err=%.1fdeg altitude=%s time=%d/%d",
            handoff_vision_aligned_ ? "TIMED HOVER 3s" : "HOLD PILOT ANCHOR",
            handoff_xy_error, handoff_xy_speed, handoff_vertical_speed,
            radToDeg(handoff_yaw_error), altitude_ready ? "OK" : "ADJUST",
            initial_altitude_stable_ticks_, required_altitude_ticks);
    }

    if (initial_altitude_stable_ticks_ >= required_altitude_ticks) {
        vehicle_.hover_position.north = takeoff_hold_north_;
        vehicle_.hover_position.east  = takeoff_hold_east_;
        vehicle_.hover_position.down  = currentAltitudeDown();
        marker_center_target_ = vehicle_.hover_position;
        marker_latest_sample_available_ = false;
        marker_offset_filter_ready_ = false;
        last_marker_sample_s_ = -1.0;
        RCLCPP_INFO(this->get_logger(),
            "Initial altitude aligned (%.2fm AGL).",
            currentAltitudeAgl());

        if (start_mode_ == StartMode::AIRBORNE_HANDOFF &&
            handoff_vision_aligned_)
        {
            // 5 frame pilot -> center pixel + yaw TF one-shot -> capture posisi
            // aktual -> altitude adjustment + hover tepat 3 detik -> maju.
            rebaseMissionOriginToCurrentPosition();
            phase_ = Phase::MISSION;
            RCLCPP_INFO(this->get_logger(),
                "=== HANDOFF ALTITUDE ALIGNED: START MISSION ===");
            return;
        }
        phase_ = Phase::HOVER;
    }
}

// ==================================================================
// HOVER — stabilisasi 3 detik (30 tick @ 10Hz)
// ==================================================================

void MissionManager::runHover()
{
    control_->publishHeartbeat(true);

    // Hover awal harus benar-benar diam di atas titik takeoff. Jangan lakukan
    // centering ArUco di sini karena koreksi kamera akan tampak sebagai gerak
    // maju/mundur tepat setelah naik. Marker baru boleh mengubah posisi pada
    // fase TAKEOFF_MARKER setelah periode stabilisasi ini selesai.
    const double hold_north = takeoff_hold_north_;
    const double hold_east = takeoff_hold_east_;
    marker_center_target_ = vehicle_.hover_position;

    control_->sendPositionSetpoint(
        toPx4North(hold_north),
        toPx4East(hold_east),
        toPx4DownForAltitudeTarget(takeoffTargetDown()),
        takeoff_hold_yaw_);

    // Baca ArUco selama hover, tetapi JANGAN menggeser setpoint. Kalau marker
    // sudah terlihat stabil, tiga detik hover sekaligus menjadi fase validasi
    // sehingga setelah hover bisa langsung centering tanpa search ke tempat
    // lain. Satu sample hanya dihitung satu kali.
    const double marker_age_s = this->now().seconds() - last_marker_sample_s_;
    const bool hover_marker_valid = marker_latest_sample_available_ &&
        marker_age_s <= 0.35 &&
        std::hypot(marker_latest_offset_north_, marker_latest_offset_east_) <= 0.80;
    if (hover_marker_valid) {
        marker_stable_frames_ = std::min(marker_stable_frames_ + 1, 5);
    } else if (marker_latest_sample_available_) {
        marker_stable_frames_ = 0;
    }
    marker_latest_sample_available_ = false;

    ++hover_counter_;
    RCLCPP_INFO(this->get_logger(), "Hover... %d/30", hover_counter_);

    if (!start_mission_after_hover_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "HOVER HOLD: anchor takeoff + koreksi ArUco terbatas; misi dinonaktifkan.");
        return;
    }

    if (hover_counter_ >= 30) {
        hover_counter_ = 0;

        // start_mode:=gate_pass -> bypass total TAKEOFF_MARKER/MISSION
        // (ArUco/YOLO/gripper), langsung ke runGatePassMission(). Lihat
        // Context di plan/docs/LIVOX_MID360_INTEGRATION.md §9 kenapa ini
        // dipisah total dari state machine ArUco, bukan diselipkan.
        if (start_mode_ == StartMode::GATE_PASS) {
            vehicle_.hover_position.north = hold_north;
            vehicle_.hover_position.east = hold_east;
            gate_pass_stage_ = GatePassStage::CENTER;
            gate_pass_centered_ticks_ = 0;
            gate_pass_engaged_prev_ = false;
            gate_pass_center_target_ = vehicle_.position;
            gate_pass_stage_started_at_ = this->now();
            phase_ = Phase::GATE_PASS;
            RCLCPP_INFO(this->get_logger(),
                "=== TAKEOFF COMPLETE: GATE PASS (CENTER) ===");
            return;
        }

        if (start_mode_ == StartMode::AIRBORNE_HANDOFF) {
            // Setelah altitude dan hover stabil, kembalikan kontrol kepada
            // pilot. Program hanya memantau ArUco sampai pilot secara
            // eksplisit memilih Offboard lagi lewat switch RC.
            vehicle_.hover_position = vehicle_.position;
            marker_center_target_ = vehicle_.position;
            handoff_marker_ready_ = marker_stable_frames_ >= 5;
            control_->setPositionMode();
            phase_ = Phase::PILOT_ARUCO_SEARCH;
            RCLCPP_WARN(this->get_logger(),
                "=== HANDOFF PILOT SEARCH === Cari ArUco memakai RC pada "
                "Position mode. Saat marker valid 5/5 program mengambil "
                "OFFBOARD otomatis untuk centering + heading alignment.");
            return;
        }

        // Lanjutkan SEARCH dari target horizontal terakhir yang dipakai saat
        // HOVER. Tanpa ini, SEARCH kembali ke anchor takeoff lama dan dapat
        // menarik drone mundur/menyamping sesaat setelah hover selesai.
        vehicle_.hover_position.north = hold_north;
        vehicle_.hover_position.east = hold_east;
        marker_center_target_.north = hold_north;
        marker_center_target_.east = hold_east;
        phase_ = Phase::TAKEOFF_MARKER;
        const bool aruco_ready_from_hover = marker_stable_frames_ >= 5;
        waypoint_phase_ = aruco_ready_from_hover
            ? WaypointPhase::CENTER_MARKER
            : WaypointPhase::SEARCH_MARKER;
        marker_search_started_at_ = this->now();
        marker_search_altitude_m_ = -takeoffTargetDown();
        marker_search_direction_ = 1;
        marker_search_horizontal_m_ = 0.0;
        marker_search_horizontal_direction_ = 1;
        if (aruco_ready_from_hover) {
            marker_center_target_ = vehicle_.position;
            marker_stable_frames_ = 0;
            wp_hold_counter_ = 0;
            RCLCPP_INFO(this->get_logger(),
                "=== TAKEOFF COMPLETE: ARUCO sudah valid saat HOVER - langsung CENTER ===");
        } else {
            RCLCPP_INFO(this->get_logger(),
                "=== TAKEOFF COMPLETE: SEARCH ARUCO WP1 ===");
        }
    }
}

// ==================================================================
// PILOT_ARUCO_SEARCH — khusus airborne_handoff
// ==================================================================

void MissionManager::runPilotArucoSearch()
{
    const double now_s = this->now().seconds();
    const double centers_age_s = now_s - last_marker_centers_s_;
    auto find_pilot_marker = [this](int id) -> const ControlModule::MarkerCenter * {
        for (const auto & marker : marker_centers_latest_.markers) {
            if (marker.id == id) return &marker;
        }
        return nullptr;
    };
    const auto * pilot_back = find_pilot_marker(marker_heading_back_id_);
    // Mode handoff sekarang memang hanya memakai satu marker besar. Jika ID
    // fisiknya bukan default 0 tetapi hanya ada satu marker di frame, marker
    // tunggal itu tetap tidak ambigu dan aman dipakai sebagai target.
    if (!pilot_back && marker_centers_latest_.markers.size() == 1) {
        pilot_back = &marker_centers_latest_.markers.front();
    }
    const bool valid_sample = marker_centers_available_ &&
        centers_age_s <= 0.35 && pilot_back;
    if (!handoff_marker_ready_) {
        marker_stable_frames_ = valid_sample ? marker_stable_frames_ + 1 : 0;
        if (marker_stable_frames_ >= 5) {
            handoff_marker_ready_ = true;
            handoff_offboard_prestream_ticks_ = 0;
            RCLCPP_WARN(this->get_logger(),
                "=== ARUCO BESAR READY id=%d (5/5) === Program menyiapkan "
                "OFFBOARD otomatis; pilot lepaskan stick pada posisi netral.",
                pilot_back ? pilot_back->id : -1);
        }
    }
    marker_latest_sample_available_ = false;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "[PILOT-ARUCO] kendali PILOT | marker=%s valid=%d/5",
        handoff_marker_ready_ ? "READY" : "searching",
        marker_stable_frames_);

    if (!handoff_marker_ready_) {
        return;
    }

    // PX4 mensyaratkan stream Offboard >2Hz sebelum menerima mode switch.
    // Lakukan selama satu detik SETELAH marker ready, sambil PX4 tetap
    // Position mode dan pilot tetap punya kontrol. Setpoint selalu mengikuti
    // posisi/yaw aktual, sehingga takeover tidak meloncat ke anchor lama.
    control_->publishHeartbeat(true);
    control_->sendPositionSetpoint(
        toPx4North(vehicle_.position.north),
        toPx4East(vehicle_.position.east),
        origin_down_ + vehicle_.position.down,
        vehicle_.yaw);
    ++handoff_offboard_prestream_ticks_;
    if (handoff_offboard_prestream_ticks_ < 10) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
            "HANDOFF OFFBOARD PRESTREAM: %d/10, pilot masih Position mode.",
            handoff_offboard_prestream_ticks_);
        return;
    }

    // Lima frame awal mengaktifkan prestream, tetapi takeover satu detik
    // kemudian hanya boleh dilakukan bila marker MASIH terlihat. Ini
    // mencegah program mengambil alih berdasarkan sampel lama lalu langsung
    // kehilangan referensi centering.
    if (!valid_sample) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[PILOT-ARUCO] sample awal READY, tetapi marker sekarang tidak "
            "fresh; pilot tetap memegang kontrol sampai marker muncul lagi.");
        return;
    }

    // VehicleStatus/nav_state tidak tersedia pada bridge device ini (terbukti
    // di log penerbangan). Marker 5/5 menjadi trigger deterministik; kirim
    // OFFBOARD satu kali lalu mulai dari posisi aktual pilot agar tidak ada
    // tarikan kembali ke titik lama.
    control_->setOffboardMode();
    vehicle_.hover_position = vehicle_.position;
    handoff_takeover_anchor_ = vehicle_.position;
    marker_center_target_ = vehicle_.position;
    marker_search_altitude_m_ = currentAltitudeAgl();
    // Altitude pilot dibekukan saat takeover. N/E dimulai dari posisi pilot,
    // lalu dikoreksi kecil oleh center pixel marker besar bersamaan dengan
    // koreksi yaw dari TF sebelum altitude align.
    takeoff_hold_north_ = vehicle_.position.north;
    takeoff_hold_east_ = vehicle_.position.east;
    takeoff_hold_yaw_ = vehicle_.yaw;
    handoff_vision_aligned_ = false;
    initial_altitude_stable_ticks_ = 0;
    marker_stable_frames_ = 0;
    wp_hold_counter_ = 0;
    marker_feedback_locked_ = false;
    marker_heading_checked_ = false;
    marker_heading_correction_active_ = false;
    marker_tf_yaw_command_ = vehicle_.yaw;
    marker_tf_centered_ticks_ = 0;
    initial_centered_samples_ = 0;
    initial_position_locked_ = false;
    initial_position_locked_s_ = -1.0;
    initial_tf_correction_used_ = false;
    initial_tf_fallback_used_ = false;
    // Pertahankan sampel center dan TF valid terakhir dari pencarian pilot.
    // Takeover dipicu saat marker memang masih terlihat, jadi kedua sampel
    // ini merupakan seed terbaik dan mencegah menunggu deteksi serempak baru.
    waypoint_phase_ = WaypointPhase::ALIGN_INITIAL_MARKER_TF;
    phase_ = Phase::TAKEOFF_MARKER;
    RCLCPP_WARN(this->get_logger(),
        "=== PILOT -> OFFBOARD AUTO TAKEOVER === [AUTO-ARUCO] program "
        "HOLD POSISI+ALTITUDE pilot; hanya YAW dikoreksi TF ArUco; anchor "
        "N=%.3f E=%.3f D=%.3f yaw=%.1fdeg.",
        handoff_takeover_anchor_.north, handoff_takeover_anchor_.east,
        handoff_takeover_anchor_.down, radToDeg(takeoff_hold_yaw_));
}

// ==================================================================
// TAKEOFF_MARKER — marker pertama wajib ditemukan tepat setelah takeoff
// ==================================================================

void MissionManager::runTakeoffMarker()
{
    control_->publishHeartbeat(true);

    if (!vision_lock_enable_) {
        RCLCPP_WARN(this->get_logger(),
            "Vision lock disabled — melewati fase pencarian/centering ArUco setelah takeoff.");
        resetWaypointVisionState();
        rebaseMissionOriginToCurrentPosition();
        phase_ = Phase::MISSION;
        return;
    }

    const bool is_handoff = start_mode_ == StartMode::AIRBORNE_HANDOFF;
    const double base_altitude_m = is_handoff
        ? marker_search_altitude_m_
        : -takeoffTargetDown();
    // Pada handoff, yaw takeover dan anchor horizontal harus tetap menjadi
    // referensi sampai pasangan marker siap melakukan alignment heading.
    const double heading = is_handoff ? takeoff_hold_yaw_ : missionReferenceYaw();

    if (waypoint_phase_ == WaypointPhase::ALIGN_INITIAL_MARKER_TF) {
        // Jalur ini hanya dimasuki dari PILOT_ARUCO_SEARCH pada handoff awal.
        // Tidak digunakan oleh ArUco waypoint berikutnya.
        const double now_s = this->now().seconds();
        // Center pixel dan pose TF datang dari dua topic/callback berbeda.
        // Detector juga mengirim array center kosong saat satu frame miss.
        // Karena itu gunakan latch sampel VALID masing-masing selama jendela
        // pendek, bukan mewajibkan keduanya tiba pada tick 100 ms yang sama.
        constexpr double VALID_SAMPLE_HOLD_S = 1.20;
        const double tf_age_s = now_s - last_marker_tf_yaw_s_;
        const bool recent_tf = is_handoff &&
            marker_tf_yaw_filter_ready_ &&
            last_marker_tf_yaw_s_ >= 0.0 &&
            tf_age_s <= VALID_SAMPLE_HOLD_S;
        // Urutan deterministik: dua sampel pixel yang sudah di tengah
        // membekukan posisi; sesudah itu paling banyak SATU koreksi TF.
        // Nilai TF besar dianggap outlier agar drone tidak yaw liar.
        constexpr double LOCK_CENTER_TOLERANCE_PX = 70.0;
        constexpr int REQUIRED_CENTER_SAMPLES = 2;
        constexpr double MAX_TF_ONE_SHOT_RAD = 10.0 * PI / 180.0;
        constexpr double YAW_COMMAND_TOLERANCE_RAD = 2.5 * PI / 180.0;
        constexpr double TF_FALLBACK_GRACE_S = 0.50;
        constexpr double CENTER_KP = 0.30;
        constexpr double MAX_CENTER_STEP_M = 0.030;
        constexpr int REQUIRED_LOCK_TICKS = 3;

        const double center_age_s = now_s - last_initial_marker_center_s_;
        const bool recent_center =
            initial_marker_center_available_ &&
            last_initial_marker_center_s_ >= 0.0 &&
            center_age_s <= VALID_SAMPLE_HOLD_S &&
            initial_marker_frame_width_px_ > 0.0 &&
            initial_marker_frame_height_px_ > 0.0;
        const bool new_center = recent_center && initial_marker_center_new_;

        double anchor_position_error_m = std::numeric_limits<double>::infinity();
        double target_position_error_m = std::numeric_limits<double>::infinity();
        double yaw_error_rad = std::numeric_limits<double>::infinity();
        double pixel_error_x = std::numeric_limits<double>::infinity();
        double pixel_error_y = std::numeric_limits<double>::infinity();
        bool marker_inside_center_box = false;
        double yaw_command_error_rad = std::numeric_limits<double>::infinity();

        // Tahap 1 — centering pixel. Begitu dua sampel BARU berturut-turut
        // berada di kotak, bekukan satu target N/E dan jangan pernah lagi
        // menggesernya pada fase ini. Ini position hold sungguhan; target
        // tidak ikut berjalan saat estimasi/drone drift.
        if (recent_center) {
            const double frame_cx =
                initial_marker_frame_width_px_ * 0.5;
            const double frame_cy =
                initial_marker_frame_height_px_ * 0.5;
            pixel_error_x = initial_marker_center_x_px_ - frame_cx;
            pixel_error_y = initial_marker_center_y_px_ - frame_cy;
            marker_inside_center_box =
                std::abs(pixel_error_x) <= LOCK_CENTER_TOLERANCE_PX &&
                std::abs(pixel_error_y) <= LOCK_CENTER_TOLERANCE_PX;

            if (!initial_position_locked_ && new_center) {
                if (marker_inside_center_box) {
                    ++initial_centered_samples_;
                    if (initial_centered_samples_ == 1) {
                        // Sampel center pertama menjadi anchor tetap.
                        marker_center_target_ = vehicle_.position;
                    }
                    if (initial_centered_samples_ >= REQUIRED_CENTER_SAMPLES) {
                        initial_position_locked_ = true;
                        initial_position_locked_s_ = now_s;
                        RCLCPP_WARN(this->get_logger(),
                            "[AUTO-ARUCO][POSITION LOCKED] %d sampel CENTER; "
                            "anchor N=%.3f E=%.3f dibekukan, centering pixel selesai.",
                            initial_centered_samples_,
                            marker_center_target_.north,
                            marker_center_target_.east);
                    }
                } else {
                    initial_centered_samples_ = 0;

                    // Satu sampel off-center menghasilkan tepat satu langkah
                    // koreksi kecil; tidak pernah memakai sampel lama dua kali.
                    const double altitude_m = std::max(0.30, currentAltitudeAgl());
                    const double optical_right_m =
                        pixel_error_x / yolo_camera_fx_px_ * altitude_m;
                    const double optical_up_m =
                        -pixel_error_y / yolo_camera_fy_px_ * altitude_m;
                    const double mount = degToRad(camera_mount_yaw_deg_);
                    const double body_forward =
                        optical_up_m * std::cos(mount) -
                        optical_right_m * std::sin(mount);
                    const double body_right =
                        optical_up_m * std::sin(mount) +
                        optical_right_m * std::cos(mount);
                    const double step_forward = std::clamp(
                        CENTER_KP * body_forward,
                        -MAX_CENTER_STEP_M, MAX_CENTER_STEP_M);
                    const double step_right = std::clamp(
                        CENTER_KP * body_right,
                        -MAX_CENTER_STEP_M, MAX_CENTER_STEP_M);
                    marker_center_target_.north = vehicle_.position.north +
                        step_forward * std::cos(vehicle_.yaw) -
                        step_right * std::sin(vehicle_.yaw);
                    marker_center_target_.east = vehicle_.position.east +
                        step_forward * std::sin(vehicle_.yaw) +
                        step_right * std::cos(vehicle_.yaw);
                    marker_center_target_.down = handoff_takeover_anchor_.down;
                }
            }
        }

        // Tahap 2 — satu tembakan TF sesudah posisi sudah dibekukan.
        // Gunakan error penuh (bukan KP berulang); setelah target yaw dibuat,
        // semua pose TF berikutnya diabaikan pada alignment awal ini.
        if (initial_position_locked_ && !initial_tf_correction_used_ && recent_tf) {
            yaw_error_rad = marker_latest_tf_yaw_error_rad_;
            if (std::abs(yaw_error_rad) <= MAX_TF_ONE_SHOT_RAD) {
                marker_tf_yaw_command_ = std::atan2(
                    std::sin(vehicle_.yaw + yaw_error_rad),
                    std::cos(vehicle_.yaw + yaw_error_rad));
                initial_tf_correction_used_ = true;
                initial_tf_fallback_used_ = false;
                RCLCPP_WARN(this->get_logger(),
                    "[AUTO-ARUCO][TF ONE-SHOT] error=%+.1fdeg -> target yaw=%.1fdeg; "
                    "target sekarang dikunci, TF tidak dihitung ulang.",
                    radToDeg(yaw_error_rad), radToDeg(marker_tf_yaw_command_));
            } else {
                // Pose besar tidak dipercaya. Tetap kunci yaw pilot dan biarkan
                // fallback deterministik menyelesaikan fase tanpa yaw liar.
                initial_tf_correction_used_ = true;
                initial_tf_fallback_used_ = true;
                marker_tf_yaw_command_ = takeoff_hold_yaw_;
                RCLCPP_ERROR(this->get_logger(),
                    "[AUTO-ARUCO][TF REJECTED] error=%+.1fdeg melebihi 10deg; "
                    "yaw pilot %.1fdeg dipertahankan.",
                    radToDeg(yaw_error_rad), radToDeg(takeoff_hold_yaw_));
            }
        }

        // Bila detector tidak pernah mem-publish /fiducial/pose (terjadi di
        // dua log terbaru walau marker_centers ramai), jangan menunggu tanpa
        // batas. Setelah posisi terkunci 0,5 s, yaw pilot menjadi target final.
        if (initial_position_locked_ && !initial_tf_correction_used_ &&
            !recent_tf && initial_position_locked_s_ >= 0.0 &&
            now_s - initial_position_locked_s_ >= TF_FALLBACK_GRACE_S)
        {
            initial_tf_correction_used_ = true;
            initial_tf_fallback_used_ = true;
            marker_tf_yaw_command_ = takeoff_hold_yaw_;
            RCLCPP_WARN(this->get_logger(),
                "[AUTO-ARUCO][TF FALLBACK] /fiducial/pose tidak fresh; "
                "yaw pilot %.1fdeg dikunci, tanpa pencarian/yaw tambahan.",
                radToDeg(takeoff_hold_yaw_));
        }

        if (recent_tf) {
            yaw_error_rad = marker_latest_tf_yaw_error_rad_;
        }
        if (initial_tf_correction_used_) {
            yaw_command_error_rad = std::abs(std::atan2(
                std::sin(marker_tf_yaw_command_ - vehicle_.yaw),
                std::cos(marker_tf_yaw_command_ - vehicle_.yaw)));
        }

        anchor_position_error_m = std::hypot(
            handoff_takeover_anchor_.north - vehicle_.position.north,
            handoff_takeover_anchor_.east - vehicle_.position.east);
        target_position_error_m = std::hypot(
            marker_center_target_.north - vehicle_.position.north,
            marker_center_target_.east - vehicle_.position.east);
        const double horizontal_speed_m_s = std::hypot(
            vehicle_.velocity.north, vehicle_.velocity.east);
        // Dua sampel visual sudah menjadi bukti posisi. Jangan menunggu
        // estimator kembali ke anchor lama—log menunjukkan syarat itu menahan
        // drone 6--13 detik dan justru menarik marker keluar dari tengah.
        // Redaman posisi/laju dilakukan pada hover 3 detik sesudah fase ini.
        const bool position_ready = initial_position_locked_;
        const bool heading_ready = initial_tf_correction_used_ &&
            yaw_command_error_rad <= YAW_COMMAND_TOLERANCE_RAD;
        const bool complete_lock = position_ready && heading_ready;
        marker_tf_centered_ticks_ = complete_lock
            ? marker_tf_centered_ticks_ + 1 : 0;

        const char * heading_source = !initial_tf_correction_used_
            ? "WAIT"
            : (initial_tf_fallback_used_ ? "PILOT" : "TF-ONE-SHOT");

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250,
            "[AUTO-ARUCO][FIRST ONLY] center_px=(%+.0f,%+.0f) box=%s "
            "pos=%s samples=%d/%d target_err=%.3fm speed=%.3fm/s | "
            "heading=%s tf_age=%.2fs tf_err=%+.1fdeg cmd_err=%.1fdeg | "
            "alt=%.2f/%.2fm anchor_drift=%.3fm lock=%d/%d",
            pixel_error_x, pixel_error_y,
            marker_inside_center_box ? "CENTER" : "ADJUST",
            initial_position_locked_ ? "LOCKED" : "CENTERING",
            initial_centered_samples_, REQUIRED_CENTER_SAMPLES,
            target_position_error_m, horizontal_speed_m_s,
            heading_source, tf_age_s, radToDeg(yaw_error_rad),
            radToDeg(yaw_command_error_rad), currentAltitudeAgl(),
            marker_search_altitude_m_, anchor_position_error_m,
            marker_tf_centered_ticks_,
            REQUIRED_LOCK_TICKS);
        if (!initial_position_locked_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "[AUTO-ARUCO][CENTERING] sampel center=%d/%d; "
                "center_recent=%s age=%.2fs. Target N/E tetap, tidak sweep.",
                initial_centered_samples_,
                REQUIRED_CENTER_SAMPLES,
                recent_center ? "yes" : "no", center_age_s);
        } else if (!initial_tf_correction_used_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "[AUTO-ARUCO][HEADING] posisi sudah LOCKED; tunggu TF hanya "
                "%.1fs lalu gunakan yaw pilot. tf_recent=%s age=%.2fs.",
                TF_FALLBACK_GRACE_S, recent_tf ? "yes" : "no", tf_age_s);
        } else if (!complete_lock) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "[AUTO-ARUCO][SETTLE] anchor tetap; target_err=%.3fm "
                "speed=%.3fm/s yaw_cmd_err=%.1fdeg.",
                target_position_error_m, horizontal_speed_m_s,
                radToDeg(yaw_command_error_rad));
        }

        // Selama menyamakan TF, tahan ketinggian AGL yang tercatat saat
        // takeover. Setpoint Z EKF tetap menjadi fallback jika TFmini stale;
        // ini mencegah drift barometer membuat drone turun perlahan saat
        // menunggu pose visual.
        double altitude_setpoint_down =
            origin_down_ + handoff_takeover_anchor_.down;
        if (use_lidar_altitude_ && got_lidar_altitude_ && !isLidarStale(0.5)) {
            altitude_setpoint_down =
                toPx4DownForAltitudeTarget(-marker_search_altitude_m_);
        }
        control_->sendPositionSetpoint(
            toPx4North(marker_center_target_.north),
            toPx4East(marker_center_target_.east),
            altitude_setpoint_down,
            marker_tf_yaw_command_);
        initial_marker_center_new_ = false;
        marker_latest_sample_available_ = false;
        marker_latest_tf_yaw_available_ = false;

        if (marker_tf_centered_ticks_ < REQUIRED_LOCK_TICKS) {
            return;
        }

        // Bekukan posisi visual yang benar-benar centered, bukan koordinat
        // takeover lama. Altitude tetap memakai altitude pilot hingga tahap
        // alignment ketinggian berikutnya.
        // Capture posisi aktual tepat setelah yaw mencapai target. Anchor
        // sampel center pertama hanya dipakai selama koreksi singkat; memakai
        // kembali anchor estimator lama di sini dapat menyeret gambar keluar
        // dari pusat walaupun TF/center sudah benar.
        takeoff_hold_north_ = vehicle_.position.north;
        takeoff_hold_east_ = vehicle_.position.east;
        takeoff_hold_yaw_ = marker_tf_yaw_command_;
        marker_center_target_.north = takeoff_hold_north_;
        marker_center_target_.east = takeoff_hold_east_;
        marker_center_target_.down = handoff_takeover_anchor_.down;
        origin_yaw_ = takeoff_hold_yaw_;
        override_mission_heading_ = true;
        mission_heading_deg_ = radToDeg(takeoff_hold_yaw_);
        mission_heading_correction_deg_ = 0.0;
        mission_waypoints_ready_ = false;
        ensureMissionWaypointsReady();
        handoff_vision_aligned_ = true;
        initial_altitude_stable_ticks_ = 0;
        marker_tf_centered_ticks_ = 0;
        phase_ = Phase::TAKEOFF;
        RCLCPP_WARN(this->get_logger(),
            "=== INITIAL ARUCO POSITION+HEADING LOCKED === center anchor tetap; "
            "heading_source=%s yaw=%.1fdeg; lanjut altitude align lalu misi.",
            heading_source, radToDeg(takeoff_hold_yaw_));
        return;
    }

    if (waypoint_phase_ == WaypointPhase::SEARCH_MARKER) {
        // Marker memang ditempatkan di sekitar titik takeoff. Tahan anchor
        // selama 5 detik, lalu sapu hanya pada sumbu depan-belakang. Tidak ada
        // gerak samping dan tidak ada perubahan altitude.
        const double search_s = (this->now() - marker_search_started_at_).seconds();
        const double active_s = std::max(0.0, search_s - 5.0);
        constexpr double SEARCH_AMPLITUDE_M = 0.05;
        constexpr double SEARCH_ANGULAR_RATE_RAD_S = 0.25;
        const double forward_displacement = SEARCH_AMPLITUDE_M *
            std::sin(active_s * SEARCH_ANGULAR_RATE_RAD_S);
        marker_search_altitude_m_ = base_altitude_m;
        const double search_anchor_n = is_handoff
            ? takeoff_hold_north_
            : vehicle_.hover_position.north;
        const double search_anchor_e = is_handoff
            ? takeoff_hold_east_
            : vehicle_.hover_position.east;
        const double search_n = search_anchor_n +
            forward_displacement * std::cos(heading);
        const double search_e = search_anchor_e +
            forward_displacement * std::sin(heading);
        control_->sendPositionSetpoint(
            toPx4North(search_n), toPx4East(search_e),
            toPx4DownForAltitudeTarget(-marker_search_altitude_m_), heading);

        const double marker_age_s = this->now().seconds() - last_marker_sample_s_;
        const bool valid_sample = marker_latest_sample_available_ &&
            marker_age_s <= 0.35 &&
            std::hypot(marker_latest_offset_north_, marker_latest_offset_east_) <= 0.80;
        marker_stable_frames_ = valid_sample ? marker_stable_frames_ + 1 : 0;
        marker_latest_sample_available_ = false;
        if (marker_stable_frames_ < 5) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[WP1-%s] mencari ArUco: %.1fs alt tetap %.2fm sweep_fb=%+.3fm valid=%d/5",
                is_handoff ? "HANDOFF" : "TAKEOFF",
                search_s, marker_search_altitude_m_,
                forward_displacement, marker_stable_frames_);
            return;
        }
        waypoint_phase_ = WaypointPhase::CENTER_MARKER;
        latched_marker_target_ = vision_lock_->lockedTarget();
        marker_target_latched_ = true;
        vision_engage_ticks_ = 0;
        wp_hold_counter_ = 0;
        marker_stable_frames_ = 0;
        marker_center_target_ = vehicle_.position;
        RCLCPP_INFO(this->get_logger(),
            "[WP1-TAKEOFF] ARUCO VALID (5 sample) - centering.");
    }

    if (waypoint_phase_ == WaypointPhase::CENTER_MARKER) {
        double target_n = vehicle_.hover_position.north;
        double target_e = vehicle_.hover_position.east;
        // Handoff wajib mempertahankan altitude tempat pilot menemukan
        // marker. Penyesuaian ke altitude misi baru dilakukan setelah
        // centering + heading selesai.
        double target_d = is_handoff
            ? -marker_search_altitude_m_
            : takeoffTargetDown();

        if (!vision_lock_enable_) {
            resetWaypointVisionState();
            rebaseMissionOriginToCurrentPosition();
            phase_ = Phase::MISSION;
            RCLCPP_INFO(this->get_logger(), "=== START MISSION (vision lock disabled) ===");
            return;
        }

        const double marker_age_s = this->now().seconds() - last_marker_sample_s_;
        if (marker_latest_sample_available_ && marker_age_s <= 0.35) {
            constexpr double CENTER_KP = 0.18;
            constexpr double MAX_CENTER_STEP_M = 0.03;
            const double correction_n = std::max(-MAX_CENTER_STEP_M,
                std::min(MAX_CENTER_STEP_M, CENTER_KP * marker_latest_offset_north_));
            const double correction_e = std::max(-MAX_CENTER_STEP_M,
                std::min(MAX_CENTER_STEP_M, CENTER_KP * marker_latest_offset_east_));

            // Target selalu hanya satu langkah kecil dari posisi aktual.
            // Dengan demikian offset ditutup perlahan secara closed-loop,
            // bukan melompat langsung ke estimasi marker yang noisy.
            const double measured_offset = std::hypot(
                marker_latest_offset_north_, marker_latest_offset_east_);
            // Di dalam toleransi jangan ubah target: deadband mencegah noise
            // pose diintegrasikan menjadi drift satu arah.
            if (!marker_feedback_locked_ &&
                measured_offset > marker_center_tolerance_m_)
            {
                marker_center_target_.north = vehicle_.position.north + correction_n;
                marker_center_target_.east  = vehicle_.position.east + correction_e;
            }
            // Jangan biarkan satu pose marker membawa target keluar jauh
            // dari area waypoint. Radius ini juga membantu menangkap
            // overshoot/inersia setelah sweep pencarian berhenti.
            constexpr double MAX_CENTER_RADIUS_M = 0.45;
            const double center_dn =
                marker_center_target_.north - vehicle_.hover_position.north;
            const double center_de =
                marker_center_target_.east - vehicle_.hover_position.east;
            const double center_dist = std::hypot(center_dn, center_de);
            if (center_dist > MAX_CENTER_RADIUS_M) {
                const double scale = MAX_CENTER_RADIUS_M / center_dist;
                marker_center_target_.north =
                    vehicle_.hover_position.north + center_dn * scale;
                marker_center_target_.east =
                    vehicle_.hover_position.east + center_de * scale;
            }
            target_n = marker_center_target_.north;
            target_e = marker_center_target_.east;

            const double horizontal_offset = std::hypot(
                marker_latest_offset_north_, marker_latest_offset_east_);
            const double lock_tolerance_m = std::max(
                marker_center_tolerance_m_, 0.15);
            const bool within_threshold =
                horizontal_offset <= lock_tolerance_m;
            if (within_threshold) {
                ++marker_stable_frames_;
                RCLCPP_INFO(this->get_logger(),
                    "[ARUCO] center terkonfirmasi=%d/6 | offset=%.3fm <= batas %.3fm (N=%.3f E=%.3f)",
                    marker_stable_frames_, horizontal_offset,
                    lock_tolerance_m,
                    marker_latest_offset_north_, marker_latest_offset_east_);
            } else {
                marker_stable_frames_ = std::max(0, marker_stable_frames_ - 1);
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "[ARUCO] offset N=%.3f E=%.3f | langkah N=%.3f E=%.3f",
                    marker_latest_offset_north_, marker_latest_offset_east_,
                    correction_n, correction_e);
            }

            marker_latest_sample_available_ = false;
            if (marker_stable_frames_ >= 6 && !marker_feedback_locked_) {
                marker_feedback_locked_ = true;
                // Hentikan integrasi koreksi ArUco tepat saat lock. Sisa
                // target satu langkah di depan dapat membawa momentum
                // lateral ke awal WP1 sehingga drone terlihat bergeser ke
                // kanan dahulu sebelum maju.
                marker_center_target_ = vehicle_.position;
                target_n = marker_center_target_.north;
                target_e = marker_center_target_.east;
                wp_hold_counter_ = 0;
                RCLCPP_INFO(this->get_logger(), "[ARUCO] LOCK SUCCESS");
            }
        } else {
            target_n = marker_center_target_.north;
            target_e = marker_center_target_.east;

            // Sama seperti fix di runMission(): marker hilang dari frame
            // terlalu lama (mis. kedorong angin) tidak boleh menahan target
            // terakhir selamanya — kembali ke SEARCH_MARKER supaya drone
            // aktif mencari lagi (radius tetap maksimum 0.02m dan altitude
            // tetap, sama seperti blok SEARCH_MARKER di atas).
            constexpr double MARKER_LOST_RESEARCH_S = 3.0;
            if (marker_age_s > MARKER_LOST_RESEARCH_S) {
                RCLCPP_WARN(this->get_logger(),
                    "[ARUCO] marker hilang > %.1fs - kembali ke SEARCH, mencari ulang.",
                    MARKER_LOST_RESEARCH_S);
                waypoint_phase_ = WaypointPhase::SEARCH_MARKER;
                marker_search_started_at_ = this->now();
                marker_stable_frames_ = 0;
                marker_latest_sample_available_ = false;
                control_->sendPositionSetpoint(
                    toPx4North(target_n), toPx4East(target_e),
                    toPx4DownForAltitudeTarget(target_d), heading);
                return;
            }

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[ARUCO] menunggu sampel segar (age=%.2fs), tahan target terakhir.",
                marker_age_s);
        }

        control_->sendPositionSetpoint(
            toPx4North(target_n), toPx4East(target_e),
            toPx4DownForAltitudeTarget(target_d), heading);

        if (marker_feedback_locked_) {
            // Jangan langsung rebase/start mission ketika badan masih
            // bergerak akibat centering. Tahan titik lock sampai laju XY
            // rendah DAN posisi kembali dekat target. Tanpa syarat posisi,
            // hembusan angin bisa membuat speed sesaat rendah di lokasi yang
            // sudah bergeser lalu dianggap stabil.
            constexpr double ARUCO_LOCK_SETTLE_SPEED_M_S = 0.08;
            constexpr double ARUCO_LOCK_SETTLE_POSITION_M = 0.08;
            constexpr int ARUCO_LOCK_SETTLE_TICKS = 5;
            const double horizontal_speed = std::hypot(
                vehicle_.velocity.north, vehicle_.velocity.east);
            const double lock_position_error = std::hypot(
                marker_center_target_.north - vehicle_.position.north,
                marker_center_target_.east - vehicle_.position.east);
            if (horizontal_speed <= ARUCO_LOCK_SETTLE_SPEED_M_S &&
                lock_position_error <= ARUCO_LOCK_SETTLE_POSITION_M)
            {
                ++wp_hold_counter_;
            } else {
                wp_hold_counter_ = 0;
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                "[ARUCO] LOCK SETTLE: pos_err=%.3fm speed_xy=%.3fm/s "
                "stable=%d/%d",
                lock_position_error, horizontal_speed,
                wp_hold_counter_, ARUCO_LOCK_SETTLE_TICKS);
            if (wp_hold_counter_ < ARUCO_LOCK_SETTLE_TICKS) {
                return;
            }

            // Airborne handoff memakai yaw yang sudah dipilih pilot sebagai
            // yaw awal. Jangan lakukan putar tambahan dari pasangan ArUco:
            // gate Livox berikutnya yang akan menyempurnakan heading ke
            // normal gate lalu menguncinya untuk maju lurus.
            if (start_mode_ == StartMode::AIRBORNE_HANDOFF) {
                RCLCPP_INFO(this->get_logger(),
                    "[MISSION][HANDOFF] ArUco position lock + stabilisasi selesai; "
                    "pertahankan yaw pilot, heading dikoreksi nanti oleh gate Livox.");
            }
            resetWaypointVisionState();
            rebaseMissionOriginToCurrentPosition();
            phase_ = Phase::MISSION;
            RCLCPP_INFO(this->get_logger(), "=== START MISSION (ArUco locked) ===");
        }
    }

    if (waypoint_phase_ == WaypointPhase::ALIGN_MARKER_HEADING) {
        control_->publishHeartbeat(true);

        // Koreksi heading sedang berjalan: satu putaran tegas, accel-limited
        // (profil identik turn antar-waypoint di runMission()), memutar di
        // tempat sambil menahan posisi hasil centering. Tidak ada logic lain
        // yang boleh jalan sampai putaran ini selesai — marker akan sedikit
        // bergeser di frame selama badan berputar, itu ditangani oleh
        // re-centering normal begitu fase ini balik ke pengecekan posisi.
        if (marker_heading_correction_active_) {
            constexpr double YAW_DT_S = 0.10;
            // Lebih cepat dari turn antar-leg (0.35 rad/s) — ini koreksi
            // kecil (dibatasi ±45deg) yang harus selesai tegas, bukan turn
            // besar antar-waypoint yang butuh mulus untuk jarak jauh.
            constexpr double YAW_MAX_RATE_RAD_S = 0.45;
            constexpr double YAW_ACCEL_RAD_S2 = 1.2;
            const double remaining_command = std::max(0.0,
                yaw_profile_direction_ *
                (yaw_target_unwrapped_ - yaw_command_unwrapped_));
            const double stopping_rate = std::sqrt(
                2.0 * YAW_ACCEL_RAD_S2 * remaining_command);
            const double desired_rate = std::min(YAW_MAX_RATE_RAD_S, stopping_rate);
            const double max_rate_step = YAW_ACCEL_RAD_S2 * YAW_DT_S;
            yaw_profile_rate_rad_s_ += std::clamp(
                desired_rate - yaw_profile_rate_rad_s_,
                -max_rate_step, max_rate_step);
            const double command_step = yaw_profile_rate_rad_s_ * YAW_DT_S;
            const bool command_reached = remaining_command <= command_step + 1e-6;
            if (command_reached) {
                yaw_command_unwrapped_ = yaw_target_unwrapped_;
                yaw_profile_rate_rad_s_ = 0.0;
            } else {
                yaw_command_unwrapped_ += yaw_profile_direction_ * command_step;
            }
            const double yaw_command = std::atan2(
                std::sin(yaw_command_unwrapped_), std::cos(yaw_command_unwrapped_));

            control_->sendPositionSetpoint(
                toPx4North(marker_center_target_.north),
                toPx4East(marker_center_target_.east),
                origin_down_ + handoff_takeover_anchor_.down,
                yaw_command);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                "[ARUCO-HEADING] KOREKSI TEGAS: cmd=%.1fdeg target=%.1fdeg sisa=%.1fdeg",
                radToDeg(yaw_command), radToDeg(yaw_target_unwrapped_),
                radToDeg(remaining_command));

            if (command_reached) {
                takeoff_hold_yaw_ = yaw_command;
                marker_heading_correction_active_ = false;
                marker_heading_checked_ = true;
                marker_heading_aligned_ticks_ = 0;
                RCLCPP_WARN(this->get_logger(),
                    "[ARUCO-HEADING] Koreksi %.1fdeg selesai -> heading baru %.1fdeg; "
                    "verifikasi ulang center sebelum lanjut.",
                    radToDeg(marker_heading_error_rad_), radToDeg(yaw_command));
            }
            return;
        }

        auto find_marker = [this](int id) -> const ControlModule::MarkerCenter * {
            for (const auto & marker : marker_centers_latest_.markers) {
                if (marker.id == id) {
                    return &marker;
                }
            }
            return nullptr;
        };

        const double now_s = this->now().seconds();
        const double align_s = now_s - marker_heading_align_started_s_;
        const double marker_centers_age_s = now_s - last_marker_centers_s_;
        const auto * back_marker = find_marker(marker_heading_back_id_);
        if (!back_marker && marker_centers_latest_.markers.size() == 1) {
            back_marker = &marker_centers_latest_.markers.front();
        }
        const bool fresh_centers =
            marker_centers_available_ && marker_centers_age_s <= 0.35;
        const bool have_large_marker = fresh_centers && back_marker;

        const double yaw_command = takeoff_hold_yaw_;
        double heading_hold_n = marker_center_target_.north;
        double heading_hold_e = marker_center_target_.east;
        bool position_locked = false;
        double center_offset_m = std::numeric_limits<double>::infinity();
        bool body_centered = false;
        marker_latest_sample_available_ = false;
        if (have_large_marker) {
            const double frame_cx = marker_centers_latest_.frame_width_px * 0.5;
            const double frame_cy = marker_centers_latest_.frame_height_px * 0.5;
            const double altitude_m = std::max(0.30, currentAltitudeAgl());
            const double optical_right_m =
                (back_marker->x_px - frame_cx) / yolo_camera_fx_px_ * altitude_m;
            const double optical_up_m =
                -(back_marker->y_px - frame_cy) / yolo_camera_fy_px_ * altitude_m;

            const double mount = degToRad(camera_mount_yaw_deg_);
            // Koreksi kedua sumbu dari pusat piksel marker besar. Sumbu maju
            // dibuat lebih lembut daripada lateral agar marker tidak hilang
            // akibat tarikan mundur, namun pusat vertikal gambar tetap bisa
            // disempurnakan dan bukan sekadar kanan/kiri.
            const double body_forward =
                optical_up_m * std::cos(mount) -
                optical_right_m * std::sin(mount);
            const double body_right =
                optical_up_m * std::sin(mount) +
                optical_right_m * std::cos(mount);
            center_offset_m = std::hypot(body_forward, body_right);
            constexpr double BODY_CENTER_TOLERANCE_M = 0.08;
            body_centered = center_offset_m <= BODY_CENTER_TOLERANCE_M;

            if (!body_centered) {
                constexpr double LATERAL_KP = 0.22;
                constexpr double FORWARD_KP = 0.10;
                constexpr double MAX_LATERAL_STEP_M = 0.025;
                constexpr double MAX_FORWARD_STEP_M = 0.010;
                const double step_forward = std::clamp(
                    FORWARD_KP * body_forward,
                    -MAX_FORWARD_STEP_M, MAX_FORWARD_STEP_M);
                const double step_right = std::clamp(
                    LATERAL_KP * body_right,
                    -MAX_LATERAL_STEP_M, MAX_LATERAL_STEP_M);
                heading_hold_n = vehicle_.position.north +
                    step_forward * std::cos(takeoff_hold_yaw_) -
                    step_right * std::sin(takeoff_hold_yaw_);
                heading_hold_e = vehicle_.position.east +
                    step_forward * std::sin(takeoff_hold_yaw_) +
                    step_right * std::cos(takeoff_hold_yaw_);

                // Semua koreksi dibatasi di sekitar takeover. Deteksi/noise
                // tidak boleh menyeret drone terus hingga keluar FOV.
                constexpr double MAX_TAKEOVER_RADIUS_M = 0.10;
                const double dn = heading_hold_n - handoff_takeover_anchor_.north;
                const double de = heading_hold_e - handoff_takeover_anchor_.east;
                const double radius = std::hypot(dn, de);
                if (radius > MAX_TAKEOVER_RADIUS_M) {
                    const double scale = MAX_TAKEOVER_RADIUS_M / radius;
                    heading_hold_n = handoff_takeover_anchor_.north + dn * scale;
                    heading_hold_e = handoff_takeover_anchor_.east + de * scale;
                }
                marker_center_target_.north = heading_hold_n;
                marker_center_target_.east = heading_hold_e;
            }

            marker_heading_aligned_ticks_ = body_centered
                ? marker_heading_aligned_ticks_ + 1 : 0;
            position_locked = marker_heading_aligned_ticks_ >= 3;

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                "[AUTO-ARUCO] program centering | error=%.3fm "
                "(forward=%.3f right=%.3f) stable=%d/3 yaw_hold=%.1fdeg",
                center_offset_m, body_forward, body_right, marker_heading_aligned_ticks_,
                radToDeg(takeoff_hold_yaw_));
        } else {
            marker_heading_aligned_ticks_ = 0;
            // Jangan terus mengejar target koreksi terakhir ketika marker
            // hilang. Bekukan langsung di posisi aktual agar drone tidak
            // mundur/menyamping keluar FOV.
            marker_center_target_ = vehicle_.position;
            heading_hold_n = vehicle_.position.north;
            heading_hold_e = vehicle_.position.east;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "[AUTO-ARUCO] PROGRAM HOLD: marker besar hilang; menunggu "
                "marker id %d atau satu marker tunggal (fresh=%d age=%.2fs).",
                marker_heading_back_id_,
                fresh_centers ? 1 : 0, marker_centers_age_s);
        }

        control_->sendPositionSetpoint(
            toPx4North(heading_hold_n),
            toPx4East(heading_hold_e),
            is_handoff
                ? origin_down_ + handoff_takeover_anchor_.down
                : toPx4DownForAltitudeTarget(takeoffTargetDown()),
            yaw_command);

        // Timeout hanya untuk peringatan. Jangan pernah mulai maju memakai
        // yaw terbaik jika badan belum centered ke marker besar.
        if (!position_locked && align_s >= marker_heading_timeout_s_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[AUTO-ARUCO] belum center setelah %.1fs; tetap HOLD anchor.", align_s);
        }
        if (position_locked) {
            // Posisi sudah terkunci ke marker besar. Sebelum finalize,
            // koreksi TEGAS SEKALI heading pilot terhadap arah depan ArUco
            // (garis marker belakang->depan), kalau errornya cukup besar.
            // Kalau sudah pas (dalam toleransi), lewati sama sekali — tidak
            // ada delay, tidak ada putar-putar percuma.
            if (marker_heading_align_enable_ && !marker_heading_checked_) {
                const auto * front_marker = find_marker(marker_heading_front_id_);
                const bool have_pair = fresh_centers && back_marker && front_marker &&
                    front_marker != back_marker &&
                    marker_heading_front_id_ != marker_heading_back_id_;
                if (!have_pair) {
                    RCLCPP_WARN(this->get_logger(),
                        "[ARUCO-HEADING] Marker depan id=%d tidak terlihat bersamaan "
                        "dengan marker belakang id=%d - lewati koreksi heading, "
                        "pakai heading pilot apa adanya.",
                        marker_heading_front_id_, marker_heading_back_id_);
                    marker_heading_checked_ = true;
                } else {
                    const double dx_px = front_marker->x_px - back_marker->x_px;
                    const double dy_px = front_marker->y_px - back_marker->y_px;
                    const double optical_right_delta = dx_px / yolo_camera_fx_px_;
                    const double optical_up_delta = -dy_px / yolo_camera_fy_px_;
                    const double mount = degToRad(camera_mount_yaw_deg_);
                    const double body_forward_delta =
                        optical_up_delta * std::cos(mount) -
                        optical_right_delta * std::sin(mount);
                    const double body_right_delta =
                        optical_up_delta * std::sin(mount) +
                        optical_right_delta * std::cos(mount);
                    // marker_heading_yaw_sign_: field-tunable, kalau mounting
                    // fisik/pemilihan id front-back membalik konvensi ini.
                    const double raw_error = std::atan2(
                        body_right_delta, body_forward_delta) *
                        (marker_heading_yaw_sign_ >= 0.0 ? 1.0 : -1.0);
                    marker_heading_error_rad_ = raw_error;
                    const double tol_rad = degToRad(marker_heading_tolerance_deg_);
                    RCLCPP_WARN(this->get_logger(),
                        "[ARUCO-HEADING] error arah depan ArUco vs heading pilot "
                        "= %.1fdeg (toleransi %.1fdeg)",
                        radToDeg(raw_error), marker_heading_tolerance_deg_);
                    if (std::abs(raw_error) <= tol_rad) {
                        RCLCPP_INFO(this->get_logger(),
                            "[ARUCO-HEADING] Error kecil - heading pilot sudah pas, "
                            "tidak dikoreksi.");
                        marker_heading_checked_ = true;
                    } else {
                        // Cap 45deg: pilot presumably sudah kasar benar, error
                        // sebesar ini lebih mungkin salah pairing id daripada
                        // heading nyata — tetap koreksi tapi jangan berputar liar.
                        constexpr double MAX_HEADING_CORRECTION_RAD = 0.7854;
                        const double clamped_error = std::clamp(
                            raw_error,
                            -MAX_HEADING_CORRECTION_RAD, MAX_HEADING_CORRECTION_RAD);
                        marker_heading_correction_active_ = true;
                        yaw_command_unwrapped_ = vehicle_.yaw;
                        yaw_target_unwrapped_ = vehicle_.yaw + clamped_error;
                        yaw_profile_direction_ = clamped_error < 0.0 ? -1.0 : 1.0;
                        yaw_profile_rate_rad_s_ = 0.0;
                        RCLCPP_WARN(this->get_logger(),
                            "[ARUCO-HEADING] Error %.1fdeg > toleransi - koreksi "
                            "tegas satu kali ke %.1fdeg.",
                            radToDeg(raw_error), radToDeg(yaw_target_unwrapped_));
                        return;
                    }
                }
            }

            const double reference_yaw = takeoff_hold_yaw_;
            origin_yaw_ = reference_yaw;
            override_mission_heading_ = true;
            mission_heading_deg_ = radToDeg(reference_yaw);
            mission_heading_correction_deg_ = 0.0;
            mission_waypoints_ready_ = false;
            ensureMissionWaypointsReady();

            if (is_handoff) {
                // Heading sudah sah; sekarang dan hanya sekarang selaraskan
                // altitude. Anchor horizontal/yaw memakai posisi hasil
                // centering agar altitude align tidak menarik drone pergi.
                handoff_vision_aligned_ = true;
                resetWaypointVisionState();
                rebaseMissionOriginToCurrentPosition();
                takeoff_hold_north_ = vehicle_.position.north;
                takeoff_hold_east_ = vehicle_.position.east;
                takeoff_hold_yaw_ = reference_yaw;
                initial_altitude_stable_ticks_ = 0;
                phase_ = Phase::TAKEOFF;
                RCLCPP_INFO(this->get_logger(),
                    "=== HANDOFF VISION ALIGNED: mulai altitude align ===");
                return;
            }

            resetWaypointVisionState();
            rebaseMissionOriginToCurrentPosition();
            phase_ = Phase::MISSION;
            RCLCPP_INFO(this->get_logger(),
                "=== START MISSION (ArUco besar centered + yaw pilot) ===");
        }
    }
}

// ==================================================================
// MISSION — kunjungi semua waypoint berurutan
// ==================================================================

void MissionManager::runMission()
{
    if (current_wp_ >= waypoints_->size()) {
        RCLCPP_INFO(this->get_logger(), "=== ALL WAYPOINT REACHED ===");
        phase_ = Phase::LAND_CMD;
        return;
    }

    const auto & wp = waypoints_->at(current_wp_);
    const std::string label = waypoints_->labelAt(current_wp_);

    // WP1 (index 0) pakai YOLO/GroundLock sebagai sumber centering ke box
    // sebelum gripper, WP lain (termasuk WP2) tetap ArUco/VisionLock —
    // lihat onTargetCenterUpdate()/onMarkerPoseUpdate() untuk sisi pengisian
    // marker_latest_*_, dan komentar ground_lock_ di mission_manager.h
    // untuk alasan keputusan desainnya (kamera nadir dipakai gantian, satu
    // box tanpa ID unik jadi cuma dipakai di satu WP).
    const bool yolo_centering_wp = current_wp_ == 0;
    const bool marker_source_enabled =
        yolo_centering_wp ? ground_lock_enable_ : vision_lock_enable_;
    const char * marker_source_label = yolo_centering_wp ? "YOLO" : "ARUCO";

    const double err_n = wp.n - vehicle_.position.north;
    const double err_e = wp.e - vehicle_.position.east;
    const double dist  = WaypointHandler::horizontalDistance(
        vehicle_.position.north, vehicle_.position.east, wp.n, wp.e);
    // Heading selama satu leg harus mengikuti perintah RelativePath, bukan
    // bearing sesaat dari posisi aktual ke target. Centering ArUco dapat
    // menggeser drone dari titik waypoint nominal; jika bearing sesaat yang
    // dipakai, perintah turnLeft(90) dapat berubah menjadi 70/80 derajat.
    // Ambil vektor waypoint sebelumnya -> waypoint sekarang agar yaw tetap
    // persis sesuai bentuk jalur yang diperintahkan.
    const double leg_start_n = current_wp_ == 0 ? 0.0 : waypoints_->at(current_wp_ - 1).n;
    const double leg_start_e = current_wp_ == 0 ? 0.0 : waypoints_->at(current_wp_ - 1).e;
    const double leg_dn = wp.n - leg_start_n;
    const double leg_de = wp.e - leg_start_e;
    const double planned_leg_dist = std::hypot(leg_dn, leg_de);
    const double raw_bearing = leg_bearing_override_valid_
        ? leg_bearing_override_
        : (planned_leg_dist > 0.30
            ? std::atan2(leg_de, leg_dn)
            : vehicle_.yaw);
    const double raw_yaw_error = std::atan2(
        std::sin(raw_bearing - vehicle_.yaw),
        std::cos(raw_bearing - vehicle_.yaw));
    const double along_track_remaining =
        err_n * std::cos(raw_bearing) + err_e * std::sin(raw_bearing);

    const double alt_error   = WaypointHandler::altitudeError(currentAltitudeDown(), wp.d);
    const bool   alt_reached = waypoints_->isAltitudeReached(alt_error);

    // vz selalu dihitung — tidak boleh diblok oleh fase memutar yaw
    const double vz = waypoints_->computeVerticalVelocity(alt_error);

    // Beri computeYaw sebuah target virtual pada heading leg. Jarak virtual
    // dibuat > YAW_FREEZE_RADIUS supaya smoothing tidak membekukan yaw lama.
    const double yaw_target_n =
        vehicle_.position.north + 2.0 * std::cos(raw_bearing);
    const double yaw_target_e =
        vehicle_.position.east + 2.0 * std::sin(raw_bearing);
    const auto yaw_result = waypoints_->computeYaw(
        vehicle_.position.north, vehicle_.position.east, vehicle_.yaw,
        yaw_target_n, yaw_target_e);

    const bool is_pure_alt = waypoints_->isPureAltitude(dist);

    RCLCPP_INFO(this->get_logger(),
        "[%s] Pos N=%.2f E=%.2f -> Target N=%.2f E=%.2f | Dist=%.2fm | "
        "Alt[%s]=%.2f->%.2fm | Yaw tgt=%.1fdeg err=%.1fdeg align=%.2f%s",
        label.c_str(),
        vehicle_.position.north, vehicle_.position.east,
        wp.n, wp.e,
        dist,
        altitudeSourceLabel(),
        currentAltitudeAgl(), -wp.d,
        radToDeg(yaw_result.target_yaw), radToDeg(yaw_result.yaw_error),
        waypoints_->yawAlignmentFactor(yaw_result.yaw_error),
        is_pure_alt ? " [CLIMB]" : "");

    // ── YOLO approach interlock ─────────────────────────────────────
    // Box adalah tujuan fisik yang lebih sah daripada jarak nominal 4,9 m.
    // Begitu SATU frame valid muncul, hentikan velocity-forward seketika dan
    // pindahkan state ke CENTER_MARKER. Log penerbangan menunjukkan box hanya
    // tampak beberapa frame singkat; menunggu pasangan frame pada tick misi
    // berikutnya membuat drone keburu melewati box.
    // Altitude tetap dapat menyesuaikan lewat position setpoint selama
    // centering, tetapi gerak maju mission sudah tidak boleh aktif lagi.
    if (yolo_centering_wp && !yolo_approach_interlock_consumed_ &&
        along_track_remaining <= 1.30)
    {
        const double yolo_age_s =
            this->now().seconds() - last_marker_sample_s_;
        const double yolo_offset_m = std::hypot(
            marker_latest_offset_north_, marker_latest_offset_east_);
        // marker_latest_sample_available_ adalah flag konsumsi SAMPLE dan
        // dapat dibersihkan state lain di antara callback kamera dan tick
        // misi. Untuk rem APPROACH gunakan marker_feedback_active_ yang
        // persisten + umur sampel: satu YOLO-RX segar tidak boleh terlewat
        // hanya karena flag event satu-tick sudah dikonsumsi.
        const bool fresh_box =
            marker_feedback_active_ &&
            last_marker_sample_s_ >= 0.0 &&
            yolo_age_s <= 0.80 &&
            yolo_offset_m <= 0.90;

        if (fresh_box) {
            yolo_approach_interlock_consumed_ = true;
            ++marker_stable_frames_;
            marker_latest_sample_available_ = false;

            // Frame pertama pun langsung mengerem di posisi aktual. Jangan
            // beri satu tick velocity maju tambahan sambil menunggu frame 2.
            marker_center_target_ = vehicle_.position;
            control_->publishHeartbeat(true);
            control_->sendPositionSetpoint(
                toPx4North(marker_center_target_.north),
                toPx4East(marker_center_target_.east),
                toPx4DownForAltitudeTarget(wp.d), raw_bearing);
            RCLCPP_WARN(this->get_logger(),
                "[YOLO-INTERLOCK] BOX TERDETEKSI 1/1: APPROACH/MAJU "
                "DIBLOKIR pada sisa nominal %.2fm; hold N=%.3f E=%.3f "
                "offset=%.3fm.",
                along_track_remaining,
                marker_center_target_.north, marker_center_target_.east,
                yolo_offset_m);

            waypoint_phase_ = WaypointPhase::CENTER_MARKER;
            marker_target_latched_ = true;
            vision_engage_ticks_ = 0;
            wp_hold_counter_ = 0;
            marker_stable_frames_ = 0;
            marker_feedback_locked_ = false;
            yolo_center_anchor_ = vehicle_.position;
            marker_center_target_ = vehicle_.position;
            yolo_center_best_offset_m_ =
                std::numeric_limits<double>::infinity();
            yolo_center_diverging_ticks_ = 0;
            yolo_correction_sign_ = 1.0;
            yolo_direction_reversed_ = false;
            yolo_center_stage_ = YoloCenterStage::SAMPLE;
            yolo_step_target_ = vehicle_.position;
            yolo_step_started_s_ = this->now().seconds();
            // Deteksi 1/1 yang menghentikan approach juga menjadi SAMPLE
            // centering pertama; jangan membuangnya lalu menunggu box muncul
            // lagi. Timestamp dibuat sedikit sebelum sampel valid terakhir.
            yolo_accept_sample_after_s_ = last_marker_sample_s_ - 1e-6;
            yolo_step_stable_ticks_ = 0;
            yolo_step_count_ = 0;
            yolo_approach_brake_active_ = false;
            marker_latest_sample_available_ = true;
            RCLCPP_WARN(this->get_logger(),
                "[YOLO-INTERLOCK] BOX VALID 1/1: jarak nominal dianggap "
                "SELESAI di titik deteksi; masuk SAMPLE/EXECUTE.");
            return;
        }
    }

    // Gate assist benar-benar opt-in. Saat parameter false, tidak ada
    // subscriber Livox dan cabang ini menjadi no-op. Saat true, satu gate
    // dapat di-center dan ditembus pada setiap leg APPROACH.
    if (!is_pure_alt &&
        waypoint_phase_ == WaypointPhase::APPROACH &&
        runMissionGateAssist(raw_bearing, along_track_remaining, label))
    {
        return;
    }

    // ── Sudah sampai? ──────────────────────────────────────────────
    if (waypoint_phase_ == WaypointPhase::APPROACH &&
        waypoints_->isWaypointReached(dist, alt_reached))
    {
        if (!marker_source_enabled) {
            // Radius APPROACH (30 cm) hanya pemicu untuk beralih ke position
            // hold, bukan bukti bahwa drone sudah tepat di pusat waypoint.
            // Langsung menaikkan current_wp_ di sini membuat leg berikutnya
            // dimulai dengan offset samping sampai 30 cm; setelah yaw drone
            // lalu bergerak pada garis paralel dan meleset dari pusat WP.
            waypoint_phase_ = WaypointPhase::CENTER_MARKER;
            // FINAL HOLD mungkin memakai anchor sementara agar tidak mondar-
            // mandir sambil menunggu altitude. CENTER harus kembali menuju
            // pusat WP satu kali secara terkontrol.
            off_mode_final_hold_anchor_valid_ = false;
            wp_hold_counter_ = 0;
            RCLCPP_INFO(this->get_logger(),
                "[%s] %s lock disabled — final position hold ke pusat WP.",
                label.c_str(), marker_source_label);
        }
        else {
            waypoint_phase_ = WaypointPhase::SEARCH_MARKER;
            marker_search_started_at_ = this->now();
            marker_search_altitude_m_ = -wp.d;
            marker_search_direction_ = 1;
            marker_search_horizontal_m_ = 0.0;
            marker_search_horizontal_direction_ = 1;
            wp_hold_counter_ = 0;
            RCLCPP_INFO(this->get_logger(),
                "[%s] SEARCH %s: marker wajib sebelum lanjut.",
                label.c_str(), marker_source_label);
        }
    }

    if (waypoint_phase_ == WaypointPhase::SEARCH_MARKER) {
        // Marker diharapkan berada dekat pusat WP. Khusus WP2 gunakan pola
        // seperti akuisisi awal handoff: tahan altitude dan sweep lebih sempit.
        // Ini mencegah pencarian menambah drift/overshoot setelah belokan.
        // Khusus WP1 (YOLO/box) sweep dibatasi 0.15m: cukup untuk reacquire
        // posisi box lebih longgar dari marker ArUco, dan drone perlu
        // benar-benar bergerak mencari kalau box meleset dari pusat WP1
        // nominal (mis. akibat sedikit drift heading di leg sebelumnya),
        // bukan cuma diam menunggu deteksi yang tidak akan pernah datang.
        // Timing sengaja disamakan dengan profil default (~15s sampai
        // radius penuh: 5s tahan + waktu naik) supaya perilaku predictable,
        // cuma jangkauan spasialnya yang beda.
        const double search_min_altitude_m = -wp.d;
        const double search_s = (this->now() - marker_search_started_at_).seconds();
        const bool handoff_style_wp2 = current_wp_ == 1;
        const double hold_s = handoff_style_wp2 ? 1.5 : 5.0;
        const double active_s = std::max(0.0, search_s - hold_s);
        const double max_radius_m = handoff_style_wp2
            ? 0.03
            : (yolo_centering_wp ? 0.15 : 0.06);
        const double radius_rate_m_s = handoff_style_wp2
            ? 0.003
            : (yolo_centering_wp ? 0.020 : 0.006);
        const double radius = std::min(max_radius_m, active_s * radius_rate_m_s);
        const double angle = active_s * 0.30;
        marker_search_altitude_m_ = (handoff_style_wp2 || yolo_centering_wp)
            ? search_min_altitude_m
            : search_min_altitude_m + std::min(0.08, active_s * 0.008);
        const double search_n = wp.n + radius * std::cos(angle);
        const double search_e = wp.e + radius * std::sin(angle);

        control_->publishHeartbeat(true);
        control_->sendPositionSetpoint(
            toPx4North(search_n), toPx4East(search_e),
            toPx4DownForAltitudeTarget(-marker_search_altitude_m_),
            yaw_result.target_yaw);
        const double marker_age_s = this->now().seconds() - last_marker_sample_s_;
        const bool valid_sample = marker_latest_sample_available_ &&
            marker_age_s <= 0.35 &&
            std::hypot(marker_latest_offset_north_, marker_latest_offset_east_) <= 0.90;
        marker_stable_frames_ = valid_sample ? marker_stable_frames_ + 1 : 0;
        marker_latest_sample_available_ = false;
        // Box YOLO lapangan sering hanya muncul singkat beberapa frame.
        // Satu target_center sudah tidak ambigu karena node YOLO sendiri
        // hanya mem-publish center bila tepat satu box terdeteksi.
        const int acquisition_frames = yolo_centering_wp ? 1 : 5;
        if (marker_stable_frames_ < acquisition_frames) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[%s] mencari %s... %.1fs alt=%.2f->%.2fm radius=%.2fm valid=%d/%d",
                label.c_str(), marker_source_label, search_s, currentAltitudeAgl(),
                marker_search_altitude_m_,
                radius, marker_stable_frames_, acquisition_frames);
            if (search_s >= marker_search_timeout_s_) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "[%s] %s belum ditemukan setelah %.1fs - tetap HOLD.",
                    label.c_str(), marker_source_label, search_s);
            }
            return;
        }
        waypoint_phase_ = WaypointPhase::CENTER_MARKER;
        latched_marker_target_ = yolo_centering_wp
            ? ground_lock_->lockedTarget()
            : vision_lock_->lockedTarget();
        marker_target_latched_ = true;
        vision_engage_ticks_ = 0;
        wp_hold_counter_ = 0;
        marker_stable_frames_ = 0;
        // YOLO memakai siklus SAMPLE -> EXECUTE -> SETTLE. Begitu akuisisi
        // selesai, bekukan posisi aktual dan tunggu SATU frame baru; jangan
        // langsung mengejar target yang dihitung saat search masih bergerak.
        marker_center_target_ = vehicle_.position;
        yolo_center_anchor_ = vehicle_.position;
        yolo_center_best_offset_m_ = std::numeric_limits<double>::infinity();
        yolo_center_diverging_ticks_ = 0;
        yolo_correction_sign_ = 1.0;
        yolo_direction_reversed_ = false;
        yolo_center_stage_ = YoloCenterStage::SAMPLE;
        yolo_step_target_ = vehicle_.position;
        yolo_step_started_s_ = this->now().seconds();
        yolo_accept_sample_after_s_ = yolo_centering_wp
            ? last_marker_sample_s_ - 1e-6
            : yolo_step_started_s_;
        yolo_step_stable_ticks_ = 0;
        yolo_step_count_ = 0;
        // Untuk YOLO, sampel akuisisi 1/1 langsung dipakai sebagai SAMPLE
        // centering pertama. ArUco tetap menunggu sampel baru seperti semula.
        marker_latest_sample_available_ = yolo_centering_wp;
        RCLCPP_INFO(this->get_logger(),
            "[%s] %s VALID (%d sample) - HOLD lalu mulai SAMPLE/EXECUTE.",
            label.c_str(), marker_source_label, acquisition_frames);
    }

    if (waypoint_phase_ == WaypointPhase::CENTER_MARKER) {
        // Setelah masuk radius akhir, kunci posisi target di PX4 agar
        // pengendali posisi mengoreksi sisa error/drift selama dwell.
        // Kalau vision lock tidak aktif, cukup pakai target waypoint biasa
        // agar drone tetap stabil dan misi tetap jalan tanpa marker.
        double hold_north = wp.n;
        double hold_east  = wp.e;
        double hold_down  = wp.d;

        // Begitu gripper mulai membuka, centering dianggap final. Bekukan
        // posisi ini selama proses gripper dan total 5 detik sejak OPEN;
        // bbox/noise tidak boleh mengaktifkan centering ulang. Sesudah hold,
        // waypoint langsung berpindah ke blok yaw berikutnya.
        if (current_wp_ == gripper_drop_after_wp_ &&
            gripper_drop_state_ != GripperDropState::IDLE)
        {
            hold_north = marker_center_target_.north;
            hold_east = marker_center_target_.east;

            if (!gripper_drop_completed_) {
                runGripperDropIfNeeded(
                    hold_north, hold_east, hold_down,
                    yaw_result.target_yaw, label);
                return;
            }

            constexpr double POST_RELEASE_HOLD_S = 5.0;
            const double released_elapsed_s = payload_released_at_s_ >= 0.0
                ? this->now().seconds() - payload_released_at_s_
                : POST_RELEASE_HOLD_S;
            if (released_elapsed_s < POST_RELEASE_HOLD_S) {
                control_->publishHeartbeat(true);
                control_->sendPositionSetpoint(
                    toPx4North(hold_north), toPx4East(hold_east),
                    gripper_hold_down_valid_
                        ? gripper_hold_down_px4_
                        : toPx4DownForAltitudeTarget(hold_down),
                    yaw_result.target_yaw);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "[%s] POST-DROP HOLD: %.1f/%.1fs sebelum yaw.",
                    label.c_str(), released_elapsed_s, POST_RELEASE_HOLD_S);
                return;
            }

            // Simpan target center box, bukan posisi sesaat yang mungkin
            // sudah terkena noise estimator. Anchor ini bertahan melewati
            // reset state waypoint dan menjadi satu-satunya target posisi
            // untuk yaw sesudah dropping.
            post_drop_anchor_.north = hold_north;
            post_drop_anchor_.east = hold_east;
            // Untuk D gunakan posisi fisik EKF saat hold selesai. Target
            // altitude berbasis lidar dapat berbeda sedikit dari wp.d;
            // menyimpan wp.d di sini dapat memerintahkan climb/descend baru
            // saat yaw, padahal tinggi setelah drop harus dibekukan.
            post_drop_anchor_.down = vehicle_.position.down;
            post_drop_anchor_valid_ = true;
            RCLCPP_INFO(this->get_logger(),
                "[%s] DROP ANCHOR SAVED: N=%.3f E=%.3f D=%.3f; "
                "yaw berikutnya wajib kembali ke titik ini.",
                label.c_str(), post_drop_anchor_.north,
                post_drop_anchor_.east, post_drop_anchor_.down);
            commitCurrentWaypointAnchor();
            ++current_wp_;
            resetWaypointVisionState();
            RCLCPP_INFO(this->get_logger(),
                "[%s] POST-DROP HOLD selesai: langsung lanjut YAW.",
                label.c_str());
            return;
        }

        if (!marker_source_enabled) {
            constexpr double FINAL_CENTER_TOLERANCE_M = 0.08;
            constexpr double OFF_MODE_SETTLE_SPEED_M_S = 0.08;
            const double horizontal_speed = std::hypot(
                vehicle_.velocity.north, vehicle_.velocity.east);

            // Setelah centering sudah sah (dan gripper mungkin sudah mulai),
            // jangan evaluasi ulang noise posisi setiap tick. Tahan anchor
            // aktual yang sudah dikomit dan teruskan state machine gripper.
            if (off_mode_waypoint_committed_) {
                hold_north = off_mode_final_hold_anchor_.north;
                hold_east = off_mode_final_hold_anchor_.east;
                if (runGripperDropIfNeeded(
                    hold_north, hold_east, hold_down, raw_bearing, label))
                {
                    return;
                }
                commitCurrentWaypointAnchor();
                ++current_wp_;
                resetWaypointVisionState();
                return;
            }

            // Tanpa referensi visual, dekat waypoint saja belum berarti
            // badan sudah diam. Wajibkan velocity PX4 rendah agar momentum
            // lateral tidak terbawa masuk ke yaw berikutnya.
            const bool centered =
                dist <= FINAL_CENTER_TOLERANCE_M &&
                alt_reached &&
                horizontal_speed <= OFF_MODE_SETTLE_SPEED_M_S;
            if (centered) {
                ++wp_hold_counter_;
            } else {
                wp_hold_counter_ = 0;
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "[%s] FINAL CENTER OFF: dist=%.3fm alt_err=%.3fm "
                "speed_xy=%.3fm/s stable=%d/10",
                label.c_str(), dist, alt_error,
                horizontal_speed, wp_hold_counter_);
            if (wp_hold_counter_ >= 10) {
                off_mode_waypoint_committed_ = true;
                off_mode_final_hold_anchor_ = vehicle_.position;
                off_mode_final_hold_anchor_valid_ = true;
                hold_north = off_mode_final_hold_anchor_.north;
                hold_east = off_mode_final_hold_anchor_.east;
                RCLCPP_INFO(this->get_logger(),
                    "[%s] WAYPOINT CENTERED OFF: commit anchor N=%.3f E=%.3f.",
                    label.c_str(), hold_north, hold_east);
                if (runGripperDropIfNeeded(
                    hold_north, hold_east, hold_down, raw_bearing, label))
                {
                    return;
                }
                commitCurrentWaypointAnchor();
                ++current_wp_;
                resetWaypointVisionState();
            }
            control_->publishHeartbeat(true);
            control_->sendPositionSetpoint(
                toPx4North(hold_north),
                toPx4East(hold_east),
                toPx4DownForAltitudeTarget(hold_down),
                raw_bearing);
            return;
        }

        if (yolo_centering_wp) {
            // Centering box bersifat sample-and-hold. Target tidak pernah
            // berubah ketika drone sedang mengeksekusi satu langkah, jadi
            // bbox baru/noise tidak dapat menariknya maju lagi sebelum
            // langkah lama benar-benar selesai.
            const double now_s = this->now().seconds();
            const double marker_age_s = now_s - last_marker_sample_s_;
            const double horizontal_speed = std::hypot(
                vehicle_.velocity.north, vehicle_.velocity.east);
            constexpr double YOLO_LOCK_TOLERANCE_M = 0.05;
            constexpr double YOLO_LOCK_SPEED_M_S = 0.10;
            constexpr double YOLO_STEP_GAIN = 0.75;
            constexpr double YOLO_MAX_STEP_M = 0.18;
            constexpr double YOLO_STEP_POSITION_M = 0.035;
            constexpr int YOLO_STEP_REACHED_TICKS = 2;
            constexpr double YOLO_STEP_TIMEOUT_S = 2.0;
            constexpr double YOLO_SETTLE_SPEED_M_S = 0.08;
            constexpr int YOLO_SETTLE_TICKS = 2;

            hold_north = marker_center_target_.north;
            hold_east = marker_center_target_.east;

            if (yolo_center_stage_ == YoloCenterStage::SAMPLE) {
                const bool fresh_new_sample =
                    marker_latest_sample_available_ &&
                    marker_age_s <= 0.35 &&
                    last_marker_sample_s_ > yolo_accept_sample_after_s_;
	                if (fresh_new_sample) {
	                    const double offset_m = std::hypot(
	                        marker_latest_offset_north_, marker_latest_offset_east_);
	                    const bool ui_center_fresh =
	                        yolo_ui_centered_ &&
	                        (now_s - last_yolo_ui_center_s_) <= 0.35;
	                    marker_latest_sample_available_ = false;
	                    yolo_accept_sample_after_s_ = last_marker_sample_s_;
	
	                    if (offset_m <= YOLO_LOCK_TOLERANCE_M && ui_center_fresh) {
	                        marker_center_target_ = vehicle_.position;
	                        yolo_step_target_ = vehicle_.position;
	                        hold_north = vehicle_.position.north;
	                        hold_east = vehicle_.position.east;
	                        // Offset visual <=5 cm adalah keputusan final. Jangan
	                        // meminta frame baru setelah SETTLE karena deteksi box
	                        // lapangan bersifat singkat dan drone justru dapat
	                        // melewati pusat. Position hold langsung membekukan
	                        // titik ini; jeda OPEN gripper 1,5 s sekaligus meredam
	                        // sisa momentum sebelum payload benar-benar terlepas.
	                        marker_feedback_locked_ = true;
	                        RCLCPP_WARN(this->get_logger(),
	                            "[YOLO-STEP] CENTER LOCK FINAL: offset=%.3fm "
	                            "ui_err=(%+.0f,%+.0f)px speed=%.3fm/s; "
	                            "anchor dibekukan, DROP sekarang.",
	                            offset_m,
	                            yolo_ui_center_error_x_px_,
	                            yolo_ui_center_error_y_px_,
	                            horizontal_speed);
	                    } else if (offset_m > 1e-6) {
	                        if (offset_m <= YOLO_LOCK_TOLERANCE_M) {
	                            RCLCPP_WARN_THROTTLE(
	                                this->get_logger(), *this->get_clock(), 500,
	                                "[YOLO-STEP] meter sudah centered (%.3fm), "
	                                "tapi GeneralBox belum di kotak UI "
	                                "(fresh=%s ui_err=%+.0f,%+.0fpx tol=%.0fpx); "
	                                "tetap koreksi ke center UI.",
	                                offset_m,
	                                ui_center_fresh ? "yes" : "no",
	                                yolo_ui_center_error_x_px_,
	                                yolo_ui_center_error_y_px_,
	                                yolo_ui_center_tolerance_px_);
	                        }
	                        const double step_m = std::min(
	                            YOLO_MAX_STEP_M, YOLO_STEP_GAIN * offset_m);
                        const double scale = step_m / offset_m;
                        yolo_step_target_ = vehicle_.position;
                        yolo_step_target_.north +=
                            scale * marker_latest_offset_north_;
                        yolo_step_target_.east +=
                            scale * marker_latest_offset_east_;

                        constexpr double MAX_CENTER_RADIUS_M = 0.45;
                        const double dn = yolo_step_target_.north -
                            yolo_center_anchor_.north;
                        const double de = yolo_step_target_.east -
                            yolo_center_anchor_.east;
                        const double radius_m = std::hypot(dn, de);
                        if (radius_m > MAX_CENTER_RADIUS_M) {
                            const double radius_scale =
                                MAX_CENTER_RADIUS_M / radius_m;
                            yolo_step_target_.north = yolo_center_anchor_.north +
                                dn * radius_scale;
                            yolo_step_target_.east = yolo_center_anchor_.east +
                                de * radius_scale;
                        }

                        marker_center_target_ = yolo_step_target_;
                        hold_north = yolo_step_target_.north;
                        hold_east = yolo_step_target_.east;
                        yolo_center_stage_ = YoloCenterStage::EXECUTE;
                        yolo_step_started_s_ = now_s;
                        yolo_step_stable_ticks_ = 0;
                        ++yolo_step_count_;
                        RCLCPP_WARN(this->get_logger(),
                            "[YOLO-STEP] SAMPLE #%d: offset=%.3fm "
                            "(N=%+.3f E=%+.3f) -> EXECUTE step=%.3fm "
                            "target=(%.3f,%.3f).",
                            yolo_step_count_, offset_m,
                            marker_latest_offset_north_,
                            marker_latest_offset_east_, step_m,
                            hold_north, hold_east);
                    }
                } else {
                    marker_latest_sample_available_ = false;
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(), *this->get_clock(), 500,
                        "[YOLO-STEP] SAMPLE: HOLD (%.3f,%.3f), menunggu "
                        "frame box baru (age=%.2fs).",
                        hold_north, hold_east, marker_age_s);
                }
            } else if (yolo_center_stage_ == YoloCenterStage::EXECUTE) {
                marker_latest_sample_available_ = false;
                const double target_error_m = std::hypot(
                    yolo_step_target_.north - vehicle_.position.north,
                    yolo_step_target_.east - vehicle_.position.east);
                const bool step_reached =
                    target_error_m <= YOLO_STEP_POSITION_M &&
                    horizontal_speed <= YOLO_LOCK_SPEED_M_S;
                yolo_step_stable_ticks_ = step_reached
                    ? yolo_step_stable_ticks_ + 1 : 0;
                const double elapsed_s = now_s - yolo_step_started_s_;

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 300,
                    "[YOLO-STEP] EXECUTE #%d: target_err=%.3fm "
                    "speed=%.3fm/s stable=%d/%d t=%.1fs.",
                    yolo_step_count_, target_error_m, horizontal_speed,
                    yolo_step_stable_ticks_, YOLO_STEP_REACHED_TICKS,
                    elapsed_s);

                if (yolo_step_stable_ticks_ >= YOLO_STEP_REACHED_TICKS ||
                    elapsed_s >= YOLO_STEP_TIMEOUT_S)
                {
                    const bool timed_out = elapsed_s >= YOLO_STEP_TIMEOUT_S;
                    marker_center_target_ = vehicle_.position;
                    yolo_step_target_ = vehicle_.position;
                    hold_north = vehicle_.position.north;
                    hold_east = vehicle_.position.east;
                    yolo_center_stage_ = YoloCenterStage::SETTLE;
                    yolo_step_started_s_ = now_s;
                    yolo_accept_sample_after_s_ = now_s;
                    yolo_step_stable_ticks_ = 0;
                    RCLCPP_WARN(this->get_logger(),
                        "[YOLO-STEP] EXECUTE #%d %s -> SETTLE dan HOLD aktual.",
                        yolo_step_count_, timed_out ? "TIMEOUT" : "SELESAI");
                }
            } else {
                marker_latest_sample_available_ = false;
                const bool settled = horizontal_speed <= YOLO_SETTLE_SPEED_M_S;
                yolo_step_stable_ticks_ = settled
                    ? yolo_step_stable_ticks_ + 1 : 0;
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 300,
                    "[YOLO-STEP] SETTLE #%d: speed=%.3fm/s stable=%d/%d.",
                    yolo_step_count_, horizontal_speed,
                    yolo_step_stable_ticks_, YOLO_SETTLE_TICKS);
                if (yolo_step_stable_ticks_ >= YOLO_SETTLE_TICKS) {
                    yolo_center_stage_ = YoloCenterStage::SAMPLE;
                    yolo_accept_sample_after_s_ = now_s;
                    yolo_step_stable_ticks_ = 0;
                    RCLCPP_INFO(this->get_logger(),
                        "[YOLO-STEP] SETTLE #%d selesai -> SAMPLE baru.",
                        yolo_step_count_);
                }
            }

            control_->publishHeartbeat(true);
            control_->sendPositionSetpoint(
                toPx4North(hold_north),
                toPx4East(hold_east),
                toPx4DownForAltitudeTarget(hold_down),
                yaw_result.target_yaw);

            if (!marker_feedback_locked_) {
                return;
            }

            RCLCPP_INFO(this->get_logger(),
                "[%s] YOLO CENTERED - waypoint selesai.", label.c_str());
            RCLCPP_INFO(this->get_logger(),
                "REACHED POSITION: N=%.2f E=%.2f",
                vehicle_.position.north, vehicle_.position.east);
            if (runGripperDropIfNeeded(
                hold_north, hold_east, hold_down,
                yaw_result.target_yaw, label))
            {
                return;
            }
            commitCurrentWaypointAnchor();
            ++current_wp_;
            resetWaypointVisionState();
            return;
        }

	        const double marker_age_s = this->now().seconds() - last_marker_sample_s_;
	        if (marker_feedback_locked_) {
            // Target NED tetap beku, tetapi box WAJIB terus diverifikasi.
            // Lock lama tidak boleh membuka gripper bila angin menggeser
            // kendaraan dan box sudah bukan di pusat gambar.
	            hold_north = marker_center_target_.north;
	            hold_east = marker_center_target_.east;
	            if (yolo_centering_wp) {
	                const double ui_age_s =
	                    this->now().seconds() - last_yolo_ui_center_s_;
	                const bool fresh_box = marker_age_s <= 0.35;
	                const bool ui_center_fresh =
	                    yolo_ui_centered_ && ui_age_s <= 0.35;
	                const double live_box_offset = fresh_box
	                    ? std::hypot(marker_latest_offset_north_, marker_latest_offset_east_)
	                    : std::numeric_limits<double>::infinity();
	                if (!fresh_box || live_box_offset > 0.05 || !ui_center_fresh) {
	                    marker_feedback_locked_ = false;
	                    marker_stable_frames_ = 0;
	                    wp_hold_counter_ = 0;
	                    marker_center_target_ = vehicle_.position;
	                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
	                        "[YOLO] DROP INTERLOCK: box tidak centered/fresh "
	                        "(age=%.2fs offset=%.3fm ui=%s ui_err=%+.0f,%+.0fpx), "
	                        "centering dilanjutkan.",
	                        marker_age_s, live_box_offset,
	                        ui_center_fresh ? "CENTER" : "NOT_CENTER",
	                        yolo_ui_center_error_x_px_,
	                        yolo_ui_center_error_y_px_);
	                }
	            }
            marker_latest_sample_available_ = false;
        } else if (marker_latest_sample_available_ && marker_age_s <= 0.35) {
            const double measured_offset = std::hypot(
                marker_latest_offset_north_, marker_latest_offset_east_);

            // Untuk YOLO, GroundLock sudah menghitung titik tanah absolut
            // (posisi drone + offset bbox). Arahkan PX4 tegas ke titik itu,
            // bukan mengakumulasi langkah kecil yang berubah bersama noise.
            const double center_kp = 0.18;
            const double max_center_step_m = 0.03;
            const double correction_n = std::clamp(
                yolo_correction_sign_ * center_kp * marker_latest_offset_north_,
                -max_center_step_m, max_center_step_m);
            const double correction_e = std::clamp(
                yolo_correction_sign_ * center_kp * marker_latest_offset_east_,
                -max_center_step_m, max_center_step_m);
            if (yolo_centering_wp) {
                constexpr double DIVERGENCE_MARGIN_M = 0.12;
                constexpr int DIVERGENCE_TICKS = 6;
                if (measured_offset < yolo_center_best_offset_m_) {
                    yolo_center_best_offset_m_ = measured_offset;
                    yolo_center_diverging_ticks_ = 0;
                } else if (measured_offset >
                    yolo_center_best_offset_m_ + DIVERGENCE_MARGIN_M)
                {
                    ++yolo_center_diverging_ticks_;
                } else {
                    yolo_center_diverging_ticks_ = std::max(
                        0, yolo_center_diverging_ticks_ - 1);
                }

                if (yolo_center_diverging_ticks_ >= DIVERGENCE_TICKS) {
                    // Arah kamera sudah terbukti benar pada penerbangan
                    // sebelumnya. Lag bbox/overshoot tidak boleh membalik
                    // tanda koreksi dan membuat osilasi tengah-menjauh.
                    RCLCPP_ERROR(this->get_logger(),
                        "[YOLO] offset konsisten membesar (%.3fm, best %.3fm) "
                        "- HOLD posisi, tunggu pengukuran stabil tanpa spiral.",
                        measured_offset, yolo_center_best_offset_m_);
                    marker_center_target_ = vehicle_.position;
                    marker_stable_frames_ = 0;
                    marker_latest_sample_available_ = false;
                    yolo_center_best_offset_m_ =
                        std::numeric_limits<double>::infinity();
                    yolo_center_diverging_ticks_ = 0;
                    control_->publishHeartbeat(true);
                    control_->sendPositionSetpoint(
                        toPx4North(vehicle_.position.north),
                        toPx4East(vehicle_.position.east),
                        toPx4DownForAltitudeTarget(hold_down),
                        yaw_result.target_yaw);
                    return;
                }
            }
            // YOLO perlu terus mengoreksi sampai dekat pusat optik. Deadband
            // parameter ArUco 10 cm terlalu besar untuk menjatuhkan payload.
            const double center_deadband_m =
                yolo_centering_wp ? 0.02 : marker_center_tolerance_m_;
            if (measured_offset > center_deadband_m) {
                if (yolo_centering_wp) {
                    // Candidate tetap menunjuk pusat box di tanah. Filter
                    // adaptif: jauh dari pusat -> kejar tegas (alpha tinggi,
                    // setpoint dipush dekat ke titik ukur supaya PX4 segera
                    // membangun kecepatan, tidak merayap/"linglung" saat
                    // ada angin); dekat pusat -> redam (alpha rendah) supaya
                    // jitter bbox tidak memicu osilasi tepat sebelum drop.
                    const double target_alpha =
                        measured_offset > 0.15 ? 0.70 :
                        measured_offset > 0.05 ? 0.55 : 0.35;
                    const auto candidate = ground_lock_->lockedTarget();
                    marker_center_target_.north += target_alpha *
                        (candidate.north - marker_center_target_.north);
                    marker_center_target_.east += target_alpha *
                        (candidate.east - marker_center_target_.east);
                } else {
                    marker_center_target_.north =
                        vehicle_.position.north + correction_n;
                    marker_center_target_.east =
                        vehicle_.position.east + correction_e;
                }
            }
            constexpr double MAX_CENTER_RADIUS_M = 0.45;
            const double center_dn = marker_center_target_.north - wp.n;
            const double center_de = marker_center_target_.east - wp.e;
            const double center_dist = std::hypot(center_dn, center_de);
            if (center_dist > MAX_CENTER_RADIUS_M) {
                const double scale = MAX_CENTER_RADIUS_M / center_dist;
                marker_center_target_.north = wp.n + center_dn * scale;
                marker_center_target_.east = wp.e + center_de * scale;
            }
            hold_north = marker_center_target_.north;
            hold_east  = marker_center_target_.east;

            const double horizontal_offset = std::hypot(
                marker_latest_offset_north_, marker_latest_offset_east_);
            const double lock_tolerance_m = yolo_centering_wp
                ? 0.07
                : std::max(marker_center_tolerance_m_, 0.15);
            const int required_lock_frames = yolo_centering_wp ? 2 : 6;
            const bool within_threshold =
                horizontal_offset <= lock_tolerance_m;
            if (within_threshold) {
                ++marker_stable_frames_;
                RCLCPP_INFO(this->get_logger(),
                    "[%s] center terkonfirmasi=%d/%d | offset=%.3fm <= batas %.3fm (N=%.3f E=%.3f)",
                    marker_source_label, marker_stable_frames_, required_lock_frames,
                    horizontal_offset,
                    lock_tolerance_m,
                    marker_latest_offset_north_, marker_latest_offset_east_);
            } else {
                marker_stable_frames_ = std::max(0, marker_stable_frames_ - 1);
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "[%s] offset N=%.3f E=%.3f | target tegas N=%.3f E=%.3f",
                    marker_source_label, marker_latest_offset_north_, marker_latest_offset_east_,
                    marker_center_target_.north, marker_center_target_.east);
            }

            marker_latest_sample_available_ = false;
            if (marker_stable_frames_ >= required_lock_frames &&
                !marker_feedback_locked_)
            {
                marker_feedback_locked_ = true;
                marker_center_target_ = vehicle_.position;
                hold_north = marker_center_target_.north;
                hold_east = marker_center_target_.east;
                wp_hold_counter_ = 0;
                RCLCPP_INFO(this->get_logger(),
                    "[%s] CENTER LOCK: posisi dibekukan N=%.3f E=%.3f; DROP sekarang.",
                    marker_source_label,
                    marker_center_target_.north, marker_center_target_.east);
            }
        } else {
            hold_north = marker_center_target_.north;
            hold_east = marker_center_target_.east;

            // Untuk box YOLO yang sudah pernah diakuisisi, jangan aktifkan
            // spiral saat detector drop sesaat. Spiral justru dapat membawa
            // box yang tadi terlihat keluar FOV. Tahan posisi dan lanjutkan
            // closed-loop otomatis ketika frame segar kembali.
            constexpr double MARKER_LOST_RESEARCH_S = 3.0;
            if (!yolo_centering_wp && marker_age_s > MARKER_LOST_RESEARCH_S) {
                RCLCPP_WARN(this->get_logger(),
                    "[%s] %s hilang > %.1fs - kembali ke SEARCH, mencari ulang.",
                    label.c_str(), marker_source_label, MARKER_LOST_RESEARCH_S);
                waypoint_phase_ = WaypointPhase::SEARCH_MARKER;
                marker_search_started_at_ = this->now();
                marker_stable_frames_ = 0;
                marker_latest_sample_available_ = false;
                control_->publishHeartbeat(true);
                control_->sendPositionSetpoint(
                    toPx4North(hold_north), toPx4East(hold_east),
                    toPx4DownForAltitudeTarget(hold_down), yaw_result.target_yaw);
                return;
            }

            if (yolo_centering_wp && marker_age_s > MARKER_LOST_RESEARCH_S) {
                marker_center_target_ = vehicle_.position;
                hold_north = vehicle_.position.north;
                hold_east = vehicle_.position.east;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "[YOLO] box pernah terlihat tetapi sekarang hilang %.1fs; "
                    "HOLD posisi tanpa spiral, menunggu detector.", marker_age_s);
            }

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[%s] menunggu sampel segar (age=%.2fs), tahan target terakhir.",
                marker_source_label, marker_age_s);
        }

        control_->publishHeartbeat(true);
        control_->sendPositionSetpoint(
            toPx4North(hold_north),
            toPx4East(hold_east),
            toPx4DownForAltitudeTarget(hold_down),
            yaw_result.target_yaw);

        if (marker_feedback_locked_) {
            // Untuk YOLO, tiga frame di pusat langsung memicu OPEN. Posisi
            // kemudian dibekukan selama proses gripper + post-drop hold 5s,
            // jadi tidak perlu velocity/NED settle tambahan sebelum drop.
            RCLCPP_INFO(this->get_logger(),
                "[%s] %s CENTERED - waypoint selesai.", label.c_str(), marker_source_label);
            RCLCPP_INFO(this->get_logger(),
                "REACHED POSITION: N=%.2f E=%.2f", vehicle_.position.north, vehicle_.position.east);
            if (runGripperDropIfNeeded(
                hold_north, hold_east, hold_down, yaw_result.target_yaw, label))
            {
                return;
            }
            commitCurrentWaypointAnchor();
            ++current_wp_;
            resetWaypointVisionState();
        }
        return;
    }

    // ── Pure altitude WP — langsung climb, skip fase putar yaw ───────
    const bool final_approach_hold_ready =
        waypoint_phase_ == WaypointPhase::APPROACH &&
        yaw_locked_for_current_wp_ &&
        along_track_remaining <= 0.45;
    if (is_pure_alt && !final_approach_hold_ready) {
        control_->publishHeartbeat(false);
        control_->sendVelocitySetpoint(0.0, 0.0, vz, yaw_result.target_yaw);
        return;
    }

    // YOLO fail-closed sebelum bidang waypoint: kalau box belum sempat
    // menghasilkan target_center, hentikan leg pada sisa 90 cm dan tahan
    // SATU anchor tetap. Kamera mendapat waktu melihat box tanpa drone terus
    // melintas/overshoot. State tetap APPROACH sehingga satu frame YOLO baru
    // langsung ditangkap interlock di bagian atas pada tick berikutnya.
    // Log lapangan menunjukkan kendaraan masih bergerak sekitar 0,5 m/s
    // walau command sudah 0,3 m/s; rem lama 45 cm terlalu dekat. 90 cm juga
    // tepat sebelum jarak saat box pertama terlihat (sisa 0,82 m), sehingga
    // detector tetap punya kesempatan masuk tanpa drone melewati box.
    constexpr double YOLO_APPROACH_BRAKE_M = 0.90;
    if (yolo_centering_wp && !yolo_approach_interlock_consumed_ &&
        along_track_remaining <= YOLO_APPROACH_BRAKE_M)
    {
        if (!yolo_approach_brake_active_) {
            yolo_approach_brake_active_ = true;
            yolo_approach_brake_anchor_ = vehicle_.position;
            RCLCPP_ERROR(this->get_logger(),
                "[YOLO-SAFETY] sisa maju %.2fm tanpa box valid: FORWARD "
                "DIBLOKIR, anchor N=%.3f E=%.3f dibekukan sampai YOLO masuk.",
                along_track_remaining,
                yolo_approach_brake_anchor_.north,
                yolo_approach_brake_anchor_.east);
        }
        control_->publishHeartbeat(true);
        control_->sendPositionSetpoint(
            toPx4North(yolo_approach_brake_anchor_.north),
            toPx4East(yolo_approach_brake_anchor_.east),
            toPx4DownForAltitudeTarget(wp.d), raw_bearing);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[YOLO-SAFETY] HOLD sebelum box: pos_err=%.3fm speed=%.3fm/s; "
            "menunggu satu /general_box/target_center valid.",
            std::hypot(
                yolo_approach_brake_anchor_.north - vehicle_.position.north,
                yolo_approach_brake_anchor_.east - vehicle_.position.east),
            std::hypot(vehicle_.velocity.north, vehicle_.velocity.east));
        return;
    }

    // Saat tinggal 20cm pada arah maju, pindah dari velocity-forward ke
    // position-hold waypoint. Syarat selesai waypoint juga memerlukan
    // altitude; tanpa hold ini drone dapat melewati target horizontal lalu
    // terus maju selamanya sambil menunggu altitude masuk toleransi.
    constexpr double FINAL_POSITION_HOLD_M = 0.45;
    if (waypoint_phase_ == WaypointPhase::APPROACH &&
        yaw_locked_for_current_wp_ &&
        along_track_remaining <= FINAL_POSITION_HOLD_M)
    {
        double final_hold_north = wp.n;
        double final_hold_east = wp.e;
        if (!marker_source_enabled) {
            constexpr double OFF_MODE_BRAKE_RADIUS_M = 0.30;
            if (!off_mode_final_hold_anchor_valid_ &&
                dist <= OFF_MODE_BRAKE_RADIUS_M)
            {
                off_mode_final_hold_anchor_ = vehicle_.position;
                off_mode_final_hold_anchor_valid_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "[%s] FINAL HOLD OFF: anchor tetap N=%.3f E=%.3f "
                    "sambil menunggu altitude.",
                    label.c_str(), off_mode_final_hold_anchor_.north,
                    off_mode_final_hold_anchor_.east);
            }
            if (off_mode_final_hold_anchor_valid_) {
                final_hold_north = off_mode_final_hold_anchor_.north;
                final_hold_east = off_mode_final_hold_anchor_.east;
            }
        }
        control_->publishHeartbeat(true);
        control_->sendPositionSetpoint(
            toPx4North(final_hold_north), toPx4East(final_hold_east),
            toPx4DownForAltitudeTarget(wp.d), raw_bearing);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[%s] FINAL HOLD: sisa maju=%.2fm dist=%.2fm; tahan WP sambil "
            "menunggu altitude (error=%.2fm).",
            label.c_str(), along_track_remaining, dist, alt_error);
        return;
    }

    // ── Kunci yaw dulu, baru boleh maju ───────────────────────────────
    // Tahan N/E pada posisi aktual dan berikan heading akhir langsung ke
    // PX4. Jangan tambahkan gerakan lateral atau target yaw hasil smoothing
    // di fase ini: keduanya pernah membuat drone bergeser/mengorbit sebelum
    // rotasi fisik selesai. Gerak horizontal baru dibuka setelah yaw AKTUAL
    // stabil dekat bearing leg selama 0.5 detik.
    if (!yaw_locked_for_current_wp_) {
        if (post_yaw_correction_active_) {
            const double correction_elapsed_s =
                this->now().seconds() - post_yaw_correction_started_s_;
            const double remaining = std::hypot(
                post_yaw_correction_target_.north - vehicle_.position.north,
                post_yaw_correction_target_.east - vehicle_.position.east);
            const double shift_error_n =
                post_yaw_correction_target_.north - vehicle_.position.north;
            const double shift_error_e =
                post_yaw_correction_target_.east - vehicle_.position.east;
            const double horizontal_speed = std::hypot(
                vehicle_.velocity.north, vehicle_.velocity.east);
            const double vertical_error = std::abs(
                post_yaw_correction_target_.down - vehicle_.position.down);
            const double vertical_speed = std::abs(vehicle_.velocity.down);
            const double yaw_error_abs = std::abs(raw_yaw_error);

            // Profil dua tahap untuk shift generik 20 cm:
            //   1) sisa >3 cm: velocity drive supaya bagian awal cepat dan
            //      tidak mudah kalah oleh angin;
            //   2) sisa <=3 cm: position hold supaya PX4 mengerem dan
            //      menstabilkan titik akhir tanpa overshoot.
            // Heartbeat selalu cocok dengan jenis setpoint pada cabangnya.
            constexpr double SHIFT_BRAKE_RADIUS_M = 0.030;
            constexpr double SHIFT_MIN_SPEED_M_S = 0.20;
            constexpr double SHIFT_MAX_SPEED_M_S = 0.30;
            constexpr double SHIFT_SPEED_KP = 2.5;
            // Return-to-drop selalu memakai position control tiga sumbu.
            // Jarak koreksinya bebas (sesuai drift aktual), tetapi targetnya
            // tetap persis anchor center box. Velocity drive lama hanya
            // dipertahankan untuk shift generik di yaw selain setelah drop.
            const bool shift_driving =
                !post_yaw_return_to_drop_anchor_ &&
                remaining > SHIFT_BRAKE_RADIUS_M;
            double shift_command_speed = 0.0;
            if (shift_driving && remaining > 1e-6) {
                shift_command_speed = std::clamp(
                    remaining * SHIFT_SPEED_KP,
                    SHIFT_MIN_SPEED_M_S, SHIFT_MAX_SPEED_M_S);
                const double inv_remaining = 1.0 / remaining;
                control_->publishHeartbeat(false);
                control_->sendVelocitySetpoint(
                    shift_command_speed * shift_error_n * inv_remaining,
                    shift_command_speed * shift_error_e * inv_remaining,
                    0.0, raw_bearing);
            } else {
                control_->publishHeartbeat(true);
                control_->sendPositionSetpoint(
                    toPx4North(post_yaw_correction_target_.north),
                    toPx4East(post_yaw_correction_target_.east),
                    origin_down_ + yaw_hold_position_.down, raw_bearing);
            }

            // Koreksi baru dianggap selesai setelah position controller
            // benar-benar masuk radius 2 cm dan laju horizontal sudah rendah.
            // Untuk RETURN-TO-DROP, altitude dan yaw juga wajib stabil.
            // DRIVE berhenti saat sisa 3 cm, tetapi shift baru dianggap
            // selesai setelah position controller masuk 2 cm terakhir.
            constexpr double SHIFT_COMPLETE_RADIUS_M = 0.020;
            constexpr double SHIFT_SETTLE_SPEED_M_S = 0.10;
            constexpr double DROP_ANCHOR_ALT_TOLERANCE_M = 0.05;
            constexpr double DROP_ANCHOR_VERTICAL_SPEED_M_S = 0.10;
            constexpr double DROP_ANCHOR_YAW_TOLERANCE_RAD = 2.0 * PI / 180.0;
            const bool shift_stable =
                remaining <= SHIFT_COMPLETE_RADIUS_M &&
                horizontal_speed <= SHIFT_SETTLE_SPEED_M_S &&
                (!post_yaw_return_to_drop_anchor_ ||
                    (vertical_error <= DROP_ANCHOR_ALT_TOLERANCE_M &&
                     vertical_speed <= DROP_ANCHOR_VERTICAL_SPEED_M_S &&
                     yaw_error_abs <= DROP_ANCHOR_YAW_TOLERANCE_RAD));
            const int required_shift_ticks =
                post_yaw_return_to_drop_anchor_ ? 5 : 3;
            constexpr double POST_YAW_SHIFT_TIMEOUT_S = 5.0;
            if (shift_stable) {
                ++post_yaw_correction_stable_ticks_;
            } else {
                post_yaw_correction_stable_ticks_ = 0;
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "[%s] POST-YAW %s %s: t=%.1fs error_xy=%.3fm "
                "error_alt=%.3fm error_yaw=%.1fdeg speed_xy=%.3fm/s "
                "speed_z=%.3fm/s stabil=%d/%d",
                label.c_str(),
                post_yaw_return_to_drop_anchor_ ? "RETURN-TO-DROP" : "SHIFT",
                shift_driving ? "DRIVE" : "HOLD",
                correction_elapsed_s, remaining, vertical_error,
                radToDeg(yaw_error_abs), horizontal_speed, vertical_speed,
                post_yaw_correction_stable_ticks_, required_shift_ticks);

            // Return-to-drop bersifat fail-closed: timeout tidak pernah
            // membuka gerak maju. Bila belum presisi, drone terus HOLD di
            // anchor daripada mengandalkan keberuntungan dan menabrak gate.
            const bool generic_shift_timeout =
                !post_yaw_return_to_drop_anchor_ &&
                correction_elapsed_s >= POST_YAW_SHIFT_TIMEOUT_S;
            if (post_yaw_return_to_drop_anchor_ &&
                correction_elapsed_s >= POST_YAW_SHIFT_TIMEOUT_S &&
                post_yaw_correction_stable_ticks_ < required_shift_ticks)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "[%s] POST-YAW RETURN belum stabil setelah %.1fs; "
                    "MAJU DIBLOKIR, tetap HOLD anchor drop.",
                    label.c_str(), correction_elapsed_s);
            }

            if (post_yaw_correction_stable_ticks_ >= required_shift_ticks ||
                generic_shift_timeout) {
                const size_t translate_from = current_wp_ > 0 ? current_wp_ - 1 : 0;
                // Anchor leg baru tepat pada posisi aktual setelah shift.
                // Menggunakan selisih terhadap yaw_hold_position_ salah bila
                // centering ArUco telah menggeser drone dari waypoint nominal:
                // waypoint lama tidak jatuh di posisi aktual dan TRACK langsung
                // memberi koreksi lateral sehingga gerak maju terlihat miring.
                const double rebase_n =
                    vehicle_.position.north - waypoints_->at(translate_from).n;
                const double rebase_e =
                    vehicle_.position.east - waypoints_->at(translate_from).e;
                waypoints_->translateWaypointsFrom(
                    translate_from, rebase_n, rebase_e);
                const bool completed_drop_return =
                    post_yaw_return_to_drop_anchor_;
                post_yaw_correction_active_ = false;
                post_yaw_return_to_drop_anchor_ = false;
                if (completed_drop_return) {
                    post_drop_anchor_valid_ = false;
                }
                altitude_hold_after_yaw_ = true;
                yaw_locked_for_current_wp_ = true;
                yaw_hold_position_valid_ = false;
                RCLCPP_INFO(this->get_logger(),
                    "[%s] POST-YAW %s COMPLETE%s: dN=%.3fm dE=%.3fm; "
                    "jalur direbase dan mulai maju lurus.",
                    label.c_str(),
                    completed_drop_return ? "RETURN-TO-DROP" : "SHIFT",
                    generic_shift_timeout ? " (timeout aman)" : "",
                    rebase_n, rebase_e);
            }
            return;
        }

        if (!yaw_hold_position_valid_) {
            const bool yaw_immediately_after_drop =
                post_drop_anchor_valid_ &&
                current_wp_ == gripper_drop_after_wp_ + 1;
            yaw_hold_position_ = yaw_immediately_after_drop
                ? post_drop_anchor_
                : vehicle_.position;
            yaw_hold_position_valid_ = true;
            altitude_hold_after_yaw_ = false;
            yaw_aligned_ticks_ = 0;
            yaw_command_unwrapped_ = vehicle_.yaw;
            // Kunci arah putar sekali di awal berdasarkan jalur terpendek.
            // Target dan command disimpan unwrapped supaya crossing -pi/+pi
            // tidak pernah membalik arah di tengah rotasi.
            yaw_target_unwrapped_ = vehicle_.yaw + raw_yaw_error;
            yaw_profile_direction_ = raw_yaw_error < 0.0 ? -1.0 : 1.0;
            yaw_profile_rate_rad_s_ = 0.0;
            yaw_hold_start_yaw_ = vehicle_.yaw;
            RCLCPP_INFO(this->get_logger(),
                "[%s] YAW START%s: tahan N=%.3f E=%.3f D=%.3f, "
                "target=%.1fdeg; altitude dibekukan.",
                label.c_str(), yaw_immediately_after_drop ? " DROP-ANCHOR" : "",
                yaw_hold_position_.north,
                yaw_hold_position_.east, yaw_hold_position_.down,
                radToDeg(raw_bearing));
        }

        constexpr double YAW_DT_S = 0.10;
        constexpr double YAW_MAX_RATE_RAD_S = 0.35;  // ~20 deg/s
        constexpr double YAW_ACCEL_RAD_S2 = 0.70;    // ~40 deg/s^2
        const double remaining_command = std::max(
            0.0, yaw_profile_direction_ *
            (yaw_target_unwrapped_ - yaw_command_unwrapped_));
        const double stopping_rate = std::sqrt(
            2.0 * YAW_ACCEL_RAD_S2 * remaining_command);
        const double desired_rate = std::min(YAW_MAX_RATE_RAD_S, stopping_rate);
        const double max_rate_step = YAW_ACCEL_RAD_S2 * YAW_DT_S;
        yaw_profile_rate_rad_s_ += std::clamp(
            desired_rate - yaw_profile_rate_rad_s_,
            -max_rate_step, max_rate_step);

        const double command_step = yaw_profile_rate_rad_s_ * YAW_DT_S;
        bool command_reached = remaining_command <= command_step + 1e-6;
        if (command_reached) {
            yaw_command_unwrapped_ = yaw_target_unwrapped_;
            yaw_profile_rate_rad_s_ = 0.0;
        } else {
            yaw_command_unwrapped_ += yaw_profile_direction_ * command_step;
        }
        const double yaw_command = std::atan2(
            std::sin(yaw_command_unwrapped_),
            std::cos(yaw_command_unwrapped_));

        control_->publishHeartbeat(true);
        control_->sendPositionSetpoint(
            toPx4North(yaw_hold_position_.north),
            toPx4East(yaw_hold_position_.east),
            origin_down_ + yaw_hold_position_.down,
            yaw_command);

        if (command_reached && waypoints_->isYawAligned(raw_yaw_error)) {
            ++yaw_aligned_ticks_;
        } else {
            yaw_aligned_ticks_ = 0;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[%s] YAW HOLD: cmd=%.1fdeg actual=%.1fdeg target=%.1fdeg "
            "error=%.1fdeg rate=%.1fdeg/s stable=%d/5",
            label.c_str(), radToDeg(yaw_command), radToDeg(vehicle_.yaw),
            radToDeg(raw_bearing), radToDeg(raw_yaw_error),
            radToDeg(yaw_profile_rate_rad_s_), yaw_aligned_ticks_);

        if (yaw_aligned_ticks_ >= 5) {
            if (!marker_source_enabled) {
                constexpr double OFF_MODE_SETTLE_SPEED_M_S = 0.08;
                constexpr int OFF_MODE_POST_YAW_SETTLE_TICKS = 10;
                const double horizontal_speed = std::hypot(
                    vehicle_.velocity.north, vehicle_.velocity.east);
                if (horizontal_speed <= OFF_MODE_SETTLE_SPEED_M_S) {
                    ++yaw_hold_settle_ticks_;
                } else {
                    yaw_hold_settle_ticks_ = 0;
                }
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "[%s] POST-YAW SETTLE OFF: speed_xy=%.3fm/s stable=%d/%d",
                    label.c_str(), horizontal_speed, yaw_hold_settle_ticks_,
                    OFF_MODE_POST_YAW_SETTLE_TICKS);
                if (yaw_hold_settle_ticks_ < OFF_MODE_POST_YAW_SETTLE_TICKS) {
                    return;
                }
            }
            const double completed_turn = std::atan2(
                std::sin(raw_bearing - yaw_hold_start_yaw_),
                std::cos(raw_bearing - yaw_hold_start_yaw_));
            constexpr double MIN_SHIFT_TURN_RAD = 0.35;
            constexpr double POST_YAW_SHIFT_M = 0.20;
            if (std::abs(completed_turn) >= MIN_SHIFT_TURN_RAD) {
                const bool yaw_immediately_after_drop =
                    post_drop_anchor_valid_ &&
                    current_wp_ == gripper_drop_after_wp_ + 1;
                if (yaw_immediately_after_drop) {
                    // Tidak ada shift 20 cm setelah drop. Kembali tepat ke
                    // center box berapa pun jarak/arah drift selama yaw.
                    post_yaw_correction_target_ = post_drop_anchor_;
                    post_yaw_return_to_drop_anchor_ = true;
                } else {
                    // Perilaku yaw lain tidak diubah: right-vector heading
                    // akhir, turn kiri -> shift kanan dan sebaliknya.
                    const double shift_sign = completed_turn < 0.0 ? 1.0 : -1.0;
                    post_yaw_correction_target_ = vehicle_.position;
                    post_yaw_correction_target_.north +=
                        shift_sign * POST_YAW_SHIFT_M * -std::sin(raw_bearing);
                    post_yaw_correction_target_.east +=
                        shift_sign * POST_YAW_SHIFT_M * std::cos(raw_bearing);
                    post_yaw_return_to_drop_anchor_ = false;
                }
                post_yaw_correction_stable_ticks_ = 0;
                post_yaw_correction_started_s_ = this->now().seconds();
                post_yaw_correction_active_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "[%s] YAW COMPLETE %.1fdeg: %s.",
                    label.c_str(), radToDeg(completed_turn),
                    yaw_immediately_after_drop
                        ? "kembali presisi ke DROP ANCHOR (tanpa shift 20cm)"
                        : "mulai shift lateral 20cm");
            } else {
                altitude_hold_after_yaw_ = true;
                yaw_locked_for_current_wp_ = true;
                yaw_hold_position_valid_ = false;
                RCLCPP_INFO(this->get_logger(),
                    "[%s] YAW COMPLETE: perubahan kecil, langsung maju lurus.",
                    label.c_str());
            }
        }
        return;
    }

    // ── Maju ke target, kecepatan diskala smooth sesuai heading ───────
    // Yaw sudah terkunci di atas, jadi align factor di sini murni
    // fine-tune kecil (bukan gate on/off) untuk redam noise setpoint yaw
    // kecil selama maju, bukan penyebab utama kecepatan.
    // Path follower: komponen utama selalu maju sepanjang bearing leg, dengan
    // koreksi cross-track kecil dan terbatas untuk menghapus drift samping.
    // Kode lama membuang seluruh error lateral sehingga offset saat keluar
    // dari WP sebelumnya dipertahankan sampai akhir leg.
    const double along_n = std::cos(raw_bearing);
    const double along_e = std::sin(raw_bearing);
    const double lateral_n = -along_e;
    const double lateral_e = along_n;
    const double cross_track_error =
        err_n * lateral_n + err_e * lateral_e;

    // Jangan langsung memberi komponen samping untuk error kecil. Estimator
    // horizontal biasa bergerak beberapa sentimeter walaupun badan drone
    // sudah menghadap tepat ke bearing leg; koreksi lama (KP 0.60, cap
    // 0.25m/s) membuat perintah maju mempunyai komponen lateral yang terlihat
    // seperti terbang miring, terutama ketika forward_speed mulai turun.
    //
    // Di dalam koridor 10 cm drone diperintah murni maju. Di luar koridor,
    // koreksi hanya menghapus kelebihan error dan selalu dibatasi 10% dari
    // kecepatan maju, sehingga arah velocity tidak dapat menyimpang lebih
    // dari sekitar 5.7 derajat. Ini berlaku identik untuk leg pertama dan leg
    // setelah yaw; leg setelah yaw sudah direbase dari posisi aktual shift.
    // Log penerbangan handoff terbaru menunjukkan WP1 konsisten hanyut ke
    // kiri: cross-track -0.09m tumbuh sampai -0.37m karena koreksi lama hanya
    // 5% kecepatan maju dan hampir nol ketika mengerem. Gunakan koridor 5cm,
    // koreksi posisi moderat, serta damping velocity kecil. Floor 8cm/s
    // menjaga koreksi tetap bekerja saat forward_speed sudah rendah.
    constexpr double CROSS_TRACK_DEADBAND_M = 0.02;
    constexpr double CROSS_TRACK_KP = 0.55;
    constexpr double FIRST_LEG_LATERAL_KD = 0.90;
    constexpr double MAX_CROSS_TRACK_SPEED_M_S = 0.18;
    constexpr double MAX_CROSS_TRACK_RATIO = 0.15;
    constexpr double MIN_CROSS_TRACK_SPEED_M_S = 0.08;
    // Leg setelah yaw/shift perlu menolak kecepatan menyamping sejak awal.
    // Rebase sudah membuat cross-track awal ~0, tetapi log penerbangan
    // menunjukkan drift kiri tumbuh sampai >0.4m karena controller lama baru
    // bereaksi terhadap error posisi dan dibatasi 0.12m/s. Tambahkan damping
    // velocity hanya pada leg pasca-turn; leg pertama tetap persis seperti
    // tuning di atas.
    const bool post_turn_leg = current_wp_ > 0;
    constexpr double POST_TURN_DEADBAND_M = 0.03;
    constexpr double POST_TURN_CROSS_KP = 0.55;
    constexpr double POST_TURN_LATERAL_KD = 0.80;
    constexpr double POST_TURN_MAX_CROSS_SPEED_M_S = 0.20;
    constexpr double POST_TURN_MAX_CROSS_RATIO = 0.15;
    // Kecepatan maju memakai satu-satunya formula resmi di
    // WaypointHandler: cruise piecewise di luar 2 m dan formula pengereman
    // baseline yang tidak berubah di dalam 2 m. Bearing tetap bearing leg;
    // koreksi samping kecil hanya menghapus drift, bukan mengubah arah misi.
    // Hard anti-overshoot: computeApproachSpeed(0) punya floor 0,3 m/s.
    // Tanpa guard ini, kendaraan yang sudah melewati bidang waypoint masih
    // diperintah maju. Nilai negatif/0 harus berarti nol velocity-forward.
    const double nominal_forward_speed = along_track_remaining > 0.0
        ? waypoints_->computeApproachSpeed(along_track_remaining)
        : 0.0;
    const double measured_lateral_speed =
        vehicle_.velocity.north * lateral_n +
        vehicle_.velocity.east * lateral_e;

    // ── Safety drift lintasan ───────────────────────────────────────
    // Koreksi normal sudah terbukti menjaga error sekitar 2--7 cm. Safety
    // tidak boleh mengambil alih pada error kecil itu karena mencampur
    // velocity maju+lateral justru membuat lintasan terlihat diagonal.
    // Baru ketika drift benar-benar >10 cm: bekukan progres maju, arahkan
    // position controller ke proyeksi terdekat pada garis leg, stabil 0,5 s,
    // lalu serahkan kembali ke follower normal. Target recovery dibekukan
    // sekali saat ENTER agar tidak bergerak mengikuti estimator tiap tick.
    constexpr double DRIFT_RECOVERY_ENTER_M = 0.10;
    constexpr double DRIFT_RECOVERY_EXIT_M = 0.03;
    constexpr double DRIFT_RECOVERY_POSITION_M = 0.03;
    constexpr double DRIFT_RECOVERY_SPEED_M_S = 0.08;
    constexpr int DRIFT_RECOVERY_STABLE_TICKS = 5;

    if (!forward_drift_recovery_active_ &&
        std::abs(cross_track_error) > DRIFT_RECOVERY_ENTER_M)
    {
        forward_drift_recovery_active_ = true;
        forward_drift_recovery_stable_ticks_ = 0;
        forward_drift_recovery_target_ = vehicle_.position;
        forward_drift_recovery_target_.north +=
            cross_track_error * lateral_n;
        forward_drift_recovery_target_.east +=
            cross_track_error * lateral_e;
        RCLCPP_ERROR(this->get_logger(),
            "[%s][DRIFT-SAFETY] ENTER: cross=%+.3fm > 0.10m; FORWARD "
            "STOP, recovery position N=%.3f E=%.3f pada garis leg.",
            label.c_str(), cross_track_error,
            forward_drift_recovery_target_.north,
            forward_drift_recovery_target_.east);
    }

    if (forward_drift_recovery_active_) {
        const double recovery_position_error = std::hypot(
            forward_drift_recovery_target_.north - vehicle_.position.north,
            forward_drift_recovery_target_.east - vehicle_.position.east);
        const double horizontal_speed = std::hypot(
            vehicle_.velocity.north, vehicle_.velocity.east);
        const bool recovered =
            std::abs(cross_track_error) <= DRIFT_RECOVERY_EXIT_M &&
            recovery_position_error <= DRIFT_RECOVERY_POSITION_M &&
            horizontal_speed <= DRIFT_RECOVERY_SPEED_M_S;
        forward_drift_recovery_stable_ticks_ = recovered
            ? forward_drift_recovery_stable_ticks_ + 1 : 0;
        if (forward_drift_recovery_stable_ticks_ >=
            DRIFT_RECOVERY_STABLE_TICKS)
        {
            forward_drift_recovery_active_ = false;
            forward_drift_recovery_stable_ticks_ = 0;
            RCLCPP_WARN(this->get_logger(),
                "[%s][DRIFT-SAFETY] CLEAR: cross=%+.3fm, speed=%.3fm/s; "
                "kecepatan maju normal dilanjutkan.",
                label.c_str(), cross_track_error, horizontal_speed);
        } else {
            control_->publishHeartbeat(true);
            control_->sendPositionSetpoint(
                toPx4North(forward_drift_recovery_target_.north),
                toPx4East(forward_drift_recovery_target_.east),
                toPx4DownForAltitudeTarget(wp.d), raw_bearing);
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 300,
                "[%s][DRIFT-SAFETY] HOLD/RECENTER: forward=0 "
                "cross=%+.3fm pos_err=%.3fm speed=%.3fm/s stable=%d/%d.",
                label.c_str(), cross_track_error, recovery_position_error,
                horizontal_speed, forward_drift_recovery_stable_ticks_,
                DRIFT_RECOVERY_STABLE_TICKS);
            return;
        }
    }

    const double forward_speed = nominal_forward_speed;
    const double active_deadband =
        post_turn_leg ? POST_TURN_DEADBAND_M : CROSS_TRACK_DEADBAND_M;
    const double active_cross_kp =
        post_turn_leg ? POST_TURN_CROSS_KP : CROSS_TRACK_KP;
    const double active_max_cross_speed = post_turn_leg
        ? POST_TURN_MAX_CROSS_SPEED_M_S
        : MAX_CROSS_TRACK_SPEED_M_S;
    const double active_max_cross_ratio = post_turn_leg
        ? POST_TURN_MAX_CROSS_RATIO
        : MAX_CROSS_TRACK_RATIO;
    const double cross_track_excess = std::copysign(
        std::max(0.0, std::abs(cross_track_error) - active_deadband),
        cross_track_error);
    const double lateral_damping = post_turn_leg
        ? -POST_TURN_LATERAL_KD * measured_lateral_speed
        : -FIRST_LEG_LATERAL_KD * measured_lateral_speed;
    const double lateral_speed_limit = std::min(
        active_max_cross_speed,
        std::max(
            post_turn_leg ? 0.0 : MIN_CROSS_TRACK_SPEED_M_S,
            forward_speed * active_max_cross_ratio));
    const double lateral_speed = std::clamp(
        cross_track_excess * active_cross_kp + lateral_damping,
        -lateral_speed_limit, lateral_speed_limit);

    double vx = forward_speed * along_n + lateral_speed * lateral_n;
    double vy = forward_speed * along_e + lateral_speed * lateral_e;
    const double align = waypoints_->yawAlignmentFactor(raw_yaw_error);
    vx *= align;
    vy *= align;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "[%s] TRACK%s%s: along_rem=%.2fm cross_err=%.2fm "
        "v_forward=%.2f v_cross=%.2f measured_cross_v=%.2f vz=%.2f%s",
        label.c_str(), post_turn_leg ? "-POST-TURN" : "", "",
        along_track_remaining, cross_track_error,
        forward_speed * align, lateral_speed * align, measured_lateral_speed,
        altitude_hold_after_yaw_ ? 0.0 : vz,
        altitude_hold_after_yaw_ ? " (ALT HOLD)" : "");
    control_->publishHeartbeat(false);
    const double commanded_vz = altitude_hold_after_yaw_ ? 0.0 : vz;
    control_->sendVelocitySetpoint(vx, vy, commanded_vz, raw_bearing);
}

bool MissionManager::runMissionGateAssist(
    double leg_bearing, double along_track_remaining,
    const std::string & waypoint_label)
{
    if (!gate_centering_enable_ ||
        mission_gate_completed_for_wp_ ||
        !yaw_locked_for_current_wp_)
    {
        return false;
    }

    const double now_s = this->now().seconds();
    const bool sample_fresh =
        last_gate_sample_s_ >= 0.0 &&
        now_s - last_gate_sample_s_ <= 0.35;
    const bool gate_observed =
        sample_fresh && gate_centering_latest_.valid;
    const bool gate_confirmed =
        sample_fresh &&
        gate_centering_latest_.valid &&
        gate_centering_latest_.width_valid &&
        !gate_centering_latest_.one_side_only;

    if (!mission_gate_active_) {
        if (!mission_gate_search_anchor_valid_) {
            mission_gate_search_anchor_ = vehicle_.position;
            mission_gate_search_anchor_valid_ = true;
            RCLCPP_INFO(this->get_logger(),
                "[MISSION][%s][GATE] SEARCH START: yaw pilot/leg %.1fdeg, "
                "maks %.2fm @ %.2fm/s.",
                waypoint_label.c_str(), radToDeg(leg_bearing),
                mission_gate_search_distance_m_, mission_gate_search_speed_m_s_);
        }
        const double search_dn =
            vehicle_.position.north - mission_gate_search_anchor_.north;
        const double search_de =
            vehicle_.position.east - mission_gate_search_anchor_.east;
        const double search_traveled = std::max(0.0,
            search_dn * std::cos(leg_bearing) +
            search_de * std::sin(leg_bearing));

        // FAIL-CLOSED: satu sisi/geometry belum valid tetap berarti ada
        // penghalang di koridor. Begitu Livox melihat kandidat gate, jalur
        // APPROACH normal tidak boleh maju lagi. Dua tiang + center + heading
        // baru menjadi syarat untuk membuka ADVANCE.
        if (!gate_observed &&
            search_traveled < mission_gate_search_distance_m_)
        {
            // Pencarian gate hanya maju pelan pada yaw pilot. Jangan gunakan
            // TRACK normal (log lama menunjukkan 2.2 m/s), karena terlalu
            // cepat untuk berhenti jika gate baru terdeteksi dekat.
            const auto & search_wp = waypoints_->at(current_wp_);
            const double search_alt_error = WaypointHandler::altitudeError(
                currentAltitudeDown(), search_wp.d);
            control_->publishHeartbeat(false);
            control_->sendVelocitySetpoint(
                mission_gate_search_speed_m_s_ * std::cos(leg_bearing),
                mission_gate_search_speed_m_s_ * std::sin(leg_bearing),
                waypoints_->computeVerticalVelocity(search_alt_error),
                leg_bearing);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "[MISSION][%s][GATE] SEARCH SLOW: %.2f/%.2fm, "
                "gate belum terlihat; TRACK NORMAL DIBLOK.",
                waypoint_label.c_str(), search_traveled,
                mission_gate_search_distance_m_);
            return true;
        }

        mission_gate_active_ = true;
        mission_gate_stage_ = GatePassStage::CENTER;
        mission_gate_center_target_ = vehicle_.position;
        // Heading awal hanya hold saat menunggu frame berikutnya. Begitu dua
        // tiang valid, target yaw diganti oleh normal bidang gate, bukan
        // heading RelativePath lama.
        mission_gate_heading_ = vehicle_.yaw;
        mission_gate_entry_heading_ = vehicle_.yaw;
        mission_gate_heading_target_valid_ = false;
        mission_gate_center_lock_captured_ = false;
        mission_gate_centered_ticks_ = 0;
        mission_gate_stage_started_s_ = now_s;
        RCLCPP_INFO(this->get_logger(),
            "[MISSION][%s][GATE] %s: APPROACH DIBLOK -> CENTER/HOLD "
            "(sisa=%.2fm, full_geometry=%s).",
            waypoint_label.c_str(),
            gate_observed ? "HAZARD DETECTED" : "SEARCH LIMIT TANPA DETEKSI",
            along_track_remaining,
            gate_confirmed ? "yes" : "no");
    }

    const auto & wp = waypoints_->at(current_wp_);
    const double elapsed = now_s - mission_gate_stage_started_s_;
    if (mission_gate_stage_ == GatePassStage::CENTER) {
        control_->publishHeartbeat(true);

        bool heading_measurement_accepted = false;
        if (sample_fresh && gate_confirmed &&
            gate_centering_latest_.heading_valid) {
            // Lidar mengukur heading relatif terhadap badan sekarang.
            // Low-pass target di dunia menghindari jitter bin point-cloud,
            // namun tetap mengunci normal gate dengan tegas.
            const double measured_gate_heading = std::atan2(
                std::sin(vehicle_.yaw + gate_centering_latest_.heading_error_rad),
                std::cos(vehicle_.yaw + gate_centering_latest_.heading_error_rad));
            const double correction_from_pilot = std::atan2(
                std::sin(measured_gate_heading - mission_gate_entry_heading_),
                std::cos(measured_gate_heading - mission_gate_entry_heading_));
            heading_measurement_accepted =
                std::abs(correction_from_pilot) <=
                    gate_centering_max_heading_correction_rad_;
            if (heading_measurement_accepted &&
                !mission_gate_heading_target_valid_)
            {
                mission_gate_heading_ = measured_gate_heading;
                mission_gate_heading_target_valid_ = true;
            } else if (heading_measurement_accepted) {
                const double target_delta = std::atan2(
                    std::sin(measured_gate_heading - mission_gate_heading_),
                    std::cos(measured_gate_heading - mission_gate_heading_));
                mission_gate_heading_ = std::atan2(
                    std::sin(mission_gate_heading_ + 0.35 * target_delta),
                    std::cos(mission_gate_heading_ + 0.35 * target_delta));
            }

            if (!heading_measurement_accepted) {
                // Geometri/noise tidak boleh membuat yaw besar. Tetap tahan
                // yaw pilot dan jangan pernah membuka ADVANCE.
                mission_gate_heading_ = mission_gate_entry_heading_;
                mission_gate_heading_target_valid_ = false;
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "[MISSION][%s][GATE] HEADING REJECTED: koreksi %.1fdeg "
                    "> batas %.1fdeg; HOLD yaw pilot, MAJU DIBLOK.",
                    waypoint_label.c_str(), radToDeg(correction_from_pilot),
                    radToDeg(gate_centering_max_heading_correction_rad_));
            }
        }

        // Koreksi lateral hanya boleh memakai geometry dua tiang dengan
        // lebar valid, atau mode one-side yang memang eksplisit. Pasangan
        // dua objek dengan lebar salah adalah clutter: HOLD, jangan diikuti.
        const bool lateral_measurement_usable = sample_fresh &&
            gate_centering_latest_.valid &&
            (gate_confirmed || gate_centering_latest_.one_side_only);
        if (lateral_measurement_usable) {

            // Adaptif seperti centering YOLO: makin jauh dari tengah gerbang,
            // makin tegas dikejar (setpoint dipush cepat supaya PX4 segera
            // membangun kecepatan lateral, bukan merayap 0.03m/tick sepanjang
            // jalan) — makin dekat toleransi (0.2m default), makin diredam
            // supaya tidak overshoot ke tiang seberang.
            const double abs_lateral_error =
                std::abs(static_cast<double>(gate_centering_latest_.lateral_error_m));
            const double center_kp =
                abs_lateral_error > 0.4 ? 0.35 :
                abs_lateral_error > 0.15 ? 0.25 : 0.18;
            const double max_step_m =
                abs_lateral_error > 0.4 ? 0.12 :
                abs_lateral_error > 0.15 ? 0.06 : 0.03;
            const double step = std::clamp(
                center_kp *
                    static_cast<double>(gate_centering_latest_.lateral_error_m),
                -max_step_m, max_step_m);
            // Error lateral adalah body-right Livox, jadi harus diproyeksikan
            // memakai yaw badan AKTUAL (bukan heading leg lama) saat yaw dan
            // center berjalan bersamaan.
            const double right_n = -std::sin(vehicle_.yaw);
            const double right_e =  std::cos(vehicle_.yaw);
            if (!gate_centering_latest_.centered) {
                mission_gate_center_lock_captured_ = false;
                mission_gate_center_target_ = vehicle_.position;
                mission_gate_center_target_.north += step * right_n;
                mission_gate_center_target_.east  += step * right_e;
            }

            const bool geometry_centered =
                gate_confirmed &&
                gate_centering_latest_.centered &&
                gate_centering_latest_.width_valid &&
                !gate_centering_latest_.one_side_only &&
                heading_measurement_accepted &&
                mission_gate_heading_target_valid_ &&
                std::abs(gate_centering_latest_.heading_error_rad) <=
                    gate_centering_heading_tolerance_rad_;
            if (geometry_centered && !mission_gate_center_lock_captured_) {
                mission_gate_center_target_ = vehicle_.position;
                mission_gate_center_lock_captured_ = true;
            }
            const double center_hold_error = std::hypot(
                mission_gate_center_target_.north - vehicle_.position.north,
                mission_gate_center_target_.east - vehicle_.position.east);
            const double center_speed = std::hypot(
                vehicle_.velocity.north, vehicle_.velocity.east);
            constexpr double GATE_LOCK_POSITION_M = 0.06;
            constexpr double GATE_LOCK_SPEED_M_S = 0.08;
            const bool physically_locked = geometry_centered &&
                center_hold_error <= GATE_LOCK_POSITION_M &&
                center_speed <= GATE_LOCK_SPEED_M_S;
            if (physically_locked) {
                ++mission_gate_centered_ticks_;
            } else {
                mission_gate_centered_ticks_ = 0;
            }
        } else {
            mission_gate_centered_ticks_ = 0;
        }

        control_->sendPositionSetpoint(
            toPx4North(mission_gate_center_target_.north),
            toPx4East(mission_gate_center_target_.east),
            toPx4DownForAltitudeTarget(wp.d),
            mission_gate_heading_);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[MISSION][%s][GATE] CENTER/HOLD: fresh=%s full=%s width=%.2fm "
            "lateral=%.2fm yaw=%.1fdeg speed=%.2fm/s stable=%d/%d | "
            "MAJU=BLOK elapsed=%.1fs",
            waypoint_label.c_str(), sample_fresh ? "yes" : "no",
            gate_confirmed ? "yes" : "no",
            gate_centering_latest_.detected_width_m,
            gate_centering_latest_.lateral_error_m,
            radToDeg(gate_centering_latest_.heading_error_rad),
            std::hypot(vehicle_.velocity.north, vehicle_.velocity.east),
            mission_gate_centered_ticks_,
            mission_gate_required_centered_ticks_, elapsed);

        if (mission_gate_centered_ticks_ >=
            mission_gate_required_centered_ticks_)
        {
            mission_gate_stage_ = GatePassStage::ADVANCE;
            mission_gate_stage_started_s_ = now_s;
            mission_gate_advance_start_ = vehicle_.position;
            // "Maju sejauh sisanya": target ADVANCE = sisa jarak leg yang
            // sesungguhnya saat lock selesai, bukan gate_pass_distance_m_
            // tetap — badan hampir tidak bergeser maju selama CENTER
            // (hanya koreksi lateral), jadi along_track_remaining di tick
            // ini masih representatif.
            mission_gate_advance_target_m_ = std::max(
                along_track_remaining,
                static_cast<double>(gate_centering_target_forward_distance_m_));
            RCLCPP_INFO(this->get_logger(),
                "[MISSION][%s][GATE] LOCKED: center + heading %.1fdeg; ADVANCE lurus %.2fm (sisa leg).",
                waypoint_label.c_str(), radToDeg(mission_gate_heading_),
                mission_gate_advance_target_m_);
        } else if (elapsed >= gate_pass_center_timeout_s_) {
            RCLCPP_ERROR(this->get_logger(),
                "[%s] GATE CENTER timeout — FAILSAFE LAND.",
                waypoint_label.c_str());
            control_->sendLandCommand();
            phase_ = Phase::WAIT_DISARM;
        }
        return true;
    }

    const double dn =
        vehicle_.position.north - mission_gate_advance_start_.north;
    const double de =
        vehicle_.position.east - mission_gate_advance_start_.east;
    const double traveled =
        dn * std::cos(mission_gate_heading_) +
        de * std::sin(mission_gate_heading_);

    // Sebelum badan mencapai bidang gate, hilangnya deteksi berarti tidak
    // aman untuk lanjut. Setelah jarak target gate terlewati, tiang memang
    // wajar keluar ROI belakang; lanjut lurus dengan koreksi lateral nol.
    const bool cleared_gate_plane =
        traveled >=
        static_cast<double>(gate_centering_target_forward_distance_m_);
    if (!sample_fresh && !cleared_gate_plane) {
        control_->publishHeartbeat(true);
        mission_gate_center_target_ = vehicle_.position;
        control_->sendPositionSetpoint(
            toPx4North(vehicle_.position.north),
            toPx4East(vehicle_.position.east),
            toPx4DownForAltitudeTarget(wp.d),
            mission_gate_heading_);
        mission_gate_stage_ = GatePassStage::CENTER;
        mission_gate_centered_ticks_ = 0;
        mission_gate_stage_started_s_ = now_s;
        RCLCPP_WARN(this->get_logger(),
            "[%s] GATE hilang sebelum terlewati — berhenti dan CENTER ulang.",
            waypoint_label.c_str());
        return true;
    }

    control_->publishHeartbeat(false);
    // Sesudah CENTER+heading terkunci, maju harus murni sepanjang normal
    // gate. Koreksi lateral realtime di sini justru membuat vektor maju
    // miring dan merupakan sumber tabrakan gate pada uji sebelumnya.
    const double vn = gate_pass_forward_velocity_m_s_ * std::cos(mission_gate_heading_);
    const double ve = gate_pass_forward_velocity_m_s_ * std::sin(mission_gate_heading_);
    const double alt_error =
        WaypointHandler::altitudeError(currentAltitudeDown(), wp.d);
    control_->sendVelocitySetpoint(
        vn, ve, waypoints_->computeVerticalVelocity(alt_error),
        mission_gate_heading_);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "[MISSION][%s][GATE] ADVANCE STRAIGHT: %.2f/%.2fm heading=%.1fdeg lateral_v=0.00",
        waypoint_label.c_str(), traveled, mission_gate_advance_target_m_,
        radToDeg(mission_gate_heading_));

    if (traveled >= mission_gate_advance_target_m_) {
        // Geser target dan seluruh sisa path hanya pada sumbu lateral.
        // Komponen forward tidak digeser agar sisa jarak waypoint tetap
        // benar; bearing leg dibekukan sampai waypoint ini selesai.
        const double start_n =
            current_wp_ == 0 ? 0.0 : waypoints_->at(current_wp_ - 1).n;
        const double start_e =
            current_wp_ == 0 ? 0.0 : waypoints_->at(current_wp_ - 1).e;
        const double locked_right_n = -std::sin(mission_gate_heading_);
        const double locked_right_e =  std::cos(mission_gate_heading_);
        const double cross_track =
            (vehicle_.position.north - start_n) * locked_right_n +
            (vehicle_.position.east - start_e) * locked_right_e;
        waypoints_->translateWaypointsFrom(
            current_wp_, cross_track * locked_right_n,
            cross_track * locked_right_e);
        leg_bearing_override_ = mission_gate_heading_;
        leg_bearing_override_valid_ = true;
        mission_gate_active_ = false;
        mission_gate_completed_for_wp_ = true;
        RCLCPP_INFO(this->get_logger(),
            "[MISSION][%s][GATE] PASSED: path mengikuti center terbaru "
            "(lateral shift %.3fm), lanjut APPROACH lurus.",
            waypoint_label.c_str(), cross_track);
    } else {
        // Sisa leg (dan karenanya jarak ADVANCE) bisa lebih panjang dari
        // gate_pass_distance_m_ lama yang dipakai untuk kalibrasi timeout
        // tetap — skalakan supaya leg panjang tidak LAND prematur.
        const double expected_travel_s = mission_gate_advance_target_m_ /
            std::max(0.2, static_cast<double>(gate_pass_forward_velocity_m_s_));
        const double advance_timeout_s = std::max(
            static_cast<double>(gate_pass_advance_timeout_s_),
            expected_travel_s * 1.5 + 3.0);
        if (elapsed >= advance_timeout_s) {
            RCLCPP_ERROR(this->get_logger(),
                "[%s] GATE ADVANCE timeout (%.1fs) — FAILSAFE LAND.",
                waypoint_label.c_str(), advance_timeout_s);
            control_->sendLandCommand();
            phase_ = Phase::WAIT_DISARM;
        }
    }
    return true;
}

// ==================================================================
// GATE_PASS — start_mode:=gate_pass. CENTER (diam, koreksi lateral
// sampai stabil) lalu ADVANCE (maju lurus menembus gerbang, terus
// dikoreksi lateral). Lihat GatePassStage di mission_manager.h dan
// docs/LIVOX_MID360_INTEGRATION.md §9 untuk kenapa ini fase terpisah
// total dari runMission()/ArUco.
// ==================================================================

void MissionManager::runGatePassMission()
{
    // Kill-switch ganda: gate_centering_enable_ wajib true di sini. Kalau
    // false, tidak ada sumber koreksi lateral sama sekali — jangan pernah
    // masuk CENTER/ADVANCE tanpa itu (lihat Context di plan: "never fly
    // the corridor-advance stage without a live, engaged detector").
    if (!gate_centering_enable_) {
        RCLCPP_ERROR(this->get_logger(),
            "GATE_PASS: gate_centering_enable_ mati — tidak ada sumber "
            "koreksi lateral. FAILSAFE: LAND.");
        control_->publishHeartbeat(true);
        control_->sendLandCommand();
        phase_ = Phase::WAIT_DISARM;
        return;
    }

    const double now_s = this->now().seconds();
    const double elapsed = (this->now() - gate_pass_stage_started_at_).seconds();

    if (gate_pass_stage_ == GatePassStage::CENTER) {
        control_->publishHeartbeat(true);

        const bool sample_fresh =
            last_gate_sample_s_ >= 0.0 &&
            now_s - last_gate_sample_s_ <= 0.35;
        const bool engaged =
            sample_fresh && gate_centering_debounce_->isEngaged(now_s);

        if (engaged) {
            if (!gate_pass_engaged_prev_) {
                RCLCPP_INFO(this->get_logger(),
                    "GATE PASS CENTER: lock engaged, mengoreksi ke tengah gerbang.");
            }
            gate_pass_engaged_prev_ = true;

            // Closed-loop dari posisi aktual, identik prinsipnya dengan
            // centering ArUco/YOLO — adaptif terhadap besar error (lihat
            // runMissionGateAssist()). Target tidak dihitung dari hover
            // anchor lama karena itu dapat menarik drone kembali.
            const double abs_gate_lateral_error =
                std::abs(static_cast<double>(gate_centering_latest_.lateral_error_m));
            const double gate_center_kp =
                abs_gate_lateral_error > 0.4 ? 0.35 :
                abs_gate_lateral_error > 0.15 ? 0.25 : 0.18;
            const double max_gate_center_step_m =
                abs_gate_lateral_error > 0.4 ? 0.12 :
                abs_gate_lateral_error > 0.15 ? 0.06 : 0.03;
            const double lateral_step = std::clamp(
                gate_center_kp *
                    static_cast<double>(gate_centering_latest_.lateral_error_m),
                -max_gate_center_step_m, max_gate_center_step_m);
            const double right_n = -std::sin(takeoff_hold_yaw_);
            const double right_e =  std::cos(takeoff_hold_yaw_);
            if (!gate_centering_latest_.centered) {
                gate_pass_center_target_ = vehicle_.position;
                gate_pass_center_target_.north += lateral_step * right_n;
                gate_pass_center_target_.east  += lateral_step * right_e;
            }

            // Satu tiang boleh membantu arah koreksi, tetapi tidak cukup
            // aman untuk menyatakan gerbang siap ditembus. ADVANCE hanya
            // boleh dimulai setelah dua tiang dan lebar gate tervalidasi.
            const bool gate_geometry_confirmed =
                gate_centering_latest_.centered &&
                gate_centering_latest_.width_valid &&
                !gate_centering_latest_.one_side_only;
            if (gate_geometry_confirmed) {
                ++gate_pass_centered_ticks_;
            } else {
                gate_pass_centered_ticks_ =
                    std::max(0, gate_pass_centered_ticks_ - 1);
            }
        } else {
            gate_pass_engaged_prev_ = false;
            gate_pass_centered_ticks_ = 0;
        }

        control_->sendPositionSetpoint(
            toPx4North(gate_pass_center_target_.north),
            toPx4East(gate_pass_center_target_.east),
            toPx4DownForAltitudeTarget(takeoffTargetDown()),
            takeoff_hold_yaw_);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "GATE PASS CENTER: engaged=%s centered_ticks=%d/%d elapsed=%.1fs",
            engaged ? "yes" : "no",
            gate_pass_centered_ticks_, gate_pass_required_centered_ticks_, elapsed);

        if (gate_pass_centered_ticks_ >= gate_pass_required_centered_ticks_) {
            RCLCPP_INFO(this->get_logger(), "=== GATE PASS: CENTERED, ADVANCE ===");
            gate_pass_stage_ = GatePassStage::ADVANCE;
            gate_pass_stage_started_at_ = this->now();
            gate_pass_advance_start_position_ = vehicle_.position;
            // Bekukan heading saat mulai menembus gate. Menggunakan yaw
            // aktual setiap tick membuat arah forward ikut berkelok ketika
            // estimator yaw sedikit noise.
            gate_pass_advance_start_yaw_ = takeoff_hold_yaw_;
            return;
        }

        if (elapsed > gate_pass_center_timeout_s_) {
            RCLCPP_ERROR(this->get_logger(),
                "GATE PASS CENTER timeout %.1fs — gerbang tidak pernah stabil "
                "di tengah. FAILSAFE: LAND.", elapsed);
            control_->publishHeartbeat(true);
            control_->sendLandCommand();
            phase_ = Phase::WAIT_DISARM;
        }
        return;
    }

    // ── ADVANCE ──────────────────────────────────────────────────────
    const bool sample_fresh =
        last_gate_sample_s_ >= 0.0 &&
        now_s - last_gate_sample_s_ <= 0.50;
    const double dn = vehicle_.position.north - gate_pass_advance_start_position_.north;
    const double de = vehicle_.position.east - gate_pass_advance_start_position_.east;
    const double cy = std::cos(gate_pass_advance_start_yaw_);
    const double sy = std::sin(gate_pass_advance_start_yaw_);
    const double traveled_forward = dn * cy + de * sy;
    const bool cleared_gate_plane =
        traveled_forward >=
        static_cast<double>(gate_centering_target_forward_distance_m_);
    if (!sample_fresh && !cleared_gate_plane) {
        // Jangan lanjut buta ketika gate hilang. Berhenti dengan position
        // hold, lalu kembali ke CENTER agar detector harus lock ulang.
        control_->publishHeartbeat(true);
        control_->sendPositionSetpoint(
            toPx4North(vehicle_.position.north),
            toPx4East(vehicle_.position.east),
            toPx4DownForAltitudeTarget(takeoffTargetDown()),
            gate_pass_advance_start_yaw_);
        gate_pass_center_target_ = vehicle_.position;
        gate_pass_centered_ticks_ = 0;
        gate_pass_stage_ = GatePassStage::CENTER;
        gate_pass_stage_started_at_ = this->now();
        RCLCPP_WARN(this->get_logger(),
            "GATE PASS ADVANCE: cloud/deteksi stale — berhenti dan lock ulang.");
        return;
    }

    control_->publishHeartbeat(false);  // velocity mode

    // lateral_error_m dipakai live tiap tick (bukan lockedTarget beku) —
    // sama seperti advanceThroughLivoxCorridor() aslinya terus mengoreksi
    // sambil maju. Kalau sample tidak valid frame ini, vy_body=0 (jangan
    // ekstrapolasi arah yang tidak diketahui).
    const double lateral_error_m =
        sample_fresh && gate_centering_latest_.valid
        ? gate_centering_latest_.lateral_error_m
        : 0.0;
    double vy_body = gate_pass_proportional_gain_ * lateral_error_m;
    vy_body = std::max(
        static_cast<double>(-gate_pass_max_lateral_velocity_m_s_),
        std::min(static_cast<double>(gate_pass_max_lateral_velocity_m_s_), vy_body));

    const double yaw = gate_pass_advance_start_yaw_;
    const double vn = gate_pass_forward_velocity_m_s_ * std::cos(yaw) - vy_body * std::sin(yaw);
    const double ve = gate_pass_forward_velocity_m_s_ * std::sin(yaw) + vy_body * std::cos(yaw);

    const double alt_error = WaypointHandler::altitudeError(
        currentAltitudeDown(), takeoffTargetDown());
    const double vz = waypoints_->computeVerticalVelocity(alt_error);

    control_->sendVelocitySetpoint(vn, ve, vz, yaw);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "GATE PASS ADVANCE: traveled=%.2f/%.2fm lat_err=%.3fm vy_body=%.3f elapsed=%.1fs",
        traveled_forward, gate_pass_distance_m_, lateral_error_m, vy_body, elapsed);

    if (traveled_forward >= gate_pass_distance_m_) {
        RCLCPP_INFO(this->get_logger(),
            "=== GATE PASS: ADVANCE COMPLETE (%.2fm ditempuh) — LAND ===",
            traveled_forward);
        gate_pass_stage_ = GatePassStage::DONE;
        control_->publishHeartbeat(true);
        control_->sendLandCommand();
        phase_ = Phase::WAIT_DISARM;
        return;
    }

    if (elapsed > gate_pass_advance_timeout_s_) {
        RCLCPP_ERROR(this->get_logger(),
            "GATE PASS ADVANCE timeout %.1fs (baru %.2f/%.2fm) — FAILSAFE: LAND.",
            elapsed, traveled_forward, gate_pass_distance_m_);
        control_->publishHeartbeat(true);
        control_->sendLandCommand();
        phase_ = Phase::WAIT_DISARM;
    }
}

// ==================================================================
// LAND_CMD — kirim land command (heartbeat tetap wajib jalan)
// ==================================================================

void MissionManager::runLandCmd()
{
    control_->publishHeartbeat(true);
    control_->sendLandCommand();
    phase_ = Phase::WAIT_DISARM;
}

// ==================================================================
// WAIT_DISARM — tunggu konfirmasi PX4 sudah disarm
// ==================================================================

void MissionManager::runWaitDisarm()
{
    if (vehicle_.arming_state ==
    px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED)
    {
        RCLCPP_INFO(this->get_logger(), "========================");
        RCLCPP_INFO(this->get_logger(), "MISSION COMPLETE");
        RCLCPP_INFO(this->get_logger(), "========================");
        timer_->cancel();
    }
}

}  // namespace px4
