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
    RelativePath path(/*start_altitude_agl_m=*/1.15);
    path.forward(4.9)
        .turnLeft(90.0)
        .forward(5.9);
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
        [this](uint8_t arming_state) {
            onStatusUpdate(arming_state);
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
        "Ground lock (YOLO): %s (fx=%.1fpx, fy=%.1fpx) — dipakai state machine untuk centering WP1 sebelum gripper",
        ground_lock_enable_ ? "enabled" : "disabled",
        yolo_camera_fx_px_, yolo_camera_fy_px_);
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
    post_yaw_correction_active_ = false;
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
    marker_center_target_ = {};
    marker_heading_aligned_ticks_ = 0;
    marker_heading_best_error_rad_ = 0.0;
    marker_heading_best_yaw_ = vehicle_.yaw;
    marker_heading_align_started_s_ = 0.0;
    mission_gate_active_ = false;
    mission_gate_completed_for_wp_ = false;
    mission_gate_stage_ = GatePassStage::CENTER;
    mission_gate_centered_ticks_ = 0;
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
    if (!gripper_drop_enable_ ||
        gripper_drop_completed_ ||
        current_wp_ != gripper_drop_after_wp_)
    {
        return false;
    }

    control_->publishHeartbeat(true);
    control_->sendPositionSetpoint(
        toPx4North(hold_north),
        toPx4East(hold_east),
        toPx4DownForAltitudeTarget(hold_down),
        hold_yaw);

    switch (gripper_drop_state_) {
        case GripperDropState::IDLE:
            publishGripperCommand("open");
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

void MissionManager::onStatusUpdate(uint8_t arming_state)
{
    ++status_update_count_;
    vehicle_.arming_state = arming_state;
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
    last_marker_sample_s_ = this->now().seconds();
    marker_feedback_active_ = true;
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
        "Gate centering (Livox): %s lat_err=%.3fm forward=%.2fm width=%.2fm centered=%s",
        gate_centering_latest_.one_side_only ? "one-side" : "two-pole",
        gate_centering_latest_.lateral_error_m,
        gate_centering_latest_.forward_distance_m,
        gate_centering_latest_.detected_width_m,
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
        case Phase::TAKEOFF_MARKER: runTakeoffMarker(); break;
        case Phase::MISSION:     runMission();      break;
        case Phase::GATE_PASS:   runGatePassMission(); break;
        case Phase::LAND_CMD:    runLandCmd();      break;
        case Phase::WAIT_DISARM: runWaitDisarm();   break;
    }
}

// ==================================================================
// INIT — diam 5 tick, lalu set offboard mode + arm
// ==================================================================

void MissionManager::runInit()
{
    control_->publishHeartbeat(true);

    control_->sendPositionSetpoint(
        toPx4North(vehicle_.position.north),
        toPx4East(vehicle_.position.east),
        origin_down_ + vehicle_.position.down,
        vehicle_.yaw);

    ++counter_;
    RCLCPP_INFO(this->get_logger(), "INIT... (%d/5)", counter_);

    if (counter_ >= 5) {
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
        phase_   = Phase::TAKEOFF;
        counter_ = 0;
        RCLCPP_INFO(this->get_logger(),
            "ARM terkonfirmasi. === INITIAL ALTITUDE ALIGN (%s) ===",
            start_mode_param_.c_str());
        return;
    }

    ++counter_;
    if (counter_ >= 10) {   // 1 detik @ 10Hz — cukup untuk PX4 memproses arm
        if (status_update_count_ == 0) {
            RCLCPP_WARN(this->get_logger(),
                "Topik status PX4 tidak memberi data (callback 0x) — lanjut "
                "TAKEOFF tanpa verifikasi arm eksplisit.");
        } else if (vehicle_.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
            RCLCPP_ERROR(this->get_logger(),
                "Status PX4 tersedia dan menunjukkan BUKAN armed setelah 1 "
                "detik — misi dibatalkan, drone tidak pernah lepas landas.");
            timer_->cancel();
            return;
        }
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

    control_->sendPositionSetpoint(
        toPx4North(takeoff_hold_north_),
        toPx4East(takeoff_hold_east_),
        toPx4DownForAltitudeTarget(takeoffTargetDown()),
        takeoff_hold_yaw_);

    const double altitude_error =
        WaypointHandler::altitudeError(currentAltitudeDown(), takeoffTargetDown());
    const bool altitude_ready =
        start_mode_ == StartMode::AIRBORNE_HANDOFF
        ? std::abs(altitude_error) <= 0.15
        : currentAltitudeDown() <= takeoffTargetDown() + 0.15;

    if (start_mode_ == StartMode::AIRBORNE_HANDOFF) {
        if (altitude_ready) {
            ++initial_altitude_stable_ticks_;
        } else {
            initial_altitude_stable_ticks_ = 0;
        }
    } else {
        initial_altitude_stable_ticks_ = altitude_ready ? 10 : 0;
    }

    RCLCPP_INFO(this->get_logger(),
        "Altitude align[%s/%s]: %.2fm -> %.2fm err=%.2fm stable=%d/10 | "
        "yaw hold=%.1fdeg | holdN=%.3f holdE=%.3f",
        start_mode_param_.c_str(),
        altitudeSourceLabel(),
        currentAltitudeAgl(), -takeoffTargetDown(),
        altitude_error,
        initial_altitude_stable_ticks_,
        radToDeg(takeoff_hold_yaw_),
        takeoff_hold_north_, takeoff_hold_east_);

    if (start_mode_ == StartMode::TAKEOFF ? altitude_ready : initial_altitude_stable_ticks_ >= 10) {
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

        // Lanjutkan SEARCH dari target horizontal terakhir yang dipakai saat
        // HOVER. Tanpa ini, SEARCH kembali ke anchor takeoff lama dan dapat
        // menarik drone mundur/menyamping sesaat setelah hover selesai.
        vehicle_.hover_position.north = hold_north;
        vehicle_.hover_position.east = hold_east;
        marker_center_target_.north = hold_north;
        marker_center_target_.east = hold_east;
        phase_ = Phase::TAKEOFF_MARKER;
        waypoint_phase_ = WaypointPhase::SEARCH_MARKER;
        marker_search_started_at_ = this->now();
        marker_search_altitude_m_ = -takeoffTargetDown();
        marker_search_direction_ = 1;
        marker_search_horizontal_m_ = 0.0;
        marker_search_horizontal_direction_ = 1;
        RCLCPP_INFO(this->get_logger(),
            "=== TAKEOFF COMPLETE: SEARCH ARUCO WP1 ===");
    }
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

    const double base_altitude_m = -takeoffTargetDown();
    const bool is_handoff = start_mode_ == StartMode::AIRBORNE_HANDOFF;
    // Pada handoff, yaw takeover dan anchor horizontal harus tetap menjadi
    // referensi sampai pasangan marker siap melakukan alignment heading.
    const double heading = is_handoff ? takeoff_hold_yaw_ : missionReferenceYaw();

    if (waypoint_phase_ == WaypointPhase::SEARCH_MARKER) {
        // Marker memang ditempatkan di sekitar titik takeoff. Tahan anchor
        // selama 5 detik, lalu lakukan pencarian melingkar SANGAT kecil dan
        // lambat. Radius/altitude dibatasi ketat agar tetap bisa mengompensasi
        // selisih akibat angin tanpa terlihat sebagai gerak maju sendiri.
        const double search_s = (this->now() - marker_search_started_at_).seconds();
        const double active_s = std::max(0.0, search_s - 5.0);
        // Handoff mendapat search yang lebih sempit dan tanpa perubahan
        // altitude. Cabang takeoff sengaja dipertahankan persis seperti tuning
        // penerbangan sebelumnya (radius 6 cm, naik maksimal 8 cm).
        const double max_radius_m = is_handoff ? 0.03 : 0.06;
        const double radius_rate_m_s = is_handoff ? 0.003 : 0.006;
        const double radius = std::min(max_radius_m, active_s * radius_rate_m_s);
        const double angle = active_s * 0.30;
        marker_search_altitude_m_ = is_handoff
            ? base_altitude_m
            : base_altitude_m + std::min(0.08, active_s * 0.008);
        const double search_anchor_n = is_handoff
            ? takeoff_hold_north_
            : vehicle_.hover_position.north;
        const double search_anchor_e = is_handoff
            ? takeoff_hold_east_
            : vehicle_.hover_position.east;
        const double search_n =
            search_anchor_n + radius * std::cos(angle);
        const double search_e =
            search_anchor_e + radius * std::sin(angle);
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
                "[WP1-%s] mencari ArUco: %.1fs alt %.2f->%.2fm radius %.2fm valid=%d/5",
                is_handoff ? "HANDOFF" : "TAKEOFF",
                search_s, currentAltitudeAgl(), marker_search_altitude_m_,
                radius, marker_stable_frames_);
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
        double target_d = takeoffTargetDown();

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
            if (measured_offset > marker_center_tolerance_m_) {
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
            if (marker_stable_frames_ >= 6) {
                marker_feedback_locked_ = true;
                RCLCPP_INFO(this->get_logger(), "[ARUCO] LOCK SUCCESS");
            }
        } else {
            target_n = marker_center_target_.north;
            target_e = marker_center_target_.east;

            // Sama seperti fix di runMission(): marker hilang dari frame
            // terlalu lama (mis. kedorong angin) tidak boleh menahan target
            // terakhir selamanya — kembali ke SEARCH_MARKER supaya drone
            // aktif mencari lagi (radius sama seperti pencarian awal:
            // is_handoff ? 0.03m : 0.06m, lihat blok SEARCH_MARKER di atas).
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
            if (start_mode_ == StartMode::AIRBORNE_HANDOFF &&
                marker_heading_align_enable_)
            {
                waypoint_phase_ = WaypointPhase::ALIGN_MARKER_HEADING;
                marker_heading_align_started_s_ = this->now().seconds();
                marker_heading_aligned_ticks_ = 0;
                marker_heading_best_error_rad_ = PI;
                marker_heading_best_yaw_ = vehicle_.yaw;
                RCLCPP_INFO(this->get_logger(),
                    "[ARUCO-HEADING] Center locked; mulai align garis marker besar->kecil.");
                return;
            }
            resetWaypointVisionState();
            rebaseMissionOriginToCurrentPosition();
            phase_ = Phase::MISSION;
            RCLCPP_INFO(this->get_logger(), "=== START MISSION (ArUco locked) ===");
        }
    }

    if (waypoint_phase_ == WaypointPhase::ALIGN_MARKER_HEADING) {
        control_->publishHeartbeat(true);

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
        const auto * front_marker = find_marker(marker_heading_front_id_);
        const bool fresh_centers =
            marker_centers_available_ && marker_centers_age_s <= 0.35;
        const bool have_pair = fresh_centers && back_marker && front_marker;

        double yaw_command = vehicle_.yaw;
        bool heading_locked = false;
        if (have_pair) {
            const double dx = front_marker->x_px - back_marker->x_px;
            const double dy = front_marker->y_px - back_marker->y_px;
            const double pair_dist_px = std::hypot(dx, dy);
            if (pair_dist_px >= 20.0) {
                const double heading_error = std::atan2(dx, -dy);
                if (std::abs(heading_error) < std::abs(marker_heading_best_error_rad_)) {
                    marker_heading_best_error_rad_ = heading_error;
                    marker_heading_best_yaw_ = vehicle_.yaw;
                }

                const double tolerance_rad = degToRad(marker_heading_tolerance_deg_);
                if (std::abs(heading_error) <= tolerance_rad) {
                    ++marker_heading_aligned_ticks_;
                } else {
                    marker_heading_aligned_ticks_ = 0;
                }
                // Heading handoff menentukan seluruh arah leg pertama.
                // Delapan tick adalah baseline yang sudah dipakai pada
                // penerbangan stabil: cukup menolak noise tanpa membuat
                // alignment terlalu ketat dan mudah timeout.
                heading_locked = marker_heading_aligned_ticks_ >= 8;

                constexpr double MAX_YAW_STEP_RAD = 0.035;  // ~20deg/s @ 10Hz
                const double yaw_step = std::clamp(
                    marker_heading_yaw_sign_ * heading_error * 0.7,
                    -MAX_YAW_STEP_RAD, MAX_YAW_STEP_RAD);
                yaw_command = vehicle_.yaw + yaw_step;

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                    "[ARUCO-HEADING] dx=%.1f dy=%.1f dist=%.1fpx err=%.1fdeg "
                    "stable=%d/8 yaw_cmd=%.1fdeg",
                    dx, dy, pair_dist_px, radToDeg(heading_error),
                    marker_heading_aligned_ticks_, radToDeg(yaw_command));
            } else {
                marker_heading_aligned_ticks_ = 0;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "[ARUCO-HEADING] pair marker terlalu dekat di gambar (%.1fpx), tahan.",
                    pair_dist_px);
            }
        } else {
            marker_heading_aligned_ticks_ = 0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "[ARUCO-HEADING] menunggu marker pair id %d->%d (fresh=%d age=%.2fs).",
                marker_heading_back_id_, marker_heading_front_id_,
                fresh_centers ? 1 : 0, marker_centers_age_s);
        }

        control_->sendPositionSetpoint(
            toPx4North(marker_center_target_.north),
            toPx4East(marker_center_target_.east),
            toPx4DownForAltitudeTarget(takeoffTargetDown()),
            yaw_command);

        const bool timed_out = align_s >= marker_heading_timeout_s_;
        if (heading_locked || timed_out) {
            const double reference_yaw = heading_locked
                ? vehicle_.yaw
                : marker_heading_best_yaw_;
            origin_yaw_ = reference_yaw;
            override_mission_heading_ = true;
            mission_heading_deg_ = radToDeg(reference_yaw);
            mission_heading_correction_deg_ = 0.0;
            mission_waypoints_ready_ = false;
            ensureMissionWaypointsReady();
            resetWaypointVisionState();
            rebaseMissionOriginToCurrentPosition();
            phase_ = Phase::MISSION;
            RCLCPP_INFO(this->get_logger(),
                heading_locked
                ? "=== START MISSION (ArUco position + heading locked) ==="
                : "=== START MISSION (ArUco heading timeout, pakai yaw terbaik) ===");
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
        // Khusus WP1 (YOLO/box) sweep dilebarkan sampai 0.5m: toleransi
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
            : (yolo_centering_wp ? 0.50 : 0.06);
        const double radius_rate_m_s = handoff_style_wp2
            ? 0.003
            : (yolo_centering_wp ? 0.05 : 0.006);
        const double radius = std::min(max_radius_m, active_s * radius_rate_m_s);
        const double angle = active_s * 0.30;
        marker_search_altitude_m_ = handoff_style_wp2
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
        if (marker_stable_frames_ < 5) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[%s] mencari %s... %.1fs alt=%.2f->%.2fm radius=%.2fm valid=%d/5",
                label.c_str(), marker_source_label, search_s, currentAltitudeAgl(),
                marker_search_altitude_m_,
                radius, marker_stable_frames_);
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
        marker_center_target_ = vehicle_.position;
        RCLCPP_INFO(this->get_logger(),
            "[%s] %s VALID (5 sample) - mulai centering.", label.c_str(), marker_source_label);
    }

    if (waypoint_phase_ == WaypointPhase::CENTER_MARKER) {
        // Setelah masuk radius akhir, kunci posisi target di PX4 agar
        // pengendali posisi mengoreksi sisa error/drift selama dwell.
        // Kalau vision lock tidak aktif, cukup pakai target waypoint biasa
        // agar drone tetap stabil dan misi tetap jalan tanpa marker.
        double hold_north = wp.n;
        double hold_east  = wp.e;
        double hold_down  = wp.d;

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

        const double marker_age_s = this->now().seconds() - last_marker_sample_s_;
        if (marker_latest_sample_available_ && marker_age_s <= 0.35) {
            constexpr double CENTER_KP = 0.18;
            constexpr double MAX_CENTER_STEP_M = 0.03;
            const double correction_n = std::max(-MAX_CENTER_STEP_M,
                std::min(MAX_CENTER_STEP_M, CENTER_KP * marker_latest_offset_north_));
            const double correction_e = std::max(-MAX_CENTER_STEP_M,
                std::min(MAX_CENTER_STEP_M, CENTER_KP * marker_latest_offset_east_));
            const double measured_offset = std::hypot(
                marker_latest_offset_north_, marker_latest_offset_east_);
            if (measured_offset > marker_center_tolerance_m_) {
                marker_center_target_.north = vehicle_.position.north + correction_n;
                marker_center_target_.east  = vehicle_.position.east + correction_e;
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
            const double lock_tolerance_m = std::max(
                marker_center_tolerance_m_, 0.15);
            const bool within_threshold =
                horizontal_offset <= lock_tolerance_m;
            if (within_threshold) {
                ++marker_stable_frames_;
                RCLCPP_INFO(this->get_logger(),
                    "[%s] center terkonfirmasi=%d/6 | offset=%.3fm <= batas %.3fm (N=%.3f E=%.3f)",
                    marker_source_label, marker_stable_frames_, horizontal_offset,
                    lock_tolerance_m,
                    marker_latest_offset_north_, marker_latest_offset_east_);
            } else {
                marker_stable_frames_ = std::max(0, marker_stable_frames_ - 1);
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "[%s] offset N=%.3f E=%.3f | langkah N=%.3f E=%.3f",
                    marker_source_label, marker_latest_offset_north_, marker_latest_offset_east_,
                    correction_n, correction_e);
            }

            marker_latest_sample_available_ = false;
            if (marker_stable_frames_ >= 6) {
                marker_feedback_locked_ = true;
                RCLCPP_INFO(this->get_logger(), "[%s] LOCK SUCCESS", marker_source_label);
            }
        } else {
            hold_north = marker_center_target_.north;
            hold_east = marker_center_target_.east;

            // Marker/box hilang dari frame terlalu lama (mis. drone
            // kedorong angin sampai target keluar FOV) — tanpa ini, drone
            // cuma menahan target terakhir SELAMANYA, tidak pernah coba
            // mencari lagi. Kembali ke SEARCH_MARKER supaya sweep pencarian
            // aktif lagi (radius sama seperti pencarian awal WP ini).
            constexpr double MARKER_LOST_RESEARCH_S = 3.0;
            if (marker_age_s > MARKER_LOST_RESEARCH_S) {
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
    if (is_pure_alt) {
        control_->publishHeartbeat(false);
        control_->sendVelocitySetpoint(0.0, 0.0, vz, yaw_result.target_yaw);
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
            const double shift_elapsed_s =
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

            // Profil dua tahap untuk shift 10 cm:
            //   1) sisa >3 cm: velocity drive supaya 7 cm awal cepat dan
            //      tidak mudah kalah oleh angin;
            //   2) sisa <=3 cm: position hold supaya PX4 mengerem dan
            //      menstabilkan titik akhir tanpa overshoot.
            // Heartbeat selalu cocok dengan jenis setpoint pada cabangnya.
            constexpr double SHIFT_BRAKE_RADIUS_M = 0.030;
            constexpr double SHIFT_MIN_SPEED_M_S = 0.20;
            constexpr double SHIFT_MAX_SPEED_M_S = 0.30;
            constexpr double SHIFT_SPEED_KP = 2.5;
            const bool shift_driving = remaining > SHIFT_BRAKE_RADIUS_M;
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
                    vz, raw_bearing);
            } else {
                control_->publishHeartbeat(true);
                control_->sendPositionSetpoint(
                    toPx4North(post_yaw_correction_target_.north),
                    toPx4East(post_yaw_correction_target_.east),
                    toPx4DownForAltitudeTarget(wp.d), raw_bearing);
            }

            // Shift nominal 10 cm baru dianggap selesai saat setidaknya
            // sekitar 8,5 cm benar-benar tercapai. Toleransi lama 4 cm dapat
            // meloloskan shift ketika drone baru bergerak sekitar 6 cm.
            // Cabang ArUco-on dipertahankan persis seperti sebelumnya.
            // ArUco-off memakai radius realistis plus velocity gate supaya
            // tidak macet akibat noise 1,5 cm dan tidak lolos saat melintas.
            // DRIVE berhenti saat sisa 3 cm, tetapi shift baru dianggap
            // selesai setelah position controller benar-benar masuk 2 cm
            // terakhir dan laju lateral sudah rendah. Ini memastikan paling
            // sedikit sekitar 8 cm dari perintah 10 cm benar-benar tercapai
            // tanpa menunggu toleransi 1 cm yang mudah macet oleh noise/angin.
            constexpr double SHIFT_COMPLETE_RADIUS_M = 0.020;
            constexpr double SHIFT_SETTLE_SPEED_M_S = 0.10;
            const bool shift_stable =
                remaining <= SHIFT_COMPLETE_RADIUS_M &&
                horizontal_speed <= SHIFT_SETTLE_SPEED_M_S;
            const int required_shift_ticks = 3;
            constexpr double POST_YAW_SHIFT_TIMEOUT_S = 3.0;
            if (shift_stable) {
                ++post_yaw_correction_stable_ticks_;
            } else {
                post_yaw_correction_stable_ticks_ = 0;
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "[%s] POST-YAW SHIFT%s %s: t=%.1fs sisa=%.3fm "
                "cmd=%.2fm/s speed_xy=%.3fm/s stabil=%d/%d",
                label.c_str(), vision_lock_enable_ ? "" : " OFF",
                shift_driving ? "DRIVE" : "BRAKE",
                shift_elapsed_s, remaining, shift_command_speed, horizontal_speed,
                post_yaw_correction_stable_ticks_, required_shift_ticks);

            if (post_yaw_correction_stable_ticks_ >= required_shift_ticks ||
                shift_elapsed_s >= POST_YAW_SHIFT_TIMEOUT_S) {
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
                post_yaw_correction_active_ = false;
                yaw_locked_for_current_wp_ = true;
                yaw_hold_position_valid_ = false;
                RCLCPP_INFO(this->get_logger(),
                    "[%s] POST-YAW SHIFT COMPLETE%s: dN=%.3fm dE=%.3fm; "
                    "jalur direbase dan mulai maju lurus.",
                    label.c_str(),
                    shift_elapsed_s >= POST_YAW_SHIFT_TIMEOUT_S ? " (timeout aman)" : "",
                    rebase_n, rebase_e);
            }
            return;
        }

        if (!yaw_hold_position_valid_) {
            yaw_hold_position_ = vehicle_.position;
            yaw_hold_position_valid_ = true;
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
                "[%s] YAW START: tahan N=%.3f E=%.3f, target=%.1fdeg.",
                label.c_str(), yaw_hold_position_.north,
                yaw_hold_position_.east, radToDeg(raw_bearing));
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
            toPx4DownForAltitudeTarget(wp.d),
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
            constexpr double POST_YAW_SHIFT_M = 0.10;
            if (std::abs(completed_turn) >= MIN_SHIFT_TURN_RAD) {
                // Right-vector heading akhir: [-sin(yaw), cos(yaw)].
                // Turn kiri -> shift kanan; turn kanan -> shift kiri.
                const double shift_sign = completed_turn < 0.0 ? 1.0 : -1.0;
                post_yaw_correction_target_ = vehicle_.position;
                post_yaw_correction_target_.north +=
                    shift_sign * POST_YAW_SHIFT_M * -std::sin(raw_bearing);
                post_yaw_correction_target_.east +=
                    shift_sign * POST_YAW_SHIFT_M * std::cos(raw_bearing);
                post_yaw_correction_stable_ticks_ = 0;
                post_yaw_correction_started_s_ = this->now().seconds();
                post_yaw_correction_active_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "[%s] YAW COMPLETE %.1fdeg: shift 10cm ke %s.",
                    label.c_str(), radToDeg(completed_turn),
                    completed_turn < 0.0 ? "KANAN" : "KIRI");
            } else {
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

    constexpr double CROSS_TRACK_KP = 0.60;
    constexpr double MAX_CROSS_TRACK_SPEED_M_S = 0.25;
    // Kecepatan maju memakai satu-satunya formula resmi di
    // WaypointHandler: cruise piecewise di luar 2 m dan formula pengereman
    // baseline yang tidak berubah di dalam 2 m. Bearing tetap bearing leg;
    // koreksi samping kecil hanya menghapus drift, bukan mengubah arah misi.
    const double forward_speed = waypoints_->computeApproachSpeed(
        std::max(0.0, along_track_remaining));
    const double lateral_speed = std::clamp(
        cross_track_error * CROSS_TRACK_KP,
        -MAX_CROSS_TRACK_SPEED_M_S, MAX_CROSS_TRACK_SPEED_M_S);

    double vx = forward_speed * along_n + lateral_speed * lateral_n;
    double vy = forward_speed * along_e + lateral_speed * lateral_e;
    const double align = waypoints_->yawAlignmentFactor(raw_yaw_error);
    vx *= align;
    vy *= align;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "[%s] TRACK: along_rem=%.2fm cross_err=%.2fm v_forward=%.2f v_cross=%.2f",
        label.c_str(), along_track_remaining, cross_track_error,
        forward_speed * align, lateral_speed * align);
    control_->publishHeartbeat(false);
    control_->sendVelocitySetpoint(vx, vy, vz, raw_bearing);
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
    const bool gate_confirmed =
        sample_fresh &&
        gate_centering_latest_.valid &&
        gate_centering_latest_.width_valid &&
        !gate_centering_latest_.one_side_only;

    if (!mission_gate_active_) {
        // Jangan mulai pass kalau sisa leg tidak cukup untuk melewati gate
        // lalu masuk final-hold waypoint dengan aman.
        constexpr double FINAL_MARGIN_M = 0.60;
        if (!gate_confirmed ||
            along_track_remaining <=
                static_cast<double>(gate_pass_distance_m_) + FINAL_MARGIN_M)
        {
            return false;
        }

        mission_gate_active_ = true;
        mission_gate_stage_ = GatePassStage::CENTER;
        mission_gate_center_target_ = vehicle_.position;
        mission_gate_heading_ = leg_bearing;
        mission_gate_centered_ticks_ = 0;
        mission_gate_stage_started_s_ = now_s;
        RCLCPP_INFO(this->get_logger(),
            "[%s] GATE DETECTED: pause APPROACH, mulai CENTER.",
            waypoint_label.c_str());
    }

    const auto & wp = waypoints_->at(current_wp_);
    const double elapsed = now_s - mission_gate_stage_started_s_;
    const double right_n = -std::sin(mission_gate_heading_);
    const double right_e =  std::cos(mission_gate_heading_);

    if (mission_gate_stage_ == GatePassStage::CENTER) {
        control_->publishHeartbeat(true);

        if (sample_fresh && gate_centering_latest_.valid) {
            constexpr double CENTER_KP = 0.18;
            constexpr double MAX_STEP_M = 0.03;
            const double step = std::clamp(
                CENTER_KP *
                    static_cast<double>(gate_centering_latest_.lateral_error_m),
                -MAX_STEP_M, MAX_STEP_M);
            if (!gate_centering_latest_.centered) {
                mission_gate_center_target_ = vehicle_.position;
                mission_gate_center_target_.north += step * right_n;
                mission_gate_center_target_.east  += step * right_e;
            }

            const bool geometry_centered =
                gate_centering_latest_.centered &&
                gate_centering_latest_.width_valid &&
                !gate_centering_latest_.one_side_only;
            if (geometry_centered) {
                ++mission_gate_centered_ticks_;
            } else {
                mission_gate_centered_ticks_ =
                    std::max(0, mission_gate_centered_ticks_ - 1);
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
            "[%s] GATE CENTER: fresh=%s stable=%d/%d elapsed=%.1fs",
            waypoint_label.c_str(), sample_fresh ? "yes" : "no",
            mission_gate_centered_ticks_,
            gate_pass_required_centered_ticks_, elapsed);

        if (mission_gate_centered_ticks_ >=
            gate_pass_required_centered_ticks_)
        {
            mission_gate_stage_ = GatePassStage::ADVANCE;
            mission_gate_stage_started_s_ = now_s;
            mission_gate_advance_start_ = vehicle_.position;
            RCLCPP_INFO(this->get_logger(),
                "[%s] GATE CENTERED: mulai ADVANCE lurus.",
                waypoint_label.c_str());
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
    double lateral_velocity = 0.0;
    if (sample_fresh && gate_centering_latest_.valid) {
        lateral_velocity = std::clamp(
            static_cast<double>(gate_pass_proportional_gain_) *
                static_cast<double>(gate_centering_latest_.lateral_error_m),
            -static_cast<double>(gate_pass_max_lateral_velocity_m_s_),
            static_cast<double>(gate_pass_max_lateral_velocity_m_s_));
    }
    const double vn =
        gate_pass_forward_velocity_m_s_ * std::cos(mission_gate_heading_) +
        lateral_velocity * right_n;
    const double ve =
        gate_pass_forward_velocity_m_s_ * std::sin(mission_gate_heading_) +
        lateral_velocity * right_e;
    const double alt_error =
        WaypointHandler::altitudeError(currentAltitudeDown(), wp.d);
    control_->sendVelocitySetpoint(
        vn, ve, waypoints_->computeVerticalVelocity(alt_error),
        mission_gate_heading_);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "[%s] GATE ADVANCE: %.2f/%.2fm lateral_v=%.2fm/s fresh=%s",
        waypoint_label.c_str(), traveled, gate_pass_distance_m_,
        lateral_velocity, sample_fresh ? "yes" : "no");

    if (traveled >= gate_pass_distance_m_) {
        // Geser target dan seluruh sisa path hanya pada sumbu lateral.
        // Komponen forward tidak digeser agar sisa jarak waypoint tetap
        // benar; bearing leg dibekukan sampai waypoint ini selesai.
        const double start_n =
            current_wp_ == 0 ? 0.0 : waypoints_->at(current_wp_ - 1).n;
        const double start_e =
            current_wp_ == 0 ? 0.0 : waypoints_->at(current_wp_ - 1).e;
        const double cross_track =
            (vehicle_.position.north - start_n) * right_n +
            (vehicle_.position.east - start_e) * right_e;
        waypoints_->translateWaypointsFrom(
            current_wp_, cross_track * right_n, cross_track * right_e);
        leg_bearing_override_ = mission_gate_heading_;
        leg_bearing_override_valid_ = true;
        mission_gate_active_ = false;
        mission_gate_completed_for_wp_ = true;
        RCLCPP_INFO(this->get_logger(),
            "[%s] GATE PASSED: path mengikuti center terbaru "
            "(lateral shift %.3fm), lanjut APPROACH lurus.",
            waypoint_label.c_str(), cross_track);
    } else if (elapsed >= gate_pass_advance_timeout_s_) {
        RCLCPP_ERROR(this->get_logger(),
            "[%s] GATE ADVANCE timeout — FAILSAFE LAND.",
            waypoint_label.c_str());
        control_->sendLandCommand();
        phase_ = Phase::WAIT_DISARM;
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

            // Closed-loop satu langkah kecil dari posisi aktual, identik
            // prinsipnya dengan centering ArUco/YOLO. Target tidak dihitung
            // dari hover anchor lama karena itu dapat menarik drone kembali.
            constexpr double GATE_CENTER_KP = 0.18;
            constexpr double MAX_GATE_CENTER_STEP_M = 0.03;
            const double lateral_step = std::clamp(
                GATE_CENTER_KP *
                    static_cast<double>(gate_centering_latest_.lateral_error_m),
                -MAX_GATE_CENTER_STEP_M, MAX_GATE_CENTER_STEP_M);
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
