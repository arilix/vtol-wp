
#include "utils/mission_manager.h"

#include <cmath>

using namespace std::chrono_literals;

namespace px4
{

namespace
{
// ==================================================================
// WAYPOINTS GLOBAL (LLA)
// Format input: Latitude [deg], Longitude [deg], target altitude AGL [m].
//
// Nanti kalau koordinat GPS waypoint asli sudah ada, ganti isi
// defaultGlobalWaypoints() dengan lat/lon asli. MissionManager akan
// konversi otomatis ke NED lokal PX4 relatif ke GPS origin saat start.
//
// Dummy saat ini: kotak 1m x 1m dari titik start:
// WP1 utara 1m, WP2 timur 1m, WP3 selatan 1m, WP4 barat 1m.
// ==================================================================
struct GlobalWaypoint
{
    double lat_deg;
    double lon_deg;
    double altitude_agl_m;
};

constexpr double EARTH_RADIUS_M = 6378137.0;
constexpr double PI = 3.14159265358979323846;

double degToRad(double deg)
{
    return deg * PI / 180.0;
}

double radToDeg(double rad)
{
    return rad * 180.0 / PI;
}

GlobalWaypoint nedOffsetToGlobal(
    double origin_lat_deg, double origin_lon_deg,
    double north_m, double east_m, double altitude_agl_m)
{
    const double origin_lat_rad = degToRad(origin_lat_deg);
    const double d_lat = north_m / EARTH_RADIUS_M;
    const double d_lon = east_m / (EARTH_RADIUS_M * std::cos(origin_lat_rad));

    return {
        origin_lat_deg + radToDeg(d_lat),
        origin_lon_deg + radToDeg(d_lon),
        altitude_agl_m
    };
}

Waypoint globalToNed(
    const GlobalWaypoint & wp,
    double origin_lat_deg, double origin_lon_deg)
{
    const double origin_lat_rad = degToRad(origin_lat_deg);
    const double d_lat = degToRad(wp.lat_deg - origin_lat_deg);
    const double d_lon = degToRad(wp.lon_deg - origin_lon_deg);

    return {
        d_lat * EARTH_RADIUS_M,
        d_lon * EARTH_RADIUS_M * std::cos(origin_lat_rad),
        -wp.altitude_agl_m
    };
}

std::vector<GlobalWaypoint> defaultGlobalWaypoints(
    double origin_lat_deg, double origin_lon_deg)
{
    constexpr double ALTITUDE_AGL_M = 1.0;

    return {
        nedOffsetToGlobal(origin_lat_deg, origin_lon_deg, 1.0, 0.0, ALTITUDE_AGL_M),
        nedOffsetToGlobal(origin_lat_deg, origin_lon_deg, 1.0, 1.0, ALTITUDE_AGL_M),
        nedOffsetToGlobal(origin_lat_deg, origin_lon_deg, 0.0, 1.0, ALTITUDE_AGL_M),
        nedOffsetToGlobal(origin_lat_deg, origin_lon_deg, 0.0, 0.0, ALTITUDE_AGL_M),
    };
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

    waypoints_ = std::make_unique<WaypointHandler>(std::vector<Waypoint>{});
    control_   = std::make_unique<ControlModule>(this);

    // Pasang callback supaya MissionManager dapat update posisi/status
    // tanpa ControlModule perlu tahu apa pun soal logic misi.
    control_->setPositionCallback(
        [this](const ControlModule::PositionSample & s) {
            onPositionUpdate(s);
        });

    control_->setGlobalPositionCallback(
        [this](const ControlModule::GlobalPositionSample & s) {
            onGlobalPositionUpdate(s);
        });

    control_->setRangeCallback(
        [this](const ControlModule::RangeSample & s) {
            onRangeUpdate(s);
        });

    control_->setStatusCallback(
        [this](uint8_t arming_state) {
            onStatusUpdate(arming_state);
        });

    timer_ = this->create_wall_timer(100ms, [this]() { loop(); });

    RCLCPP_INFO(this->get_logger(), "=== NODE SIAP ===");
    RCLCPP_INFO(this->get_logger(),
        "Altitude command bias: %.2fm", altitude_command_bias_m_);
    RCLCPP_INFO(this->get_logger(),
        "Waypoint global LLA akan dikonversi ke NED setelah GPS origin valid.");
}

// ==================================================================
// Callback dari ControlModule
// ==================================================================

namespace
{
// Toleransi & durasi stabilisasi origin — lihat komentar di header.
constexpr double ORIGIN_STABLE_TOL_M = 0.20;   // meter
constexpr double ORIGIN_STABLE_DUR_S = 1.0;    // detik
}  // namespace

void MissionManager::onPositionUpdate(const ControlModule::PositionSample & s)
{
    const double x = s.x, y = s.y, z = s.z, yaw = s.yaw;

    last_position_time_ = this->now();

    if (!got_origin_) {
        const auto now = this->now();

        if (!has_candidate_) {
            origin_candidate_ = {x, y, z};
            candidate_since_  = now;
            has_candidate_    = true;
        } else {
            const double dn = x - origin_candidate_.north;
            const double de = y - origin_candidate_.east;
            const double dd = z - origin_candidate_.down;

            if (std::abs(dn) > ORIGIN_STABLE_TOL_M ||
                std::abs(de) > ORIGIN_STABLE_TOL_M ||
                std::abs(dd) > ORIGIN_STABLE_TOL_M)
            {
                // Estimasi masih bergeser (EKF belum konvergen) —
                // reset jendela stabilisasi, jangan kunci origin dulu.
                origin_candidate_ = {x, y, z};
                candidate_since_  = now;
            } else if ((now - candidate_since_).seconds() >= ORIGIN_STABLE_DUR_S) {
                origin_north_ = x;
                origin_east_  = y;
                origin_down_  = z;
                got_origin_   = true;

                // Baseline reset counter mulai dari titik ini — reset
                // SEBELUM origin terkunci tidak relevan (origin toh
                // masih bergerak-gerak saat itu).
                last_xy_reset_counter_      = s.xy_reset_counter;
                last_z_reset_counter_       = s.z_reset_counter;
                last_heading_reset_counter_ = s.heading_reset_counter;
                have_reset_counters_        = true;

                RCLCPP_INFO(this->get_logger(),
                    "PX4 local origin captured (stabil %.1fs, toleransi %.2fm): "
                    "x=%.2f y=%.2f z=%.2f",
                    ORIGIN_STABLE_DUR_S, ORIGIN_STABLE_TOL_M,
                    origin_north_, origin_east_, origin_down_);
                RCLCPP_INFO(this->get_logger(),
                    "Mission frame is now relative to this start position.");
            }
        }

        // Selama origin belum terkunci, jangan hitung posisi relatif —
        // biarkan loop() tetap di jalur "Menunggu data posisi dari PX4..."
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

    vehicle_.yaw = yaw;

    vehicle_.got_position = true;
}

void MissionManager::onGlobalPositionUpdate(
    const ControlModule::GlobalPositionSample & s)
{
    if (!s.lat_lon_valid) {
        return;
    }

    if (!got_global_origin_) {
        origin_lat_deg_ = s.lat_deg;
        origin_lon_deg_ = s.lon_deg;
        origin_alt_m_ = s.alt_m;
        got_global_origin_ = true;

        RCLCPP_INFO(this->get_logger(),
            "GPS origin captured: lat=%.8f lon=%.8f alt=%.2fm%s",
            origin_lat_deg_, origin_lon_deg_, origin_alt_m_,
            s.alt_valid ? "" : " (alt belum valid, hanya lat/lon dipakai)");
    }
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

    if (!got_global_origin_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
            1000, "Menunggu GPS global origin untuk konversi waypoint LLA -> NED...");
        return false;
    }

    std::vector<Waypoint> ned_waypoints;
    const auto global_waypoints = defaultGlobalWaypoints(origin_lat_deg_, origin_lon_deg_);
    ned_waypoints.reserve(global_waypoints.size());

    RCLCPP_INFO(this->get_logger(), "=== WAYPOINT GLOBAL LLA -> NED PX4 ===");
    for (size_t i = 0; i < global_waypoints.size(); ++i) {
        const auto ned = globalToNed(
            global_waypoints[i], origin_lat_deg_, origin_lon_deg_);
        ned_waypoints.push_back(ned);

        RCLCPP_INFO(this->get_logger(),
            "WP%zu LLA=(%.8f, %.8f, AGL %.1fm) -> N=%.2fm E=%.2fm Alt=%.1fm",
            i + 1,
            global_waypoints[i].lat_deg,
            global_waypoints[i].lon_deg,
            global_waypoints[i].altitude_agl_m,
            ned.n, ned.e, -ned.d);
    }

    waypoints_ = std::make_unique<WaypointHandler>(std::move(ned_waypoints));
    mission_waypoints_ready_ = true;
    return true;
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
    if (got_lidar_altitude_) {
        return -lidar_altitude_m_;
    }
    return vehicle_.position.down;
}

double MissionManager::currentAltitudeAgl() const
{
    return -currentAltitudeDown();
}

double MissionManager::toPx4DownForAltitudeTarget(double mission_down) const
{
    if (got_lidar_altitude_) {
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

bool MissionManager::isPositionStale(double threshold_s) const
{
    return (this->now() - last_position_time_).seconds() > threshold_s;
}

bool MissionManager::isLidarStale(double threshold_s) const
{
    return (this->now() - last_range_time_).seconds() > threshold_s;
}

// ==================================================================
// Loop utama — dipanggil 10Hz
// ==================================================================

void MissionManager::loop()
{
    if (!vehicle_.got_position) {
        control_->publishHeartbeat(true);
        control_->sendPositionSetpoint(0.0, 0.0, -1.5, 0.0);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
            1000, "Menunggu data posisi dari PX4...");
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

    if (!got_lidar_altitude_ || isLidarStale(0.5)) {
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
        case Phase::MISSION:     runMission();      break;
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

    const auto & wp0 = waypoints_->at(0);
    const auto yaw_result = waypoints_->computeYaw(
        vehicle_.position.north, vehicle_.position.east, vehicle_.yaw, wp0.n, wp0.e);

    control_->sendPositionSetpoint(
        toPx4North(vehicle_.position.north),
        toPx4East(vehicle_.position.east),
        origin_down_ + vehicle_.position.down,
        yaw_result.target_yaw);

    ++counter_;
    RCLCPP_INFO(this->get_logger(), "INIT... (%d/5)", counter_);

    if (counter_ >= 5) {
        control_->setOffboardMode();
        control_->arm();
        phase_   = Phase::WAIT_ARM;
        counter_ = 0;
        RCLCPP_INFO(this->get_logger(), "Menunggu konfirmasi ARM dari PX4...");
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

    const auto & wp0 = waypoints_->at(0);
    const auto yaw_result = waypoints_->computeYaw(
        vehicle_.position.north, vehicle_.position.east, vehicle_.yaw, wp0.n, wp0.e);

    control_->sendPositionSetpoint(
        toPx4North(vehicle_.position.north),
        toPx4East(vehicle_.position.east),
        origin_down_ + vehicle_.position.down,   // tahan posisi sekarang, jangan naik dulu
        yaw_result.target_yaw);

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
        RCLCPP_INFO(this->get_logger(), "ARM terkonfirmasi. === TAKEOFF ===");
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
        RCLCPP_INFO(this->get_logger(), "=== TAKEOFF ===");
    }
}

// ==================================================================
// TAKEOFF — naik ke altitude waypoint pertama
// ==================================================================

void MissionManager::runTakeoff()
{
    control_->publishHeartbeat(true);

    const auto & wp0 = waypoints_->at(0);
    const auto yaw_result = waypoints_->computeYaw(
        vehicle_.position.north, vehicle_.position.east, vehicle_.yaw, wp0.n, wp0.e);

    control_->sendPositionSetpoint(
        toPx4North(vehicle_.position.north),
        toPx4East(vehicle_.position.east),
        toPx4DownForAltitudeTarget(takeoffTargetDown()),
        yaw_result.target_yaw);

    RCLCPP_INFO(this->get_logger(),
        "Altitude lidar: %.2fm / %.2fm | raw z: %.2f -> %.2f",
        currentAltitudeAgl(), -takeoffTargetDown(),
        origin_down_ + vehicle_.position.down,
        toPx4DownForAltitudeTarget(takeoffTargetDown()));

    if (currentAltitudeDown() <= takeoffTargetDown() + 0.15) {
        vehicle_.hover_position.north = vehicle_.position.north;
        vehicle_.hover_position.east = vehicle_.position.east;
        vehicle_.hover_position.down  = currentAltitudeDown();
        RCLCPP_INFO(this->get_logger(), "Takeoff complete!");
        phase_ = Phase::HOVER;
    }
}

// ==================================================================
// HOVER — stabilisasi 3 detik (30 tick @ 10Hz)
// ==================================================================

void MissionManager::runHover()
{
    control_->publishHeartbeat(true);

    const auto & wp0 = waypoints_->at(0);
    const auto yaw_result = waypoints_->computeYaw(
        vehicle_.position.north, vehicle_.position.east, vehicle_.yaw, wp0.n, wp0.e);

    control_->sendPositionSetpoint(
        toPx4North(vehicle_.hover_position.north),
        toPx4East(vehicle_.hover_position.east),
        toPx4DownForAltitudeTarget(takeoffTargetDown()),
        yaw_result.target_yaw);

    ++hover_counter_;
    RCLCPP_INFO(this->get_logger(), "Hover... %d/30", hover_counter_);

    if (hover_counter_ >= 30) {
        hover_counter_ = 0;
        phase_ = Phase::MISSION;
        RCLCPP_INFO(this->get_logger(), "=== START MISSION ===");
    }
}

// ==================================================================
// MISSION — kunjungi semua waypoint berurutan
// ==================================================================

void MissionManager::runMission()
{
    control_->publishHeartbeat(false);

    if (current_wp_ >= waypoints_->size()) {
        RCLCPP_INFO(this->get_logger(), "=== ALL WAYPOINT REACHED ===");
        phase_ = Phase::LAND_CMD;
        return;
    }

    const auto & wp = waypoints_->at(current_wp_);
    const std::string label = waypoints_->labelAt(current_wp_);

    const double err_n = wp.n - vehicle_.position.north;
    const double err_e = wp.e - vehicle_.position.east;
    const double dist  = WaypointHandler::horizontalDistance(
        vehicle_.position.north, vehicle_.position.east, wp.n, wp.e);

    const double alt_error   = WaypointHandler::altitudeError(currentAltitudeDown(), wp.d);
    const bool   alt_reached = waypoints_->isAltitudeReached(alt_error);

    // vz selalu dihitung — tidak boleh diblok oleh fase memutar yaw
    const double vz = waypoints_->computeVerticalVelocity(alt_error);

    const auto yaw_result = waypoints_->computeYaw(
        vehicle_.position.north, vehicle_.position.east, vehicle_.yaw, wp.n, wp.e);

    const bool is_pure_alt = waypoints_->isPureAltitude(dist);

    RCLCPP_INFO(this->get_logger(),
        "[%s] N=%.1f E=%.1f | Dist:%.2fm | Alt:%.2f->%.1fm%s",
        label.c_str(),
        vehicle_.position.north, vehicle_.position.east,
        dist,
        currentAltitudeAgl(), -wp.d,
        is_pure_alt ? " [CLIMB]" : "");

    // ── Sudah sampai? ──────────────────────────────────────────────
    if (waypoints_->isWaypointReached(dist, alt_reached)) {
        ++wp_hold_counter_;
        control_->sendVelocitySetpoint(0.0, 0.0, 0.0, yaw_result.target_yaw);

        if (wp_hold_counter_ >= 3) {
            RCLCPP_INFO(this->get_logger(), "[%s] REACHED!", label.c_str());
            RCLCPP_INFO(this->get_logger(),
                "REACHED POSITION: N=%.2f E=%.2f", vehicle_.position.north, vehicle_.position.east);
            ++current_wp_;
            wp_hold_counter_ = 0;
        }
        return;
    } else {
        wp_hold_counter_ = 0;
    }

    // ── Pure altitude WP — langsung climb, skip fase putar yaw ───────
    if (is_pure_alt) {
        control_->sendVelocitySetpoint(0.0, 0.0, vz, yaw_result.target_yaw);
        return;
    }

    // ── Maju ke target, kecepatan diskala smooth sesuai heading ───────
    // FIX (bukti lapangan): gate biner lama (facing ? full-speed :
    // stop-total-lalu-putar) menyebabkan drone orbit/loiter tanpa
    // konvergen di waypoint yang butuh belok besar (WP2, ~90°) — bearing
    // ke target bergeser tiap drone bergerak sedikit, yaw_error nyebrang
    // threshold, drone berhenti mendadak, siklus berulang. Sekarang
    // kecepatan maju diskala kontinu dengan yawAlignmentFactor(): makin
    // pas heading-nya makin cepat, makin meleset makin pelan (tapi tetap
    // ada progres), sambil tetap berputar menuju target_yaw. Tidak ada
    // lagi transisi on/off mendadak.
    double vx, vy;
    waypoints_->computeApproachVelocity(err_n, err_e, dist, vx, vy);
    const double align = waypoints_->yawAlignmentFactor(yaw_result.yaw_error);
    vx *= align;
    vy *= align;
    control_->sendVelocitySetpoint(vx, vy, vz, yaw_result.target_yaw);
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

