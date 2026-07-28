#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <array>

#include <functional>
#include <vector>

namespace px4
{

// ==================================================================
// ControlModule
//
// Satu-satunya kelas yang menyentuh publisher/subscriber ke PX4.
// Tugasnya murni I/O: kirim command, kirim setpoint, terima posisi
// dan status drone. Tidak ada logic keputusan misi di sini —
// itu tanggung jawab MissionManager.
//
// Dipakai sebagai komponen di dalam MissionManager (composition,
// bukan inheritance) supaya MissionManager tetap fokus ke state
// machine saja.
// ==================================================================

class ControlModule
{
public:
    // node: pointer ke node ROS2 milik MissionManager — dipakai untuk
    // membuat publisher/subscriber dan logging, BUKAN untuk spin sendiri.
    explicit ControlModule(rclcpp::Node * node, bool enable_livox);

    // ── Sample posisi + info reset EKF ────────────────────────────
    // xy_reset_counter/z_reset_counter/heading_reset_counter berubah
    // setiap kali EKF2 PX4 melakukan reset internal (lompatan estimasi
    // akibat re-fusi GPS/baro/dst). delta_x/delta_y/delta_z/delta_heading
    // adalah besar lompatan itu. Tanpa info ini, konsumen posisi lokal
    // (seperti MissionManager) akan melihat lompatan tersebut sebagai
    // "pergerakan nyata" dan mengejarnya — persis penyebab drone drift
    // beberapa meter yang terekam di lapangan.
    struct PositionSample
    {
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double vx{0.0};
        double vy{0.0};
        double vz{0.0};
        double yaw{0.0};

        uint8_t xy_reset_counter{0};
        double  delta_x{0.0};
        double  delta_y{0.0};

        uint8_t z_reset_counter{0};
        double  delta_z{0.0};

        uint8_t heading_reset_counter{0};
        double  delta_heading{0.0};
    };

    struct RangeSample
    {
        double range_m{0.0};
        double min_range_m{0.0};
        double max_range_m{0.0};
        bool valid{false};
    };

    // Offset marker ArUco (frame kamera, meter) dari /fiducial/pose.
    // x = kanan-gambar, y = bawah-gambar (konvensi OpenCV optical frame).
    // Konversi image->body (mount yaw) sengaja TIDAK dilakukan di sini —
    // ControlModule cuma ekstrak field mentah, matematika ada di
    // MissionManager/VisionLock (batas ROS<->math murni sama seperti
    // pola yang sudah ada di file ini).
    struct MarkerPoseSample
    {
        double x{0.0};
        double y{0.0};
    };

    struct MarkerCenter
    {
        int id{-1};
        double x_px{0.0};
        double y_px{0.0};
    };

    struct MarkerCentersSample
    {
        double frame_width_px{0.0};
        double frame_height_px{0.0};
        std::vector<MarkerCenter> markers;
    };

    // Titik tengah box terbaik dari /general_box/target_center
    // (yolo_camera_node, package general_box_detector_ros). Beda dari
    // MarkerPoseSample: ini piksel mentah, BUKAN meter — box tidak
    // punya ukuran fisik pasti seperti marker ArUco, jadi tidak bisa
    // solvePnP. Konversi piksel->meter (ground-plane projection, butuh
    // altitude) dilakukan di GroundLock, bukan di sini.
    struct TargetCenterSample
    {
        double frame_width_px{0.0};
        double frame_height_px{0.0};
        double cx_px{0.0};
        double cy_px{0.0};
        double confidence{0.0};
        // false = frame ini tidak ada box terdeteksi, ATAU lebih dari satu
        // box terdeteksi (yolo_camera_node sengaja menolak frame ambigu —
        // box tidak punya ID unik seperti marker ArUco, jadi kalau lebih
        // dari satu box kelihatan tidak ada cara aman menentukan mana yang
        // benar tanpa risiko flicker target antar-frame).
        bool valid{false};
    };

    // Titik body-frame Livox MID360s dari /livox/points (PointCloud2),
    // sudah diekstrak x/y/z mentah (x=forward, y=lateral, z=height,
    // sama seperti frame sensor asli) — matematika ROI/binning ada di
    // GateCenteringLock (utils/gate_centering_lock.h), bukan di sini,
    // sama seperti pola MarkerPoseSample/TargetCenterSample: ControlModule
    // cuma ekstrak field mentah dari message.
    struct LivoxSample
    {
        std::vector<std::array<float, 3>> points;
    };

    // ── Callback registration ─────────────────────────────────────
    // MissionManager pasang callback ini untuk menerima update posisi
    // dan status drone tanpa ControlModule perlu tahu logic misi.
    using PositionCallback = std::function<void(const PositionSample &)>;
    using RangeCallback = std::function<void(const RangeSample &)>;
    using StatusCallback = std::function<void(uint8_t arming_state)>;
    using MarkerPoseCallback = std::function<void(const MarkerPoseSample &)>;
    using MarkerCentersCallback = std::function<void(const MarkerCentersSample &)>;
    using TargetCenterCallback = std::function<void(const TargetCenterSample &)>;
    using LivoxCallback = std::function<void(const LivoxSample &)>;

    void setPositionCallback(PositionCallback cb);
    void setRangeCallback(RangeCallback cb);
    void setStatusCallback(StatusCallback cb);
    void setMarkerPoseCallback(MarkerPoseCallback cb);
    void setMarkerCentersCallback(MarkerCentersCallback cb);
    void setTargetCenterCallback(TargetCenterCallback cb);
    void setLivoxCallback(LivoxCallback cb);

    // ── Command ke PX4 ─────────────────────────────────────────────
    void arm();
    void setOffboardMode();
    void sendLandCommand();

    // ── Heartbeat (wajib dikirim tiap tick selama offboard aktif) ───
    void publishHeartbeat(bool use_position);

    // ── Setpoint ───────────────────────────────────────────────────
    void sendPositionSetpoint(double x, double y, double z, double yaw);
    void sendVelocitySetpoint(double vx, double vy, double vz, double yaw);

private:
    uint64_t nowUs() const;

    void onPosition(
        const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void onRange(
        const sensor_msgs::msg::Range::SharedPtr msg);
    void onStatus(
        const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void onMarkerPose(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void onMarkerCenters(
        const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void onTargetCenter(
        const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void onLivox(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Node * node_;   // non-owning, milik MissionManager

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  traj_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr      cmd_pub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr        status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    marker_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr    marker_centers_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr    target_center_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr       livox_sub_;

    PositionCallback   position_cb_;
    RangeCallback      range_cb_;
    StatusCallback     status_cb_;
    MarkerPoseCallback marker_pose_cb_;
    MarkerCentersCallback marker_centers_cb_;
    TargetCenterCallback  target_center_cb_;
    LivoxCallback         livox_cb_;
};

}  // namespace px4
