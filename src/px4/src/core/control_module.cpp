#include "utils/control_module.h"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <rclcpp/qos.hpp>
#include <cmath>
#include <limits>

namespace px4
{

ControlModule::ControlModule(rclcpp::Node * node, bool enable_livox)
: node_(node)
{
    // QoS PX4 uXRCE-DDS yang terlihat di graph: BEST_EFFORT + TRANSIENT_LOCAL.
    // Samakan subscriber dengan publisher agar tidak ada mismatch durability.
    rclcpp::QoS px4_qos(rclcpp::KeepLast(1));
    px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

    // ── Publisher ──────────────────────────────────────────────────
    offboard_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);

    traj_pub_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);

    cmd_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);

    // ── Subscriber ─────────────────────────────────────────────────
    pos_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position",
        px4_qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
            onPosition(msg);
        });

    range_sub_ = node_->create_subscription<sensor_msgs::msg::Range>(
        "/range",
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Range::SharedPtr msg) {
            onRange(msg);
        });

    status_sub_ = node_->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1",
        px4_qos,
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
            onStatus(msg);
        });

    // /fiducial/pose dipublish dengan create_publisher<PoseStamped>(topic, 10)
    // -> Reliable + Volatile (BUKAN px4_qos yang BestEffort+TransientLocal
    // untuk topik uXRCE-DDS PX4). Samakan durability dengan publisher asli.
    marker_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/fiducial/pose",
        rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            onMarkerPose(msg);
        });

    marker_centers_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/fiducial/marker_centers",
        rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            onMarkerCenters(msg);
        });

    // /general_box/target_center dipublish yolo_camera_node (package
    // general_box_detector_ros) dengan create_publisher default (Reliable
    // + Volatile) — sama seperti /fiducial/marker_centers.
    target_center_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/general_box/target_center",
        rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            onTargetCenter(msg);
        });

    // Isolasi keras Livox: saat fitur gate mati, jangan membuat subscriber
    // sama sekali. PointCloud2 berukuran besar tidak boleh ikut antre di
    // executor mission_manager dan mengganggu heartbeat/setpoint 10 Hz.
    if (enable_livox) {
        livox_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/points",
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                onLivox(msg);
            });
    }
}

uint64_t ControlModule::nowUs() const
{
    return static_cast<uint64_t>(node_->get_clock()->now().nanoseconds() / 1000);
}

void ControlModule::setPositionCallback(PositionCallback cb)
{
    position_cb_ = std::move(cb);
}

void ControlModule::setRangeCallback(RangeCallback cb)
{
    range_cb_ = std::move(cb);
}

void ControlModule::setStatusCallback(StatusCallback cb)
{
    status_cb_ = std::move(cb);
}

void ControlModule::setMarkerPoseCallback(MarkerPoseCallback cb)
{
    marker_pose_cb_ = std::move(cb);
}

void ControlModule::setMarkerCentersCallback(MarkerCentersCallback cb)
{
    marker_centers_cb_ = std::move(cb);
}

void ControlModule::setTargetCenterCallback(TargetCenterCallback cb)
{
    target_center_cb_ = std::move(cb);
}

void ControlModule::setLivoxCallback(LivoxCallback cb)
{
    livox_cb_ = std::move(cb);
}

void ControlModule::onPosition(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    if (position_cb_) {
        PositionSample s;
        s.x = msg->x;
        s.y = msg->y;
        s.z = msg->z;
        s.vx = msg->vx;
        s.vy = msg->vy;
        s.vz = msg->vz;
        s.yaw = msg->heading;

        s.xy_reset_counter = msg->xy_reset_counter;
        s.delta_x = msg->delta_xy[0];
        s.delta_y = msg->delta_xy[1];

        s.z_reset_counter = msg->z_reset_counter;
        s.delta_z = msg->delta_z;

        s.heading_reset_counter = msg->heading_reset_counter;
        s.delta_heading = msg->delta_heading;

        position_cb_(s);
    }
}

void ControlModule::onRange(const sensor_msgs::msg::Range::SharedPtr msg)
{
    if (range_cb_) {
        RangeSample s;
        s.range_m = msg->range;
        s.min_range_m = msg->min_range;
        s.max_range_m = msg->max_range;
        s.valid =
            std::isfinite(s.range_m) &&
            s.range_m >= s.min_range_m &&
            s.range_m <= s.max_range_m;

        range_cb_(s);
    }
}

void ControlModule::onStatus(
    const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    if (status_cb_) {
        status_cb_(msg->arming_state);
    }
}

void ControlModule::onMarkerPose(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (marker_pose_cb_) {
        MarkerPoseSample s;
        s.x = msg->pose.position.x;
        s.y = msg->pose.position.y;
        marker_pose_cb_(s);
    }
}

void ControlModule::onMarkerCenters(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (!marker_centers_cb_ || msg->data.size() < 2) {
        return;
    }

    MarkerCentersSample s;
    s.frame_width_px = msg->data[0];
    s.frame_height_px = msg->data[1];
    for (size_t i = 2; i + 2 < msg->data.size(); i += 3) {
        MarkerCenter marker;
        marker.id = static_cast<int>(std::lround(msg->data[i]));
        marker.x_px = msg->data[i + 1];
        marker.y_px = msg->data[i + 2];
        s.markers.push_back(marker);
    }
    marker_centers_cb_(s);
}

void ControlModule::onTargetCenter(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (!target_center_cb_ || msg->data.size() < 2) {
        return;
    }

    TargetCenterSample s;
    s.frame_width_px  = msg->data[0];
    s.frame_height_px = msg->data[1];
    // yolo_camera_node cuma menambah cx/cy/confidence kalau ada box
    // terdeteksi di frame itu (lihat FORMAT di log CENTER_TOPIC node
    // itu) — array 2 elemen berarti "tidak ada target frame ini".
    if (msg->data.size() >= 5) {
        s.cx_px       = msg->data[2];
        s.cy_px       = msg->data[3];
        s.confidence  = msg->data[4];
        s.valid       = true;
    }
    target_center_cb_(s);
}

void ControlModule::onLivox(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!livox_cb_ || msg->width * msg->height == 0) {
        return;
    }

    LivoxSample s;
    s.points.reserve(msg->width * msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        s.points.push_back({*iter_x, *iter_y, *iter_z});
    }

    livox_cb_(s);
}

void ControlModule::arm()
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = nowUs();
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    msg.param1  = 1.0f;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    cmd_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "ARM sent");
}

void ControlModule::setOffboardMode()
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = nowUs();
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    msg.param1  = 1.0f;
    msg.param2  = 6.0f;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    cmd_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "OFFBOARD mode sent");
}

void ControlModule::sendLandCommand()
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = nowUs();
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    cmd_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "LAND command sent");
}

void ControlModule::publishHeartbeat(bool use_position)
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp    = nowUs();
    msg.position     = use_position;
    msg.velocity     = !use_position;
    msg.acceleration = false;
    offboard_pub_->publish(msg);
}

void ControlModule::sendPositionSetpoint(
    double x, double y, double z, double yaw)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();

    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = nowUs();
    msg.position = {
        static_cast<float>(x),
        static_cast<float>(y),
        static_cast<float>(z)
    };
    msg.velocity = { nan, nan, nan };
    msg.acceleration = { nan, nan, nan };
    msg.jerk = { nan, nan, nan };
    msg.yaw = static_cast<float>(yaw);
    // Gunakan yaw angle sebagai satu-satunya referensi heading. Jika field
    // ini dibiarkan pada default 0.0, PX4 menerima perintah yaw-rate nol
    // bersamaan dengan target yaw dan dapat menahan rotasi pada beberapa
    // versi/controller. NaN menonaktifkan kontrol yawspeed sesuai kontrak
    // TrajectorySetpoint.
    msg.yawspeed = nan;
    traj_pub_->publish(msg);
}

void ControlModule::sendVelocitySetpoint(
    double vx, double vy, double vz, double yaw)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();

    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = nowUs();
    msg.position = { nan, nan, nan };
    msg.velocity = {
        static_cast<float>(vx),
        static_cast<float>(vy),
        static_cast<float>(vz)
    };
    msg.acceleration = { nan, nan, nan };
    msg.jerk = { nan, nan, nan };
    msg.yaw = static_cast<float>(yaw);
    msg.yawspeed = nan;
    traj_pub_->publish(msg);
}

}  // namespace px4
