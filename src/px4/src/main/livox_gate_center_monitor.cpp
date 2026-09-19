#include "utils/gate_centering_lock.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <array>
#include <vector>

namespace px4
{

// Node diagnostik darat: hanya subscribe PointCloud2. Tidak membuat
// publisher PX4 dan tidak memiliki jalur untuk arm, takeoff, atau setpoint.
class LivoxGateCenterMonitor final : public rclcpp::Node
{
public:
    LivoxGateCenterMonitor()
    : Node("livox_gate_center_monitor"), lock_(readConfig())
    {
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/points", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                onPoints(std::move(msg));
            });
        RCLCPP_INFO(get_logger(),
            "LIVOX DIAGNOSTIC aktif: hanya membaca /livox/points; tidak ada perintah flight.");
    }

private:
    GateCenteringLock::Config readConfig()
    {
        GateCenteringLock::Config cfg;
        cfg.roi_forward_min_m = declare_parameter<float>("detection_range_min", 1.0F);
        cfg.roi_forward_max_m = declare_parameter<float>("detection_range_max", 10.0F);
        cfg.roi_lateral_m = declare_parameter<float>("roi_lateral", 3.0F);
        cfg.gate_width_m = declare_parameter<float>("gate_width", 1.5F);
        cfg.centering_tolerance_m = declare_parameter<float>("centering_tolerance", 0.2F);
        cfg.min_cluster_points = declare_parameter<int>("min_cluster_points", 5);
        return cfg;
    }

    void onPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
    {
        if (msg->width * msg->height == 0U) return;

        std::vector<std::array<float, 3>> points;
        points.reserve(msg->width * msg->height);
        try {
            sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
            for (; x != x.end(); ++x, ++y, ++z) points.push_back({*x, *y, *z});
        } catch (const std::runtime_error & e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                "Format PointCloud2 tidak memiliki field x/y/z: %s", e.what());
            return;
        }

        const auto result = lock_.update(points);
        // Harus dua tiang dan lebar valid, sama seperti syarat misi sebelum maju.
        const bool fully_centered = result.valid && result.width_valid &&
            !result.one_side_only && result.centered;
        if (fully_centered) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                "[LIVOX CENTER] CENTER | error=%.2f m | jarak=%.2f m | lebar=%.2f m",
                result.lateral_error_m, result.forward_distance_m, result.detected_width_m);
        } else if (result.valid) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "[LIVOX CENTER] BELUM CENTER | error=%.2f m | jarak=%.2f m | lebar=%.2f m | %s",
                result.lateral_error_m, result.forward_distance_m, result.detected_width_m,
                result.one_side_only ? "hanya satu tiang" : "lebar/toleransi belum cocok");
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "[LIVOX CENTER] GATE BELUM TERDETEKSI");
        }
    }

    GateCenteringLock lock_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

}  // namespace px4

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4::LivoxGateCenterMonitor>());
    rclcpp::shutdown();
    return 0;
}
