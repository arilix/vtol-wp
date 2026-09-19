#include <rclcpp/rclcpp.hpp>
#include "utils/mission_manager.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>

namespace
{
volatile std::sig_atomic_t stop_requested = 0;

void handleSignal(int)
{
    stop_requested = 1;
}
}  // namespace

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv, rclcpp::InitOptions(),
        rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    auto node = std::make_shared<px4::MissionManager>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    while (rclcpp::ok() && !stop_requested) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (stop_requested && rclcpp::ok()) {
        RCLCPP_WARN(node->get_logger(),
            "Ctrl+C diterima: kirim LAND berulang selama 1 detik sebelum shutdown.");
        for (int i = 0; i < 10; ++i) {
            node->requestGracefulLand();
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    rclcpp::shutdown();
    return 0;
}
