#include <rclcpp/rclcpp.hpp>
#include "utils/mission_manager.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4::MissionManager>());
    rclcpp::shutdown();
    return 0;
}
