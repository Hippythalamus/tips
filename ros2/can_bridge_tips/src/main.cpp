#include <rclcpp/rclcpp.hpp>
#include "can_bridge_tips/can_bridge_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("startup"), "Starting CAN Bridge Node...");

    auto node = std::make_shared<CanBridgeNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
