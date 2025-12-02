#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <can_msgs/msg/frame.hpp>

#include <atomic>
#include <string>


#define DEFAULT_CAN_IF "can0"
#define DEFAULT_CAN_QUEUE 10
#define DEFAULT_CAN_ID 0x123


class CanBridgeNode : public rclcpp::Node
{
public:
    CanBridgeNode();
    ~CanBridgeNode();

private:
    bool setupSocket(int id);     // Create CAN socket
    void canReadOnce();         // Reading 
    void stop();            // close socket)

    std::string interface_name_ = DEFAULT_CAN_IF;
    int queue_size_ = DEFAULT_CAN_QUEUE;
    int can_id_ = DEFAULT_CAN_ID;

    int sock_ = -1;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_;
};
