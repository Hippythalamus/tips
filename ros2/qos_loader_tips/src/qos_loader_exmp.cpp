#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_options.hpp>
#include  <rclcpp/executors.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "qos_loader_tips/qos_loader.hpp"
#include "qos_loader_tips/execution_loader.hpp"

class TestNode : public rclcpp::Node
{
public:
    TestNode() : Node("test_node")
    {
        RCLCPP_INFO(this->get_logger(), "Node created");

        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        rclcpp::SubscriptionOptions opts;
        opts.callback_group = cb_group_;

        std::shared_ptr<QosLoader> loader_  = std::make_shared<QosLoader>("config/qos.yaml"); //TODO to parameters


        rclcpp::QoS qos = loader_->qos("camera_default"); //TODO to parameters


        RCLCPP_INFO(this->get_logger(), "Loaded QoS from YAML");

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", //TODO to parameters
            qos,
            std::bind(&TestNode::callback, this, std::placeholders::_1),
            opts
        );
    }

    rclcpp::CallbackGroup::SharedPtr getCallbackGroup() const { return cb_group_; }

private:
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Got image %dx%d", msg->width, msg->height);
    }

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TestNode>();

    //TODO EXECUTER parameters from yaml soon

    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    
    executor->add_callback_group(
        node->getCallbackGroup(),
        node->get_node_base_interface()
    );

    std::thread spin_thread([executor]() {
        executor->spin();
    });

    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}