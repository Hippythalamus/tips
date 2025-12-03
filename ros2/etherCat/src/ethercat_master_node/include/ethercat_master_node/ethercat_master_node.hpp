#ifndef ETHERCAT_MASTER_NODE_HPP
#define ETHERCAT_MASTER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <soem/soem.h>

#define DEFAULT_QUEUE_SZ 10
#define DEFAULT_PERIOD 1
#define DEFAULT_IOMAP_BUF 4096



class EthercatMasterNode : public rclcpp::Node
{
public: 
    EthercatMasterNode();
    ~EthercatMasterNode();

private:
    void initSOEM();

    void cyclicTask();
    void cmdCallback(const std::shared_ptr<const std_msgs::msg::Float64> & msg);

    static ecx_contextt ctx_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr status_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr cyclic_timer_;
};


#endif 