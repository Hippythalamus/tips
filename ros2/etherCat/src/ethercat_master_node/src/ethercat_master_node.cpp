#include "ethercat_master_node/ethercat_master_node.hpp"
#include <soem/soem.h>
#include <stdexcept>
#include <cstring>
#include <chrono>

// IOmap is the shared memory area between EtherCAT master and devices(slaves)
static uint8 IOmap[4096]; //TODO: size - to parameters

// holds all EtherCAT master state and configuration
ecx_contextt EthercatMasterNode::ctx_;

// Expected Working Counter (number of expected responses from devices)
static int expectedWKC;

    EthercatMasterNode::EthercatMasterNode()
        : Node("ethercat_master_node")
    {
        // send status information (from devices) to ROS
        status_pub_ = this->create_publisher<std_msgs::msg::Float64>("status", rclcpp::QoS(DEFAULT_QUEUE_SZ));
        // Create a subscriber to receive commands
        cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "command",
            rclcpp::QoS(DEFAULT_QUEUE_SZ),
            [this](const std::shared_ptr<const std_msgs::msg::Float64> msg) {
                this->cmdCallback(msg);
            }
        );
        cyclic_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(DEFAULT_PERIOD),
            std::bind(&EthercatMasterNode::cyclicTask, this)
        );
        // Initialize a master
        initSOEM();
    }

EthercatMasterNode::~EthercatMasterNode()
{
    RCLCPP_INFO(this->get_logger(), "Stopping EtherCAT Node...");
    ecx_close(&ctx_);
}
// Initialize SOEM (Simple Open EtherCAT Master)
void EthercatMasterNode::initSOEM()
{
    const char *ifname = "veth0"; // Network interface for EtherCAT  TODO: to parameters
    
    if (!ecx_init(&ctx_, ifname)) { //TODO: implement retry logic
        RCLCPP_ERROR(this->get_logger(), "SOEM: failed to initialize interface %s", ifname);
        throw std::runtime_error("SOEM init failed");
    }
    // Detect and configure connected devices
    int n_slaves = ecx_config_init(&ctx_);
    if (n_slaves == 0) throw std::runtime_error("No devices found");
    ec_groupt *group = &ctx_.grouplist[0];

    // Map process data to IOmap
    ecx_config_map_group(&ctx_, IOmap, 0);
    // Calculate expected working counter (responses from devices)
    expectedWKC = (group->outputsWKC * 2) + group->inputsWKC;
    RCLCPP_INFO(this->get_logger(),"%d devices found and configured.\n", ctx_.slavecount);

    RCLCPP_INFO(this->get_logger(),"segments : %d : %d %d %d %d\n",
                group->nsegments,
                group->IOsegment[0],
                group->IOsegment[1],
                group->IOsegment[2],
                group->IOsegment[3]);

    // Configure distributed clocks for synchronized cyclic operation
    ecx_configdc(&ctx_);
    // Set all devices to operational state
    ecx_statecheck(&ctx_, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
}

// Cyclic task: read inputs, publish to ROS, and send outputs to devices
void EthercatMasterNode::cyclicTask()
{
    //TODO Monitor expectedWKC  to detect missing responses.


    // Receive process data from devices
    ecx_receive_processdata(&ctx_, EC_TIMEOUTRET);
    // Read a 16-bit value from IOmap (device input) 
    uint16 val = *(uint16 *)(IOmap + 0);
    auto msg = std_msgs::msg::Float64();
    msg.data = static_cast<double>(val);
    status_pub_->publish(msg); // Publish to ROS
    // Send process data (outputs) to devices
    ecx_send_processdata(&ctx_);
}

// Callback for ROS subscriber -> write command to IOmap
void EthercatMasterNode::cmdCallback(const std::shared_ptr<const std_msgs::msg::Float64> & msg)
{   //TODO: add more devices (here just one)
    uint16 val = static_cast<uint16>(msg->data);
    *(uint16 *)(IOmap + 0) = val;

    /* I had no time to check it
    for (int s = 0; s < ctx_.slavecount; ++s)
        {
            ec_slave_configt *sc = &ctx_.slave[s];

            uint16 input = *(uint16*)(IOmap + sc->inputs);

            *(uint16*)(IOmap + sc->outputs) = some_value;
        }
    */
}