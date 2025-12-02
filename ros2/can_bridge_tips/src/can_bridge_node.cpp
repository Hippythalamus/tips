#include "can_bridge_tips/can_bridge_node.hpp"

#include <sys/ioctl.h>
#include <net/if.h>
#include <poll.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>


CanBridgeNode::CanBridgeNode()
    : Node("can_bridge_vcan")
{
    // parameters 
    this->declare_parameter<std::string>("can_interface", DEFAULT_CAN_IF);
    this->declare_parameter<int>("queue_size", DEFAULT_CAN_QUEUE);
    this->declare_parameter<int>("can_id", DEFAULT_CAN_ID);


    interface_name_ = this->get_parameter("can_interface").as_string();
    queue_size_     = this->get_parameter("queue_size").as_int();
    can_id_         = this->get_parameter("can_id").as_int();

    RCLCPP_INFO(
        this->get_logger(),
        "CAN bridge starting with if='%s', q_sze=%d",
        interface_name_.c_str(),
        queue_size_
    );

    using namespace std::chrono_literals;

        timer_ = this->create_wall_timer(
            1ms,                            // period should be in config but I do not have time                         
            std::bind(&CanBridgeNode::canReadOnce, this) // 
        );
        /*
        I just got an idea. 
        I could group callbacks together and assign them specific parameters 
         such as execution period, purpose, and description 
          and move all that into a transport-layer configuration. 
          I think this could be an interesting architectural solution.
        */

    auto qos = rclcpp::QoS(rclcpp::KeepLast(queue_size_)); 
    qos.best_effort(); //msg can be lost
    qos.durability_volatile(); //duration. msgs will not be stored for new subscriber

    pub_ = this->create_publisher<can_msgs::msg::Frame>("can_rx", qos);

    // Setup socket
    if (!setupSocket(can_id_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to setup CAN socket");
        throw std::runtime_error("CAN socket setup failed");
    }


    RCLCPP_INFO(this->get_logger(), "CAN bridge started OK");
}

CanBridgeNode::~CanBridgeNode()
{
    stop();
}

void CanBridgeNode::stop()
{

    RCLCPP_INFO(this->get_logger(), "Stopping CAN bridge…");


    // Close socket to unblock read()
    if (sock_ >= 0) {
        shutdown(sock_, SHUT_RDWR);
        close(sock_);
        sock_ = -1;
    }

    RCLCPP_INFO(this->get_logger(), "CAN bridge stopped");
}

bool CanBridgeNode::setupSocket(int id)
{

    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "socket() failed: %s", strerror(errno));
        return false;
    }

    struct can_filter filter;
    filter.can_id   = id;
    filter.can_mask = CAN_SFF_MASK;

    if (setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER,
                   &filter, sizeof(filter)) < 0)
    {
        RCLCPP_WARN(this->get_logger(), 
                    "setsockopt(FILTER) failed: %s", strerror(errno));
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);

    // Resolve interface index
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "ioctl(SIOCGIFINDEX) failed: %s", strerror(errno));
        return false;
    }

    // Prepare bind
    struct sockaddr_can addr {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "bind() failed: %s", strerror(errno));
        return false;
    }

    return true;
}

void CanBridgeNode::canReadOnce(){

    struct pollfd pfd;
    pfd.fd = sock_;
    pfd.events = POLLIN;

    struct can_frame frame;

        int ret = poll(&pfd, 1, 0);  


        if (ret < 0) {
            if (errno == EINTR)   RCLCPP_WARN(this->get_logger(), "poll() error: %s", strerror(errno));
            return;
        }

        if (ret == 0) {
            return;
        }

        if (pfd.revents & POLLIN) {
            int nbytes = recv(sock_, &frame, sizeof(frame), 0);

            if (nbytes < 0) {
                if (errno == EWOULDBLOCK || errno == EAGAIN) return;;
                RCLCPP_WARN(this->get_logger(), "recv() error: %s", strerror(errno));
                return;
            }
        // Fill ROS message
        if (nbytes == sizeof(struct can_frame)) {
                can_msgs::msg::Frame msg;
                msg.id         = frame.can_id;
                msg.dlc        = frame.can_dlc;
                msg.is_extended = (frame.can_id & CAN_EFF_FLAG) != 0;
                msg.is_rtr      = (frame.can_id & CAN_RTR_FLAG) != 0;
                msg.is_error    = (frame.can_id & CAN_ERR_FLAG) != 0;

                std::copy(frame.data, frame.data + frame.can_dlc, msg.data.begin());

                for(int i = 0; i< 4; i++){
                    RCLCPP_INFO(this->get_logger(), "CAN msg [%i] : %02X", i, msg.data[i]); //My personal check
                }

                pub_->publish(msg);
            }
        }
    
    RCLCPP_INFO(this->get_logger(), "CAN read loop exited");
}
