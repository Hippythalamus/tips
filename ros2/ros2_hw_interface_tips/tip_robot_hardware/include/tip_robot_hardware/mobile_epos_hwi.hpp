#ifndef MOBILE_EPOS_HWI_HPP
#define MOBILE_EPOS_HWI_HPP

#include "hardware_interface/system_interface.hpp"
#include "tip_robot_hardware/epos4_driver.hpp"

namespace mobile_epos_hardware {

    class MobileBaseHardwareInterface : public hardware_interface::SystemInterface{
        public:
            // Lifecycle
            hardware_interface::CallbackReturn
                on_configure(const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::CallbackReturn
                on_activate(const rclcpp_lifecycle::State & previous_state) override;
            
            hardware_interface::CallbackReturn
                on_deactivate(const rclcpp_lifecycle::State & previous_state) override;


            //SystemInterface override
            hardware_interface::CallbackReturn
                on_init(const hardware_interface::HardwareInfo & info) override;
            hardware_interface::return_type
                read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            hardware_interface::return_type
                write(const rclcpp::Time & time, const rclcpp::Duration & period) override;


            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

                
        private:
            std::shared_ptr<EPOSDriver> epos_driver_;
            std::string port_;
            std::string device_;
            std::string protocol_;
            std::string interface_;
            int node_id_;
            int baudrate_;
            int control_mode_;

            std::vector<double> cmd_velocities_;
            std::vector<double> cmd_positions_;

            std::vector<double> state_velocities_;
            std::vector<double> state_positions_;

    }; //end MobileBaseHardwareInterface

} //end namespace mobile_epos_hardware

#endif