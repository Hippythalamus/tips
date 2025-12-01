#include "tip_robot_hardware/mobile_epos_hwi.hpp"
#include <rclcpp/rclcpp.hpp>


namespace mobile_epos_hardware {

        std::vector<hardware_interface::StateInterface> MobileBaseHardwareInterface::export_state_interfaces()
            {
                std::vector<hardware_interface::StateInterface> state_interfaces;
                state_interfaces.emplace_back("base_left_wheel_joint", "velocity", &state_velocities_[0]);
                state_interfaces.emplace_back("base_right_wheel_joint", "velocity", &state_velocities_[1]);


                state_interfaces.emplace_back("base_left_wheel_joint", "position", &state_positions_[0]);
                state_interfaces.emplace_back("base_right_wheel_joint", "position", &state_positions_[1]);

                return state_interfaces;
            }

        std::vector<hardware_interface::CommandInterface> MobileBaseHardwareInterface::export_command_interfaces()
        {
            std::vector<hardware_interface::CommandInterface> command_interfaces;

            command_interfaces.emplace_back("base_left_wheel_joint", "velocity", &cmd_velocities_[0]);
            command_interfaces.emplace_back("base_right_wheel_joint", "velocity", &cmd_velocities_[1]);


            command_interfaces.emplace_back("base_left_wheel_joint", "position", &cmd_positions_[0]);
            command_interfaces.emplace_back("base_right_wheel_joint", "position", &cmd_positions_[1]);


            return command_interfaces;
        }

        hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_init(const hardware_interface::HardwareInfo & info) {
        
        if(
            hardware_interface::SystemInterface::on_init(info) != 
            hardware_interface::CallbackReturn::SUCCESS)
            {
                RCLCPP_ERROR(
                rclcpp::get_logger("MobileBaseHardwareInterface"),
                "EPOS Inition failed");
                return hardware_interface::CallbackReturn::ERROR;
            }
        info_ = info;
        device_ = info_.hardware_parameters["device_name"];
        protocol_ = info_.hardware_parameters["protocol"];
        interface_ = info_.hardware_parameters["interface"];
        port_ = info_.hardware_parameters["port"];
        baudrate_ = std::stoi(info_.hardware_parameters["baudrate"]);
        node_id_ = std::stoi(info_.hardware_parameters["node_id"]);
        control_mode_ = std::stoi(info_.hardware_parameters["control_mode"]);

        cmd_positions_.resize(2, 0.0);
        cmd_velocities_.resize(2, 0.0);
        state_positions_.resize(2, 0.0);
        state_velocities_.resize(2, 0.0);
        //check interfaces

        try {
            epos_driver_ = std::make_shared<EPOSDriver>(device_, protocol_, interface_, port_, node_id_, baudrate_);
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("MobileBaseHardwareInterface"),
                        "Failed to initialize EPOSDriver on port %s: %s",
                        port_.c_str(), e.what()
                    );
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state) {
        (void)previous_state;

           
        if(epos_driver_->init() != 0){

             RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
                "Fail to init driver");

            return hardware_interface::CallbackReturn::ERROR;
        }

        epos_driver_->setControlMode(control_mode_);
        
        return hardware_interface::CallbackReturn::SUCCESS;
     }
    hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state) { 

        (void)previous_state;


        if(!epos_driver_->activate()){
            RCLCPP_ERROR(
                        rclcpp::get_logger("MobileBaseHardwareInterface"),
                        "Device wasn't activated"
                    );
        }


        epos_driver_->setVelocity(0.0);

        epos_driver_->setPosition(0.0);


        return hardware_interface::CallbackReturn::SUCCESS;

    }
            
    hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
        (void)previous_state;
        epos_driver_->stop();
        epos_driver_->shutdown();

        return hardware_interface::CallbackReturn::SUCCESS;
     }


    hardware_interface::return_type MobileBaseHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period) { 
        (void)time;
        (void)period;
        double vel = (double)epos_driver_->getVelocity();

        state_velocities_[0] = vel;

        double pos = (double)epos_driver_->getPosition();

        state_positions_[0] = pos;

        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type MobileBaseHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period) { 
        (void)time;
        (void)period;
        if(control_mode_ == VELOCITY_MODE){
            epos_driver_->setVelocity((int)cmd_velocities_[0]);
        }
        else if (control_mode_ == POSITION_MODE)
        {
             epos_driver_->setPosition((int)cmd_positions_[0]);
        }

        return hardware_interface::return_type::OK;
    }
        

} //end namespace mobile_epos_hardware 

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mobile_epos_hardware::MobileBaseHardwareInterface, hardware_interface::SystemInterface)



