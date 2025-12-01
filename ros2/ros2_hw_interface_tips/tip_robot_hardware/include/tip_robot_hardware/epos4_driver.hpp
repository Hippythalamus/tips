#ifndef EPOS4_DRIVER_HPP
#define EPOS4_DRIVER_HPP

#include "Definitions.h"
#include <iostream>

#define POSITION_MODE  OMD_PROFILE_POSITION_MODE
#define VELOCITY_MODE  OMD_PROFILE_VELOCITY_MODE

#define SAFE_CALL(api_call) \
    if(key_handle_ == nullptr) { \
        std::cerr << "FATAL: " << #api_call << " called with NULL handle!" << std::endl; \
        return false; \
    }

// default parameters
static constexpr const char* DEFAULT_DEVICE_NAME        = "EPOS4";
static constexpr const char* DEFAULT_PROTOCOL           = "CANopen";
static constexpr const char* DEFAULT_INTERFACE          = "CAN";
static constexpr const char* DEFAULT_PORT               = "can0";
static constexpr int         DEFAULT_BAUDRATE           = 1000000;
static constexpr int         DEFAULT_NODE_ID            = 1;

enum class EPOSState : int {
    OK                       = 0,   // Constructor validated parameters
    INIT_OK                  = 1,   // Device was successfully opened (key handle OK)
    ACTIVE                   = 2,   // Device active (Enable state set)
    READY                    = 3,   // Motor ready for motion commands (mode set)
    WORKING                  = 4,   // Motor is executing a motion command
    STOP                     = 5,   // Motor movement was stopped

    SHUT_DOWN                = 9,   // Device was cleanly closed

    BAD_PARAMS               = 10,  // Invalid parameters passed to constructor
    INIT_FAILED              = 11,  // Failed to open device
    PROTOCOL_SETTINGS_FAILED = 12,  // Failed to configure protocol or interface
    CONNECTION_FAILED        = 13,  // Any VCS_* function returned an error
    FAILED_SHUTDOWN          = 14,  // Failed to close device
    FAILED_STOP              = 15   // Failed to stop movement
};



class EPOSDriver {
    public:
        EPOSDriver(
            const std::string& device        = DEFAULT_DEVICE_NAME,
            const std::string& protocol      = DEFAULT_PROTOCOL,
            const std::string& interface_name= DEFAULT_INTERFACE,
            const std::string& port_name     = DEFAULT_PORT,
            int node_id                      = DEFAULT_NODE_ID,
            int baudrate                     = DEFAULT_BAUDRATE)
            :
            device_(device),
            protocol_(protocol),
            interface_(interface_name),
            port_(port_name),
            node_id_(node_id),
            baudrate_(baudrate),
            error_code_(0),
            driver_state_(EPOSState::BAD_PARAMS)   // initially invalid
        {

            // Validate parameters

            if (!device_.empty() &&
                !protocol_.empty() &&
                !interface_.empty() &&
                !port_.empty() &&
                node_id_ > 0 &&
                baudrate_ > 0)
            {
                driver_state_ = EPOSState::OK;   // constructor OK
            }
            key_handle_ = 0;
            //just for example 
            if (protocol_ == "CANopen") {
                if (interface_ != "IXXAT" && interface_ != "CAN" && interface_ != "Kvaser")
                    driver_state_ = EPOSState::BAD_PARAMS;
            }

            getDriverInfo(); // print driver info
        }

        void getDriverInfo(){
            std::cout << "====== EPOS4 Driver Info ======" << std::endl;
            std::cout << "Driver state : " << int(driver_state_) << std::endl;
            std::cout << "Device : " << device_ << std::endl;
            std::cout << "Protocol : " << protocol_ << std::endl;
            std::cout << "Interface Name : " << interface_ << std::endl;
            std::cout << "Port Name : " << port_ << std::endl;
            std::cout << "Node ID : " << node_id_ << std::endl;
            std::cout << "Baudrate : " << baudrate_ << std::endl;
            std::cout << "===============================" << std::endl;
        }

        bool init(){

            unsigned int lTimeout = 0;

            std::cout << "====== EPOS4 Open Device ======" << std::endl;

            // Ensure that initialization is allowed only in correct driver states
            if(driver_state_ != EPOSState::OK && driver_state_ != EPOSState::SHUT_DOWN) {
                std::cout << "====== Device could not be openned. Driver state is wrong" << int(driver_state_)<< "======" << std::endl;
                return false;
            }
            //mutable copies
            std::string device_str = device_;
            std::string protocol_str = protocol_;
            std::string interface_str = interface_;
            std::string port_str = port_;

            // pointers to writable buffers (Maxon library makes copy of arguments - so thats legal to use pointers)
            char* pDeviceName       = device_str.empty() ? nullptr : &device_str[0];
            char* pProtocolStack    = protocol_str.empty() ? nullptr : &protocol_str[0];
            char* pInterfaceName    = interface_str.empty() ? nullptr : &interface_str[0];
            char* pPortName         = port_str.empty() ? nullptr : &port_str[0];
            // ---- OPEN DEVICE ----
            key_handle_ = VCS_OpenDevice(pDeviceName,
                                 pProtocolStack,
                                 pInterfaceName,
                                 pPortName,
                                 &error_code_);
            
            if (key_handle_ == nullptr) {
                    driver_state_ = EPOSState::INIT_FAILED;
                    std::cout << "====== Device init failed. EPOS4 error code " << error_code_<< "======" << std::endl;
                    return false;
                }
            // ---- SET BAUDRATE / TIMEOUT ----
            if(VCS_SetProtocolStackSettings(key_handle_, baudrate_, lTimeout, &error_code_)!=0){
                    driver_state_ = EPOSState::PROTOCOL_SETTINGS_FAILED;
                    std::cout << "====== Device protocol parameters setting failed. EPOS4 error code " << error_code_<< "======" << std::endl;
                    return false;
            }
            // ---- VERIFY SETTINGS ----
            unsigned int verify_baud = 0, verify_timeout = 0;
            SAFE_CALL(VCS_GetProtocolStackSettings);
            if (VCS_GetProtocolStackSettings(key_handle_, &verify_baud, &verify_timeout, &error_code_)!=0 && verify_baud != (unsigned int)baudrate_) {
                driver_state_ = EPOSState::PROTOCOL_SETTINGS_FAILED;
                std::cout << "====== Device protocol parameters setting failed. EPOS4 error code " << error_code_<< "======" << std::endl;
                return false;
            }

            driver_state_ = (EPOSState::INIT_OK); 
            return true;
        }
        bool activate() {

            int oIsFault = 0;

            std::cout << "====== EPOS4 Activating Device ======" << std::endl;

            // Ensure that activation is allowed only in correct driver states
            if(driver_state_ != EPOSState::INIT_OK) {
                std::cout << "====== Device could not be activated. Driver state is wrong : " << int(driver_state_)<< "======" << std::endl;
                return false;
            }
            SAFE_CALL(VCS_GetFaultState);
            if(VCS_GetFaultState(key_handle_, node_id_, &oIsFault, &error_code_ ) == 0)
                {
                    driver_state_ = EPOSState::CONNECTION_FAILED;
                    std::cout << "====== Device failed to get fault state. Errorcode : " << int(error_code_)<< "======" << std::endl;
                    return false;
                }
            if(oIsFault != 0)
		        {
                    std::cout << "====== Device in Fault state :" << int(oIsFault)<< " Cleaning Fault ======" << std::endl;
                    SAFE_CALL(VCS_ClearFault);
                    if(VCS_ClearFault(key_handle_, node_id_, &error_code_) == 0)
                    {
                        driver_state_ = EPOSState::CONNECTION_FAILED;
                        std::cout << "====== Device failed to clear fault state. Errorcode : " << int(error_code_)<< "======" << std::endl;
                        return false;
                    }
                    //here should be one more check of fault state
		        }
                SAFE_CALL(VCS_SetEnableState);
                if(VCS_SetEnableState(key_handle_, node_id_, &error_code_) == 0)
					{
						driver_state_ = EPOSState::CONNECTION_FAILED;
                        std::cout << "====== Device failed to set enable state. Errorcode : " << int(error_code_)<< "======" << std::endl;
                        return false;
					}
            
            driver_state_ = EPOSState::ACTIVE;
            std::cout << "====== Device is enable ======" << std::endl;
                        
            return true;
        }

        bool setControlMode(int mode){

            std::cout << "====== EPOS4 Setting Control Mode ======" << std::endl;

            // Ensure that setting is allowed only in correct driver states
            if(driver_state_ != EPOSState::ACTIVE && driver_state_ != EPOSState::STOP) {
                std::cout << "====== Control mode could not be set. Driver state is wrong :" << int(driver_state_)<< "======" << std::endl;
                return false;
            }

            switch (mode)
            {
            case VELOCITY_MODE:
                SAFE_CALL(VCS_ActivateProfileVelocityMode);
                if(VCS_ActivateProfileVelocityMode(key_handle_, node_id_, &error_code_) == 0)
                {
                    driver_state_ = EPOSState::CONNECTION_FAILED;
                    std::cout << "====== Failed to set control mode. Errorcode : " << int(error_code_)<< "======" << std::endl;
                    return false;
                }
                break;
            case POSITION_MODE:
                SAFE_CALL(VCS_ActivateProfilePositionMode);
                if(VCS_ActivateProfilePositionMode(key_handle_, node_id_, &error_code_) == 0)
                {
                    driver_state_ = EPOSState::CONNECTION_FAILED;
                    std::cout << "====== Failed to set control mode. Errorcode : " << int(error_code_)<< "======" << std::endl;
                    return false;
                }
                break;           
            default:
                    std::cout << "====== UNKNOWN mode. mode :" << int(mode)<< "======" << std::endl;
                    return false;
                break;
            }
            //here should be getMode and double check
            std::cout << "====== Device is ready. Mode " << mode <<" was set  ======" << std::endl;
            driver_state_ = EPOSState::READY;
            return true;
        }
        bool checkMode(int expected)
        {
            char current_mode = 0;
            SAFE_CALL(VCS_GetOperationMode);
            if(VCS_GetOperationMode(key_handle_, node_id_, &current_mode, &error_code_) == 0) {
                std::cout << "Failed to get operation mode. Err=" << error_code_ << std::endl;
                return false;
            }

            if(current_mode != expected) {
                std::cout << "Wrong mode. Current=" << int(current_mode)
                        << " expected=" << expected << std::endl;
                return false;
            }
            return true;
        }
        bool setPosition(int position_ticks){
                std::cout << "====== EPOS4 Setting Position ======" << std::endl;

                // Driver must be in READY state to accept motion commands
                if (driver_state_ != EPOSState::READY && driver_state_ != EPOSState::STOP) {
                    std::cout << "====== Cannot set position. Driver state is wrong: "
                            << int(driver_state_) << " ======" << std::endl;
                    return false;
                }

                // Verify actual EPOS mode before sending target
                if(!checkMode(POSITION_MODE)) return false;

                // Move to absolute target position
                // Parameters: target, absolute(0), immediately start(1)
                SAFE_CALL(VCS_MoveToPosition);
                if(VCS_MoveToPosition(key_handle_, node_id_, position_ticks, 0, 1, &error_code_) == 0)
                {
                    std::cout << "====== Failed to set position. Error: " 
                                << error_code_ << " ======" << std::endl;
                        driver_state_ = EPOSState::CONNECTION_FAILED;
                        return false;
                }

                driver_state_ = EPOSState::WORKING;
                std::cout << "====== Position set to " << position_ticks << " rpm ======" << std::endl;
                return true;
        }
        bool setVelocity(int velocity_rpm){
                std::cout << "====== EPOS4 Setting Velocity ======" << std::endl;
                // Driver must be ready
                if (driver_state_ != EPOSState::READY && driver_state_ != EPOSState::STOP) {
                    std::cout << "====== Cannot set velocity. Driver state is wrong: "
                            << int(driver_state_) << " ======" << std::endl;
                    return false;
                }
                // Verify actual EPOS mode before sending target
                if(!checkMode(VELOCITY_MODE)) return false;
                // Apply velocity command
                SAFE_CALL(VCS_MoveWithVelocity);
                if (VCS_MoveWithVelocity(key_handle_, node_id_, velocity_rpm, &error_code_) == 0) {
                    std::cout << "====== Failed to set velocity. Error: " 
                            << error_code_ << " ======" << std::endl;
                    driver_state_ = EPOSState::CONNECTION_FAILED;
                    return false;
                }
                driver_state_ = EPOSState::WORKING;
                std::cout << "====== Velocity set to " << velocity_rpm << " rpm ======" << std::endl;
                return true;
        }
        int  getPosition(){
            // Reading allowed both in READY and ACTIVE state
            if(driver_state_ != EPOSState::READY &&
            driver_state_ != EPOSState::ACTIVE)
            {
                std::cout << "====== Cannot read position. Driver state is wrong: "
                        << int(driver_state_) << " ======" << std::endl;
                return 0;
            }

            int position = 0;
            SAFE_CALL(VCS_GetPositionIs);
            if(VCS_GetPositionIs(key_handle_, node_id_, &position, &error_code_) == 0)
            {
                std::cout << "====== Failed to read position. Error code: "
                        << error_code_ << " ======" << std::endl;
                return 0;
            }

            return position;
        }
        int  getVelocity(){
               // Reading allowed both in READY and ACTIVE state
               if(driver_state_ != EPOSState::READY &&
                    driver_state_ != EPOSState::ACTIVE)
                    {
                        std::cout << "====== Cannot read velocity. Driver state is wrong: "
                                << int(driver_state_) << " ======" << std::endl;
                        return 0;
                    }

                    int velocity = 0;
                    SAFE_CALL(VCS_GetVelocityIs);
                    if(VCS_GetVelocityIs(key_handle_, node_id_, &velocity, &error_code_) == 0)
                    {
                        std::cout << "====== Failed to read velocity. Error code: "
                                << error_code_ << " ======" << std::endl;
                        return 0;
                    }

                    return velocity;
        }
        bool stop(){

            std::cout << "====== EPOS4 Hault movement ======" << std::endl;
            if(driver_state_ != EPOSState::WORKING)
                    {
                        std::cout << "====== Cannot stop motor. Driver state is wrong: "
                                << int(driver_state_) << " ======" << std::endl;
                        return 0;
                    }
            if(checkMode(VELOCITY_MODE)){
                 SAFE_CALL(VCS_HaltVelocityMovement);
                 if(VCS_HaltVelocityMovement(key_handle_, node_id_, &error_code_) == 0)
                    {
                        driver_state_ = EPOSState::FAILED_STOP;
                            std::cout << "====== Failed to hault movement. Errorcode : " << int(error_code_)<< "======" << std::endl;
                            return false;
                    }
                     
            }
            if(checkMode(POSITION_MODE)){
                SAFE_CALL(VCS_HaltPositionMovement);
                if(VCS_HaltPositionMovement(key_handle_, node_id_, &error_code_) == 0)
                    {
                        driver_state_ = EPOSState::FAILED_STOP;
                            std::cout << "====== Failed to hault movement. Errorcode : " << int(error_code_)<< "======" << std::endl;
                            return false;
                    }
            }
                
            std::cout << "====== EPOS4 motor was stopped  ======" << std::endl;  
            driver_state_ = EPOSState::STOP;
            return true;
        }
        bool shutdown(){
            if(key_handle_ == nullptr){
                std::cout << "====== Device handle is null. Nothing to close. ======" << std::endl;
                driver_state_ = EPOSState::SHUT_DOWN;
                return true;
            }
            if(VCS_CloseDevice(key_handle_, &error_code_) == 0){
                driver_state_ = EPOSState::FAILED_SHUTDOWN;
                std::cout << "====== Failed to close device. Errorcode :"
                        << error_code_<< "======" << std::endl;
                return false;
            }

            std::cout << "====== EPOS4 Device was closed ======" << std::endl;  
            driver_state_ = EPOSState::SHUT_DOWN;
            key_handle_ = nullptr;
            return true;
        }

         EPOSState state() const { return driver_state_; }

    private:
        void* key_handle_;

        std::string device_;
        std::string protocol_;
        std::string interface_;
        std::string port_;
        int node_id_;
        int baudrate_;
        unsigned int error_code_;
        EPOSState driver_state_; 
};

#endif