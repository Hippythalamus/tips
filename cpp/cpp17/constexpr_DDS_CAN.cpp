/*
TIPS 
VERY simple example
Using if constexpr in DDS msg and CAN msg processing
*/

#include <iostream>
#include <type_traits>

struct SensorMsg { int timestamp = 0; int sensor_value;  };
struct ControlMsg { int command; };

/* ============= Ex 1 ============
Processing msg depends on its fields*/
template<typename T, typename = void>
struct HasTimestamp : std::false_type {};

template<typename T>
struct HasTimestamp<T, std::void_t<decltype(T().timestamp)>> : std::true_type {};

template<typename Msg>
void printTimestamp(const Msg& msg) {
    if constexpr (HasTimestamp<Msg>::value) {
        std::cout << "Timestamp: " << msg.timestamp << "\n";
    } else {
        std::cout << "No timestamp in this message\n";
    }
}



/* ============= Ex 2 ============
Processing msg depends on its type*/
template<typename Msg>
void processMessage(const Msg& msg) {
    if constexpr (std::is_same_v<Msg, SensorMsg>) {
        std::cout << "Processing sensor message: " << msg.sensor_value << "\n";
    } 
    else if constexpr (std::is_same_v<Msg, ControlMsg>) {
        std::cout << "Processing control message: " << msg.command << "\n";
    } 
}

/* ============= Ex 3 ============
Filtering CAN nsg*/

struct CanMsg { int id; int data; };
struct NonCanMsg { int data; };

template<typename Msg>
void sendCanMessage(const Msg& msg) {
    if constexpr (std::is_member_object_pointer_v<decltype(&Msg::id)>) {
        std::cout << "Sending CAN message with id: " << msg.id << "\n";
    } else {
        static_assert(std::is_member_object_pointer_v<decltype(&Msg::id)>, 
                      "Message does not have 'id', cannot  be send via CAN!");
    }
}

int main() {
    SensorMsg sensor{42};
    ControlMsg control{7};

    processMessage(sensor); // Processing sensor message: 42
    processMessage(control); // Processing control message: 7


    printTimestamp(sensor); // Timestamp 42
    printTimestamp(control); // No timestamp in this message


    CanMsg can{10, 255};
    sendCanMessage(can); // Sending CAN message with id: 10

    
    // sendCanMessage(control); //  errror  no member named 'id' in 'ControlMsg'
}