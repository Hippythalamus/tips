cmake_minimum_required(VERSION 3.8)
project(motor_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include_directories(
  /opt/EposCmdLib_6.8.1.0/include
  )
include_directories(
  include
  )
link_directories(
  /opt/EposCmdLib_6.8.1.0/lib/x86_64)

add_executable(motors1 src/motors1.cpp)

ament_target_dependencies(motors1 rclcpp dynamixel_sdk)


target_link_libraries(motors1 EposCmd)

install(TARGETS
  motors1
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
