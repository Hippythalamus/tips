cmake_minimum_required(VERSION 3.8)
project(tip_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

include_directories(
  /opt/EposCmdLib_6.8.1.0/include
  )
link_directories(
  /opt/EposCmdLib_6.8.1.0/lib/x86_64)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/mobile_epos_hwi.cpp
)

target_link_libraries(${PROJECT_NAME} EposCmd)


ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_lifecycle hardware_interface pluginlib) 

pluginlib_export_plugin_description_file(hardware_interface tip_robot_hwi.xml)


install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include
  DESTINATION include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
   rclcpp
   rclcpp_lifecycle 
   hardware_interface 
   pluginlib 
)

ament_package()
