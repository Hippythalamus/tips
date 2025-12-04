cmake_minimum_required(VERSION 3.8)
project(qos_loader_tips)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)

add_executable(qos_loader_node
  src/qos_loader_exmp.cpp
)


target_link_libraries(qos_loader_node
  yaml-cpp
)

ament_target_dependencies(qos_loader_node
  rclcpp
  sensor_msgs
)


install(
  TARGETS qos_loader_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
