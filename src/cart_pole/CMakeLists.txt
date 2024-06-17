cmake_minimum_required(VERSION 3.5)
project(cart_pole)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(cartpole_controller src/cartpole_controller.cpp)
ament_target_dependencies(cartpole_controller rclcpp sensor_msgs geometry_msgs std_srvs)

install(TARGETS
  cartpole_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
