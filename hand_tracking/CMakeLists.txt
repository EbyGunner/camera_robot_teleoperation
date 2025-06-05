cmake_minimum_required(VERSION 3.15)  # or higher, depending on your ROS2 distro requirements

project(hand_tracking)  # your package name here

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_package()