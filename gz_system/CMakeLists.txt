# Stand-alone sub-project for the Gazebo Fortress (gz-sim 6) system plugin.
# It is configured separately from the main package because the Classic and
# Fortress dependency trees both define the imported JsonCpp target.
cmake_minimum_required(VERSION 3.8)
project(rover_swerve_gz_system)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
add_library(rover_swerve_gz_system SHARED swerve_gz_system.cpp)
target_compile_features(rover_swerve_gz_system PUBLIC cxx_std_17)
target_link_libraries(rover_swerve_gz_system ignition-gazebo6::ignition-gazebo6 ignition-plugin1::register
  rclcpp::rclcpp ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${nav_msgs_TARGETS} ${diagnostic_msgs_TARGETS}
  ${tf2_msgs_TARGETS} ${geometry_msgs_TARGETS})
install(TARGETS rover_swerve_gz_system LIBRARY DESTINATION lib)
