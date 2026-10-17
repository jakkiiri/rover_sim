cmake_minimum_required(VERSION 3.8)
project(rover_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(rover_swerve_actuators SHARED src/swerve_gazebo_plugin.cpp)
target_compile_features(rover_swerve_actuators PUBLIC cxx_std_17)
ament_target_dependencies(rover_swerve_actuators gazebo_dev gazebo_ros rclcpp sensor_msgs std_msgs)
install(TARGETS rover_swerve_actuators LIBRARY DESTINATION lib)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/swerve_controller scripts/validate_swerve.py
  scripts/rover_teleop scripts/validate_teleop.py scripts/validate_hardening.py
  scripts/validate_shutdown.py scripts/validate_lidar.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(swerve_kinematics test/test_kinematics.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(keyboard_teleop test/test_teleop.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(swerve_controller_state test/test_controller.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(lidar_description test/test_lidar.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()

install(DIRECTORY launch worlds urdf LICENSES docs config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES THIRD_PARTY_NOTICES.md README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
