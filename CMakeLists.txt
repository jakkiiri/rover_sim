cmake_minimum_required(VERSION 3.8)
project(rover_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(rover_swerve_actuators SHARED src/swerve_gazebo_plugin.cpp)
target_compile_features(rover_swerve_actuators PUBLIC cxx_std_17)
ament_target_dependencies(rover_swerve_actuators gazebo_dev gazebo_ros rclcpp sensor_msgs std_msgs
  nav_msgs diagnostic_msgs tf2_msgs geometry_msgs)
install(TARGETS rover_swerve_actuators LIBRARY DESTINATION lib)

# Optional Gazebo Fortress backend (gz-sim 6). Built as a separate CMake
# sub-project: finding gazebo_dev and ignition-gazebo6 in one project makes
# both import the JsonCpp target and CMake aborts.
find_path(ROVER_SIM_IGN_GAZEBO6_CMAKE NAMES ignition-gazebo6-config.cmake
  PATHS /usr/lib/x86_64-linux-gnu/cmake/ignition-gazebo6 /usr/lib/cmake/ignition-gazebo6 /usr/share/cmake/ignition-gazebo6
  NO_DEFAULT_PATH)
if(ROVER_SIM_IGN_GAZEBO6_CMAKE)
  include(ExternalProject)
  ExternalProject_Add(rover_swerve_gz_system_project
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/gz_system
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/gz_system
    CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}
               -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/gz_system/install
    BUILD_ALWAYS TRUE)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/gz_system/install/lib/librover_swerve_gz_system.so DESTINATION lib)
  message(STATUS "rover_sim: building the Gazebo Fortress system plugin (gz_system/)")
else()
  message(STATUS "rover_sim: Gazebo Fortress (ignition-gazebo6) not found; Classic only")
endif()
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/swerve_controller
  scripts/rover_teleop scripts/validate_teleop.py scripts/validate_hardening.py
  scripts/validate_shutdown.py
  scripts/generate_terrain.py scripts/measure_performance.py scripts/validate_terrain.py
  scripts/validate_step_climb.py scripts/validate_slope.py scripts/validate_suspension.py
  scripts/validate_lidar3d.py scripts/validate_controller.py scripts/validate_traverse.py
  scripts/lidar_postprocess scripts/rover_teleop_joy scripts/run_hardening_and_teleop.py
  DESTINATION lib/${PROJECT_NAME})

# Procedural Mars terrain assets are generated at build time (deterministic
# from the default seed) and installed under share/rover_sim/generated.
set(ROVER_SIM_TERRAIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(GLOB ROVER_SIM_TERRAIN_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/rover_sim/terrain/*.py)
add_custom_command(
  OUTPUT ${ROVER_SIM_TERRAIN_DIR}/terrain_manifest.json
  COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
          ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_terrain.py
          --output-dir ${ROVER_SIM_TERRAIN_DIR} --quiet
  DEPENDS ${ROVER_SIM_TERRAIN_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_terrain.py
  COMMENT "Generating Mars terrain assets (seed 7)")
add_custom_target(rover_sim_terrain ALL DEPENDS ${ROVER_SIM_TERRAIN_DIR}/terrain_manifest.json)
install(DIRECTORY ${ROVER_SIM_TERRAIN_DIR}/ DESTINATION share/${PROJECT_NAME}/generated)

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
  ament_add_pytest_test(terrain_generator test/test_terrain.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(validation_harness test/test_harness.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()

install(DIRECTORY launch worlds urdf LICENSES docs config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES THIRD_PARTY_NOTICES.md README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
