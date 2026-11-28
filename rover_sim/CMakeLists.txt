cmake_minimum_required(VERSION 3.8)
project(rover_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Gazebo Fortress (gz-sim 6) swerve actuator System; the URDFs load it as
# librover_swerve_gz_system.so (found through IGN_GAZEBO_SYSTEM_PLUGIN_PATH).
add_library(rover_swerve_gz_system SHARED src/swerve_gz_system.cpp)
target_compile_features(rover_swerve_gz_system PUBLIC cxx_std_17)
target_link_libraries(rover_swerve_gz_system ignition-gazebo6::ignition-gazebo6 ignition-plugin1::register)
ament_target_dependencies(rover_swerve_gz_system rclcpp sensor_msgs std_msgs nav_msgs)
install(TARGETS rover_swerve_gz_system LIBRARY DESTINATION lib)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/swerve_controller scripts/validate_swerve.py
  scripts/rover_teleop scripts/validate_teleop.py scripts/validate_hardening.py
  scripts/validate_shutdown.py scripts/validate_lidar.py scripts/lidar_postprocess
  scripts/validate_lidar3d.py scripts/terrain_chunks scripts/gazebo_gui DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/stereo_camera_relay scripts/stereo_processor scripts/generate_sand_texture.py
  scripts/validate_stereo.py scripts/validate_launch_matrix.py scripts/generate_mars_world.py
  scripts/generate_real_rover.py scripts/import_usgs_terrain.py scripts/validate_survey_terrain.py
  DESTINATION lib/${PROJECT_NAME})

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
  # Stereo camera system (self-contained; see docs/stereo.md).
  foreach(stereo_test stereo_geometry stereo_matching stereo_truth stereo_description
          stereo_nodes stereo_launch stereo_world)
    ament_add_pytest_test(${stereo_test} test/test_${stereo_test}.py
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  endforeach()
  ament_add_pytest_test(lidar3d test/test_lidar3d.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(real_rover test/test_real_rover.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(fortress_worlds test/test_fortress_worlds.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(gpu_lidar_check test/test_gpu_lidar_check.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(fortress_launch test/test_fortress_launch.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(survey_terrain test/test_survey_terrain.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(usgs_sites test/test_usgs_sites.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(gazebo_gui test/test_gazebo_gui.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()

install(DIRECTORY launch worlds urdf LICENSES docs config rviz
  DESTINATION share/${PROJECT_NAME}
)

# Model/material resource folder; launch/desert.launch.py puts it on IGN_GAZEBO_RESOURCE_PATH.
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

# Real rover meshes (scripts/generate_real_rover.py); urdf/real_rover.urdf.xacro uses file:// URIs.
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})

install(FILES THIRD_PARTY_NOTICES.md README.md requirements-terrain.txt
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
