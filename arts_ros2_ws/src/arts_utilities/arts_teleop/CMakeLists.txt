cmake_minimum_required(VERSION 3.8)
project(arts_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/footswitch.py scripts/teleop_follower.py scripts/head_sensor.py
  scripts/camera_teleop.py
  scripts/mtm_hold.py
  scripts/gamepad_jog.py scripts/jaw_follower.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
