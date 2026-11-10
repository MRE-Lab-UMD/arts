# ESP-IDF project for the ARTS camera sync box.
#
# Build with the ESP-IDF toolchain, NOT with colcon — the enclosing directory is an ament
# package only so that the launch file, config and docs ship with the workspace.
#
#   idf.py set-target esp32
#   idf.py menuconfig          # "ARTS camera sync" menu selects the Stage 5 run mode
#   idf.py build flash monitor
#
# The MICROROS run mode additionally needs the micro-ROS component and ARTS's own messages;
# see README.md in this package.
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(arts_camera_sync)
