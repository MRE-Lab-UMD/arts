cmake_minimum_required(VERSION 3.8)
project(arts_gui)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/arts_gui.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
