cmake_minimum_required(VERSION 3.8)
project(arts_esu)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/esu_node.py scripts/fake_esu_tap.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
