cmake_minimum_required(VERSION 3.8)
project(arts_dv_tool_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(disc_drive src/disc_drive_node.cpp)
target_compile_features(disc_drive PUBLIC cxx_std_17)
target_include_directories(disc_drive PRIVATE include)
target_link_libraries(disc_drive rclcpp::rclcpp ${sensor_msgs_TARGETS} ${std_msgs_TARGETS}
  ${std_srvs_TARGETS} Eigen3::Eigen)
ament_target_dependencies(disc_drive dynamixel_sdk)

# Automatic instrument homing (the adapter designer's node, ported 2026-11-04; src/homing/).
# Its config files (config/homing/) are read from the installed share directory.
add_executable(tool_homing src/homing/tool_homing_main.cpp src/homing/tool_homing_node.cpp
  src/homing/dxl_interface.cpp)
target_compile_features(tool_homing PUBLIC cxx_std_17)
target_include_directories(tool_homing PRIVATE include)
target_compile_definitions(tool_homing PRIVATE
  ADAPTER_CONFIG_DIR="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/homing")
target_link_libraries(tool_homing rclcpp::rclcpp ${sensor_msgs_TARGETS} ${std_msgs_TARGETS}
  Eigen3::Eigen)
ament_target_dependencies(tool_homing dynamixel_sdk)

install(TARGETS disc_drive tool_homing DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/fake_instrument.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # The engagement sequence against a simulated adapter (no bus, no hardware).
  ament_add_gtest(engage_sequence_test test/engage_sequence_test.cpp)
  target_include_directories(engage_sequence_test PRIVATE include)
  target_compile_features(engage_sequence_test PUBLIC cxx_std_17)
  # The instrument joint math (coupling, limits) with the needle driver's file values.
  ament_add_gtest(instrument_joints_test test/instrument_joints_test.cpp)
  target_include_directories(instrument_joints_test PRIVATE include)
  target_compile_features(instrument_joints_test PUBLIC cxx_std_17)
  target_link_libraries(instrument_joints_test Eigen3::Eigen)
endif()

ament_package()
