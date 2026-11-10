cmake_minimum_required(VERSION 3.8)
project(arts_display)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(arts_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

add_library(stereo_display SHARED src/stereo_display.cpp)
target_include_directories(stereo_display PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(stereo_display ${OpenCV_LIBS})
ament_target_dependencies(stereo_display rclcpp rclcpp_components sensor_msgs arts_interfaces
  geometry_msgs std_msgs)
rclcpp_components_register_nodes(stereo_display "arts_display::StereoDisplay")

install(TARGETS stereo_display
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
