cmake_minimum_required(VERSION 3.8)
project(arts_camera_sync)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# firmware/ is deliberately NOT built here: it is an ESP-IDF project for the xtensa
# toolchain. Installing the sources anyway keeps the flashing recipe with the workspace.
#
# The exclusions mirror firmware/.gitignore and are load-bearing, not tidiness: the
# fetched micro-ROS component plus an ESP-IDF build tree are 644 MB of generated
# third-party content, and copying them on every colcon build also fails outright the
# moment one of the fetched .git hooks is read-only. Keep this list in step with
# firmware/.gitignore. Note "sdkconfig" matches that name exactly, not sdkconfig.defaults*.
install(DIRECTORY firmware
  DESTINATION share/${PROJECT_NAME}
  PATTERN "build" EXCLUDE
  PATTERN "components" EXCLUDE
  PATTERN "managed_components" EXCLUDE
  PATTERN "sdkconfig" EXCLUDE
  PATTERN "sdkconfig.old" EXCLUDE
  PATTERN "dependencies.lock" EXCLUDE
)

ament_package()
