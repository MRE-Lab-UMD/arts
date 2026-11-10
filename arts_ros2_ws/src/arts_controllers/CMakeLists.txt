cmake_minimum_required(VERSION 3.8)
project(arts_controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(arts_interfaces REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(Eigen3 3.4 REQUIRED)

# --- pinned dependency version assertions -------------------------------------------------
# These are not defensive boilerplate. On 2026-08-10 an apt dependency of a package we do
# not even use installed a second Ruckig into /opt/ros/jazzy; because the underlay is
# always on CMAKE_PREFIX_PATH, find_package() would have silently preferred it over the
# pinned copy, five minor versions off, inside the 500 Hz control path, and it would have
# compiled cleanly. A wrong library here has to fail at configure time or it does not fail
# at all. Each check names the version found and where it came from, because the useful
# question is always "which package dragged this in".

# Ruckig comes from apt (ros-jazzy-ruckig, upstream 0.9.2), never from source.
# open_source_software/ruckig/VERSION carries the evidence behind that decision.
find_package(ruckig REQUIRED)
if(NOT ruckig_VERSION VERSION_EQUAL "0.9.2")
  message(FATAL_ERROR
    "Found Ruckig ${ruckig_VERSION} at ${ruckig_DIR}, but ARTS requires exactly 0.9.2 "
    "from ros-jazzy-ruckig.\n"
    "A different version reaching CMAKE_PREFIX_PATH is how the 500 Hz smoother silently "
    "links the wrong library. Find the extra copy before overriding this check; "
    "`dpkg -S` on the path above is usually enough to name the package that pulled it "
    "in. Rationale: open_source_software/ruckig/VERSION.")
endif()

# pinocchio and coal come from SOURCE, pinned in open_source_software/*/VERSION and built
# into $ARTS_LOCAL. The opposite call from Ruckig, and deliberately so: apt is already
# shipping pinocchio 4.0.0, a major version ahead with breaking API changes, so these are
# moving rather than frozen -- and they produce numbers we publish.
find_package(pinocchio REQUIRED)
if(NOT pinocchio_VERSION VERSION_EQUAL "3.9.0")
  message(FATAL_ERROR
    "Found pinocchio ${pinocchio_VERSION} at ${pinocchio_DIR}, but ARTS pins 3.9.0.\n"
    "If this resolved to /opt/ros/jazzy you have picked up ros-jazzy-pinocchio instead of "
    "the source build in $ARTS_LOCAL; source the overlay or remove the apt package. "
    "Rationale: open_source_software/pinocchio/VERSION.")
endif()

# coal is only used transitively today (pinocchio's collision support links it) but is
# asserted here anyway, because coal is built WITH the hpp-fcl compatibility shim and so
# answers to two names -- which means apt's ros-jazzy-hpp-fcl could shadow it.
find_package(coal REQUIRED)
if(NOT coal_VERSION VERSION_EQUAL "3.0.4")
  message(FATAL_ERROR
    "Found coal ${coal_VERSION} at ${coal_DIR}, but ARTS pins 3.0.4.\n"
    "Rationale: open_source_software/coal/VERSION.")
endif()

# proxsuite: the task-layer QP (decision 17), from SOURCE like pinocchio and coal; apt's
# ros-jazzy-proxsuite (0.6.5) would otherwise win from /opt/ros/jazzy. Header-only.
find_package(proxsuite REQUIRED)
if(NOT proxsuite_VERSION VERSION_EQUAL "0.7.3")
  message(FATAL_ERROR
    "Found proxsuite ${proxsuite_VERSION} at ${proxsuite_DIR}, but ARTS pins 0.7.3.\n"
    "Build it with open_source_software/proxsuite/build.sh (arts_install.sh does) and "
    "put its prefix on CMAKE_PREFIX_PATH ahead of /opt/ros/jazzy.")
endif()

generate_parameter_library(cartesian_servo_controller_parameters
  src/cartesian_servo_controller_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/cartesian_servo_controller.cpp
  src/task_qp.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  cartesian_servo_controller_parameters
  pinocchio::pinocchio
  # Linked explicitly even though it is only used through pinocchio. As a purely
  # transitive dependency its directory never joined the link path, so RPATH did not
  # cover it and the library failed to load at runtime while linking without complaint.
  coal::coal
  ruckig::ruckig
  proxsuite::proxsuite
  Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  arts_interfaces
)

# pinocchio and coal are installed OUTSIDE the ament prefix (in $ARTS_LOCAL), so nothing
# puts their lib directories on the runtime search path. Without this the library links
# fine and then fails to LOAD, and because it is loaded by pluginlib the error surfaces as
# "controller type not found" rather than as a missing shared object -- a genuinely
# misleading symptom to debug. Baking the resolved link directories into the RPATH keeps
# the two build paths honest: each build records wherever its own dependencies actually
# landed, which differs between bare metal and the container.
set_target_properties(${PROJECT_NAME} PROPERTIES
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

pluginlib_export_plugin_description_file(controller_interface arts_controllers_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME} cartesian_servo_controller_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Pure-math test of the task-layer QP: the RCM equality is exact, bounds hold, and a
  # solve does not touch the heap. No ROS graph needed.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(task_qp_test test/task_qp_test.cpp src/task_qp.cpp)
  target_include_directories(task_qp_test PRIVATE include)
  target_link_libraries(task_qp_test proxsuite::proxsuite Eigen3::Eigen)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  geometry_msgs
  arts_interfaces
)

ament_package()
