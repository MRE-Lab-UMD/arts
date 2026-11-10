cmake_minimum_required(VERSION 3.8)
project(arts_safety)

find_package(ament_cmake REQUIRED)

# The collision guard (Python, rclpy): capsule distances between and within the arms, and the
# latched /arts/collision_hold every servo controller obeys.
install(PROGRAMS scripts/collision_guard.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
