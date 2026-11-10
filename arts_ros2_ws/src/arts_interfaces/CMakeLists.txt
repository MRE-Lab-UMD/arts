cmake_minimum_required(VERSION 3.8)
project(arts_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/ArmStatus.msg"
  "msg/CameraSyncPulse.msg"
  "msg/EnergyCommand.msg"
  "msg/EnergyState.msg"
  "msg/EnergyTarget.msg"
  "msg/EpisodeMark.msg"
  "msg/EsuInfo.msg"
  "msg/RcmState.msg"
  "msg/StylusState.msg"
  "msg/TeleopState.msg"
  "msg/ToolInfo.msg"
  "msg/ToolTipState.msg"
  "msg/ToolTipTarget.msg"
)

set(srv_files
  "srv/SetRcm.srv"
  "srv/SetToolTipOffset.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
)

ament_package()
