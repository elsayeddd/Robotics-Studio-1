cmake_minimum_required(VERSION 3.8)
project(embertrace_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EnvironmentalReading.msg"
  "msg/VisualCue.msg"
  "msg/ThermalCue.msg"
  "msg/PerceptionStatus.msg"
  "msg/TrialTruth.msg"
  "msg/FireState.msg"
  "msg/FireAlert.msg"
  "msg/MissionState.msg"
  "srv/OperatorCommand.srv"
  "srv/SetFireState.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
