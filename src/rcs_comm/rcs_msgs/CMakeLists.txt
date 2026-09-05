cmake_minimum_required(VERSION 3.8)
project(rcs_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChassisCmd.msg"
  "msg/ArmCmd.msg"
  "msg/SensorState.msg"
  "msg/VisionData.msg"
  "srv/ResetLocalization.srv"
  "srv/SetTask.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
