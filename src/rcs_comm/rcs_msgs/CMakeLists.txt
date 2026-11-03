cmake_minimum_required(VERSION 3.8)
project(rcs_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GcsDetection.msg"
  "msg/GcsDetections.msg"
  "msg/GcsTaskCode.msg"
  "msg/GcsMode.msg"
  "msg/GcsGripperEvent.msg"
  "msg/GcsStatus.msg"
  "msg/ActionCommand.msg"
  "msg/ActionFeedback.msg"
  "msg/ChassisCmd.msg"
  "msg/ArmCmd.msg"
  "msg/SensorState.msg"
  "msg/VisionData.msg"
  "srv/ResetLocalization.srv"
  "srv/SetTask.srv"
  DEPENDENCIES std_msgs
)

# Install alongside the generated rcs_msgs Python package without a second
# ament_python_install_package target conflicting with rosidl generation.
find_package(ament_cmake_python REQUIRED)
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY business DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")

ament_export_dependencies(rosidl_default_runtime)
ament_package()
