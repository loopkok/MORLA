cmake_minimum_required(VERSION 3.8)
project(xhand_control_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/XHandCommand.msg
  msg/XHandState.msg
  msg/FingerSensorState.msg
  msg/XHandSensorState.msg
  msg/XHandStateArray.msg
  msg/XHandSensorStateArray.msg
)
set(srv_files
  srv/ReadHandInfo.srv
  srv/ResetSensor.srv
  srv/SetHandId.srv
  srv/SetHandName.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
  ADD_LINTER_TESTS
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
