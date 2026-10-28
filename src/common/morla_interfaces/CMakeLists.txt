cmake_minimum_required(VERSION 3.8)
project(morla_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ComponentTarget.msg" "msg/CommandEnvelope.msg" "msg/CommandReceipt.msg"
  "msg/ComponentHealth.msg" "msg/InputEnvelope.msg"
  "srv/RuntimeOperation.srv" "srv/ControlAuthority.srv")
ament_export_dependencies(rosidl_default_runtime)
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_package()
