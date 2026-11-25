cmake_minimum_required(VERSION 3.8)
project(driver_interface)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SutEvent.msg"
  "msg/ExecuteStep.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
