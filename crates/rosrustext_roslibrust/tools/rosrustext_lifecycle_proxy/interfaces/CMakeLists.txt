cmake_minimum_required(VERSION 3.8)
project(rosrustext_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Status.msg"
  "srv/GetTransitionGraph.srv"
  DEPENDENCIES lifecycle_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
