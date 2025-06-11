cmake_minimum_required(VERSION 3.8)
project(tars_msgs)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SerialReceive.msg"
  "msg/SerialSend.msg"
)

ament_package()