cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)
project(autonomy_datasets_msgs)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

ament_export_dependencies(
    diagnostic_msgs
    std_msgs
)

set(msg_files
    "msg/ObjectListMetaInfo.msg"
    "msg/ObjectMetaInfo.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES diagnostic_msgs std_msgs
)

ament_package()
