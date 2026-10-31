cmake_minimum_required(VERSION 3.16)
project(safe_redcho_distributed_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/RingProbe.msg" "msg/RingAck.msg"
 "msg/ObserverInput.msg" "msg/ObserverNeighbor.msg" "msg/ObserverResult.msg"
 "msg/ControlFrame.msg" "msg/ControlPrepared.msg" "msg/SafetyBounds.msg" "msg/ControlResult.msg" "msg/FormationNeighbor.msg" "msg/PoseRedchoNeighbor.msg")
ament_export_dependencies(rosidl_default_runtime)
ament_package()
