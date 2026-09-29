cmake_minimum_required(VERSION 3.8)
project(robot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/radar_station/Situation.msg"
  
  "msg/referee/AllRobotHP.msg"
  "msg/referee/FriendLocation.msg"
  "msg/referee/GameStatus.msg"
  "msg/referee/RFID.msg"
  "msg/referee/RfidStatus.msg"
  "msg/referee/RobotStatus.msg"
  "msg/referee/RobotControl.msg"

  "msg/PlannerStatus.msg"
  "msg/PidDebug.msg"
  "msg/TrajPoint.msg"
  "msg/Trajectory.msg"
  "msg/ChassisCmd.msg"
  "msg/NavData.msg"
  "msg/RobotState.msg"
  "msg/RemoteStatus.msg"
  "msg/AutoAimMode.msg"
  "msg/AutoAimStatus.msg"
  "msg/ChassisMode.msg"
  "msg/SuperCapacitor.msg"
  "msg/GimbalTarget.msg"
  "msg/ChassisEnergy.msg"
  "msg/RuneStatus.msg"

  DEPENDENCIES
    std_msgs
    geometry_msgs
    builtin_interfaces
)

ament_export_include_directories(include)
ament_package()
