cmake_minimum_required(VERSION 3.5)
project(roborts_msgs)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add message, service, and action files
set(msg_files
    "msg/ChassisCmd.msg"

    "msg/ChassisDebug.msg"
    "msg/AutoAimCmd.msg"
    "msg/AutoAimInf.msg"
    "msg/RefereeRMUL.msg"
    "msg/RefereeRMUC.msg"
    "msg/ChassisCmdMode.msg"

    "msg/AllRobotHP.msg"
    "msg/SentryBullet.msg"
    "msg/TimeRemain.msg"
    "msg/SentryStatus.msg"
    "msg/GamesStatus.msg"

    "msg/TwistAccel.msg"
    "msg/GimbalCmd.msg"
    "msg/GimbalAngle.msg"
    "msg/GimbalRate.msg"
    "msg/ObstacleMsg.msg"
    "msg/ShootInfo.msg"
    "msg/ShootState.msg"
    "msg/referee_system/BonusStatus.msg"
    "msg/referee_system/GameResult.msg"
    "msg/referee_system/GameStatus.msg" 
    "msg/referee_system/GameSurvivor.msg"
    "msg/referee_system/ProjectileSupply.msg"
    "msg/referee_system/RobotBonus.msg"
    "msg/referee_system/RobotDamage.msg"
    "msg/referee_system/RobotHeat.msg"
    "msg/referee_system/RobotShoot.msg"
    "msg/referee_system/RobotStatus.msg" 
    "msg/referee_system/SupplierStatus.msg"
)

set(action_files
    "action/LocalPlanner.action"
    "action/GlobalPlanner.action"
    "action/ArmorDetection.action"
)

set(srv_files
    "srv/GimbalMode.srv"
    "srv/FricWhl.srv"
    "srv/ShootCmd.srv"
    "srv/AutoAimWorking.srv"
    "srv/UpdateMapTF.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs nav_msgs action_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
