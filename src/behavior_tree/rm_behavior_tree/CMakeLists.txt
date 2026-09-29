cmake_minimum_required(VERSION 3.10)
project(rm_behavior_tree)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## 禁用符号链接安装（适用于部署机/NFS等不支持symlink的环境）
## 必须放在 find_package 之前
set(AMENT_CMAKE_SYMLINK_INSTALL 0)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(robot_msgs REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

set(PACKAGE_INCLUDE_DEPENDS
    rm_decision_interfaces
    rclcpp
    std_msgs
    behaviortree_cpp
    behaviortree_ros2
    auto_aim_interfaces
    nav2_msgs
    nav_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs
    sp_msgs
)

# 定义宏函数 add_target_dependencies
function(add_target_dependencies target)
    target_include_directories(${target} PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
    ament_target_dependencies(${target} ${PACKAGE_INCLUDE_DEPENDS})
    target_compile_definitions(${target} PRIVATE BT_PLUGIN_EXPORT)
endfunction()

ament_auto_add_executable(rm_behavior_tree 
    src/rm_behavior_tree.cpp
)
# Controllers
ament_auto_add_library(decision_switch SHARED
    plugins/control/decision_switch.cpp
)
add_target_dependencies(decision_switch)

# ament_auto_add_library(resettable_sequence SHARED
#     plugins/control/resettable_sequence.cpp
# )
# add_target_dependencies(resettable_sequence)

# Decorators
ament_auto_add_library(rate_controller SHARED 
    plugins/decorator/rate_controller.cpp
)
add_target_dependencies(rate_controller)

# Conditions
ament_auto_add_library(is_game_time SHARED
    plugins/condition/is_game_time.cpp
)
add_target_dependencies(is_game_time)

ament_auto_add_library(is_chassis_energy_ok SHARED
    plugins/condition/is_chassis_energy_ok.cpp
)
add_target_dependencies(is_chassis_energy_ok)

ament_auto_add_library(is_status_ok SHARED 
    plugins/condition/is_status_ok.cpp
)
add_target_dependencies(is_status_ok)

ament_auto_add_library(is_remote_status SHARED 
    plugins/condition/is_remote_status.cpp
)
add_target_dependencies(is_remote_status)

ament_auto_add_library(is_auto_aim_ok SHARED 
    plugins/condition/is_auto_aim_ok.cpp
)
add_target_dependencies(is_auto_aim_ok)

ament_auto_add_library(is_attacked SHARED 
    plugins/condition/is_attacked.cpp
)
add_target_dependencies(is_attacked)

ament_auto_add_library(is_friend_ok SHARED 
    plugins/condition/is_friend_ok.cpp
)
add_target_dependencies(is_friend_ok)

ament_auto_add_library(is_detect_enemy SHARED 
    plugins/condition/is_detect_enemy.cpp
)
add_target_dependencies(is_detect_enemy)

ament_auto_add_library(is_outpost_ok SHARED 
    plugins/condition/is_outpost_ok.cpp
)
add_target_dependencies(is_outpost_ok)

ament_auto_add_library(is_enemy_outpost_ok SHARED 
    plugins/condition/is_enemy_outpost_ok.cpp
)
add_target_dependencies(is_enemy_outpost_ok)

ament_auto_add_library(is_base_attacked SHARED
    plugins/condition/is_base_attacked.cpp
)
add_target_dependencies(is_base_attacked)

ament_auto_add_library(is_base_status_ok SHARED
    plugins/condition/is_base_status_ok.cpp
)
add_target_dependencies(is_base_status_ok)

ament_auto_add_library(is_time_to_bullet SHARED 
    plugins/condition/is_time_to_bullet.cpp
)
add_target_dependencies(is_time_to_bullet)

ament_auto_add_library(is_rfid_detected SHARED 
    plugins/condition/is_rfid_detected.cpp
)
add_target_dependencies(is_rfid_detected)

ament_auto_add_library(is_gimbal_target_valid SHARED 
    plugins/condition/is_gimbal_target_valid.cpp
)
add_target_dependencies(is_gimbal_target_valid)

ament_auto_add_library(is_target_in_zone SHARED
    plugins/condition/is_target_in_zone.cpp
)
add_target_dependencies(is_target_in_zone)

ament_auto_add_library(is_gimbal_target_in_zone SHARED
    plugins/condition/is_gimbal_target_in_zone.cpp
)
add_target_dependencies(is_gimbal_target_in_zone)

# Custom condition nodes
ament_auto_add_library(not_condition SHARED
    plugins/condition/not_condition.cpp
)
add_target_dependencies(not_condition)

ament_auto_add_library(get_blackboard SHARED
    plugins/condition/get_blackboard.cpp
)
add_target_dependencies(get_blackboard)

# Actions
ament_auto_add_library(send_goal SHARED
    plugins/action/send_goal.cpp
)
add_target_dependencies(send_goal)

ament_auto_add_library(sub_all_robot_hp SHARED 
    plugins/action/sub_all_robot_hp.cpp
)
add_target_dependencies(sub_all_robot_hp)

ament_auto_add_library(sub_robot_status SHARED 
    plugins/action/sub_robot_status.cpp
)
add_target_dependencies(sub_robot_status)

ament_auto_add_library(sub_remote_status SHARED
    plugins/action/sub_remote_status.cpp
)
add_target_dependencies(sub_remote_status)

ament_auto_add_library(sub_auto_aim_status SHARED
    plugins/action/sub_auto_aim_status.cpp
)
add_target_dependencies(sub_auto_aim_status)

ament_auto_add_library(sub_target_pos SHARED 
    plugins/action/sub_target_pos.cpp
)
add_target_dependencies(sub_target_pos)

ament_auto_add_library(sub_gimbal_target SHARED 
    plugins/action/sub_gimbal_target.cpp
)
add_target_dependencies(sub_gimbal_target)

ament_auto_add_library(sub_game_status SHARED 
    plugins/action/sub_game_status.cpp
)
add_target_dependencies(sub_game_status)

ament_auto_add_library(sub_chassis_energy SHARED
    plugins/action/sub_chassis_energy.cpp
)
add_target_dependencies(sub_chassis_energy)

ament_auto_add_library(sub_rune_status SHARED
    plugins/action/sub_rune_status.cpp
)
add_target_dependencies(sub_rune_status)

ament_auto_add_library(is_rune_status SHARED
    plugins/condition/is_rune_status.cpp
)
add_target_dependencies(is_rune_status)

# ament_auto_add_library(sub_armors SHARED 
#     plugins/action/sub_armors.cpp
# )
# add_target_dependencies(sub_armors)

ament_auto_add_library(sub_decision_num SHARED
    plugins/action/sub_decision_num.cpp
)
add_target_dependencies(sub_decision_num)

ament_auto_add_library(get_current_location SHARED 
    plugins/action/get_current_location.cpp
)
add_target_dependencies(get_current_location)
ament_auto_add_library(rm_set_blackboard SHARED
    plugins/action/set_blackboard.cpp
)
add_target_dependencies(rm_set_blackboard)

ament_auto_add_library(move_around SHARED
    plugins/action/move_around.cpp
)
add_target_dependencies(move_around)

ament_auto_add_library(pursue_enemy SHARED
    plugins/action/pursue_enemy.cpp
    src/map_accessor.cpp
    src/key_point_loader.cpp
    src/visibility_checker.cpp
)
add_target_dependencies(pursue_enemy)

ament_auto_add_library(pub_auto_aim_mode SHARED 
    plugins/action/pub_auto_aim_mode.cpp
)
add_target_dependencies(pub_auto_aim_mode)

ament_auto_add_library(pub_chassis_mode SHARED
    plugins/action/pub_chassis_mode.cpp
)
add_target_dependencies(pub_chassis_mode)

ament_auto_add_library(pub_super_capacitor SHARED
    plugins/action/pub_super_capacitor.cpp
)
add_target_dependencies(pub_super_capacitor)

ament_auto_add_library(robot_control SHARED 
    plugins/action/robot_control.cpp
)
add_target_dependencies(robot_control)

ament_auto_add_library(pub_robot_state SHARED 
    plugins/action/pub_robot_state.cpp
)
add_target_dependencies(pub_robot_state)

ament_auto_add_library(pub_enemy_status SHARED 
    plugins/action/pub_enemy_status.cpp
)
add_target_dependencies(pub_enemy_status)

ament_auto_add_library(pub_autoaim_target SHARED 
    plugins/action/pub_autoaim_target.cpp
)
add_target_dependencies(pub_autoaim_target)

ament_auto_add_library(wait_duration SHARED
    plugins/action/wait_duration.cpp
)
add_target_dependencies(wait_duration)

ament_auto_add_library(print_message SHARED 
    plugins/action/print_message.cpp
)
add_target_dependencies(print_message)

# Graph building libraries
ament_auto_add_library(key_point_loader SHARED
    src/key_point_loader.cpp
)
add_target_dependencies(key_point_loader)

ament_auto_add_library(map_accessor SHARED
    src/map_accessor.cpp
)
add_target_dependencies(map_accessor)

ament_auto_add_library(visibility_checker SHARED
    src/visibility_checker.cpp
)
add_target_dependencies(visibility_checker)
target_link_libraries(visibility_checker map_accessor key_point_loader)

ament_auto_add_library(graph_builder SHARED
    src/graph_builder.cpp
)
add_target_dependencies(graph_builder)
target_link_libraries(graph_builder visibility_checker map_accessor key_point_loader)

ament_auto_add_library(graph_visualizer SHARED
    src/graph_visualizer.cpp
)
add_target_dependencies(graph_visualizer)
target_link_libraries(graph_visualizer graph_builder visibility_checker map_accessor key_point_loader)

# Test executables
ament_auto_add_executable(test_key_point_loader
    src/test_key_point_loader.cpp
)
add_target_dependencies(test_key_point_loader)
target_link_libraries(test_key_point_loader key_point_loader)

ament_auto_add_executable(test_graph_builder
    src/test_graph_builder.cpp
)
add_target_dependencies(test_graph_builder)
target_link_libraries(test_graph_builder graph_visualizer graph_builder visibility_checker map_accessor key_point_loader)

# Graph planner node (request/response via Int32MultiArray)
ament_auto_add_executable(graph_planner_node
    src/graph_planner_node.cpp
)
add_target_dependencies(graph_planner_node)
target_link_libraries(graph_planner_node graph_builder visibility_checker map_accessor key_point_loader)

# Enemy state estimator node
ament_auto_add_executable(enemy_state_node
    src/enemy_state_node.cpp
)
add_target_dependencies(enemy_state_node)
target_link_libraries(enemy_state_node visibility_checker map_accessor key_point_loader)
 
# Intercept node
ament_auto_add_executable(intercept_node
    src/intercept_node.cpp
)
add_target_dependencies(intercept_node)
target_link_libraries(intercept_node graph_builder visibility_checker map_accessor key_point_loader)

# ---- Install Python scripts ----
# libexec 是 launch 文件中 executable= 查找可执行文件的正确路径
# install(PROGRAMS
#     scripts/pursue_enemy_mock_publisher.py
#   DESTINATION libexec/${PROJECT_NAME}
# )

# ---- ament_auto_package ----
ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)