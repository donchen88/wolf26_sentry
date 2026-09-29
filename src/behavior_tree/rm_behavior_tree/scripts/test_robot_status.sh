#!/bin/bash
# 测试 robot_status 消息发布的脚本

# 根据修改后的 is_status_ok.cpp 逻辑：
# 要让节点返回 SUCCESS，需要满足：
# - current_hp >= hp_threshold (血量大于等于阈值)
# - shooter_heat >= heat_threshold (热量大于等于阈值)
# - shooter_barrel_heat_limit <= barrel_heat_limit_threshold (热量上限小于等于阈值，即小于阈值时返回成功)

# 第一个 Sequence 的条件：hp_threshold=50, heat_threshold=350, barrel_heat_limit_threshold=300
# 第二个 Sequence 的条件：hp_threshold=100, heat_threshold=350, barrel_heat_limit_threshold=300

# 方法1：只发布三个必需字段（ROS2会自动给其他字段赋默认值）
# 单次发布（用于测试）
ros2 topic pub /robot_status robot_msgs/msg/RobotStatus \
  "{current_hp: 50, shooter_barrel_heat_limit: 250, shooter_heat: 400}" \
  --once

# 方法2：循环发布（每秒一次，用于持续测试）
# ros2 topic pub /robot_status robot_msgs/msg/RobotStatus \
#   "{current_hp: 50, shooter_barrel_heat_limit: 250, shooter_heat: 400}" \
#   -r 1

# 注意：ROS2会自动给未指定的字段赋默认值：
# - robot_id: 0 (uint8默认值)
# - team_color: false (bool默认值)
# - is_attacked: false (bool默认值)

