#include "rm_behavior_tree/plugins/condition/is_detect_enemy.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <atomic>

namespace rm_behavior_tree
{
static std::atomic<int> g_is_detect_enemy_tick(0);

IsDetectEnemyAction::IsDetectEnemyAction(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsDetectEnemyAction::detectEnemyStatus, this), config)
{
}

BT::NodeStatus IsDetectEnemyAction::detectEnemyStatus()
{
  auto msg = getInput<auto_aim_interfaces::msg::Armors>("message");

  if (!msg) {
    std::cerr << "Missing required input [message]" << '\n';
    return BT::NodeStatus::FAILURE;
  }

  if (msg->armors.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  // 检查是否有有效的装甲板（非零位置）
  bool has_valid_armor = false;
  for (const auto & armor : msg->armors) {
    if (std::abs(armor.pose.position.x) > 0.001 ||
        std::abs(armor.pose.position.y) > 0.001 ||
        std::abs(armor.pose.position.z) > 0.001) {
      has_valid_armor = true;
      break;
    }
  }

  if (!has_valid_armor) {
    RCLCPP_INFO(rclcpp::get_logger("is_detect_enemy"), "IsDetectEnemy: No valid armor (all zeros), returning FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  // 有效装甲板检测成功，不打印高频日志

  // 获取最大范围参数（可选，默认-1表示不限制）
  double max_range = -1.0;
  getInput("max_range", max_range);

  // 如果设置了最大范围限制，检查是否有敌人在范围内
  if (max_range > 0.0) {
    // 尝试获取机器人当前位置（用于计算距离）
    auto robot_location = getInput<geometry_msgs::msg::TransformStamped>("current_location");
    geometry_msgs::msg::Point robot_pos;
    bool has_robot_pos = false;
    
    if (robot_location) {
      robot_pos.x = robot_location->transform.translation.x;
      robot_pos.y = robot_location->transform.translation.y;
      robot_pos.z = robot_location->transform.translation.z;
      has_robot_pos = true;
    }
    
    for (const auto & armor : msg->armors) {
      double distance;
      
      if (has_robot_pos) {
        // 计算敌人到机器人的距离（更准确）
        double dx = armor.pose.position.x - robot_pos.x;
        double dy = armor.pose.position.y - robot_pos.y;
        double dz = armor.pose.position.z - robot_pos.z;
        distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      } else {
        // 如果没有机器人位置，使用敌人到原点的距离（兼容旧逻辑）
        distance = std::hypot(
        armor.pose.position.x,
        armor.pose.position.y,
        armor.pose.position.z);
      }
      
      if (distance <= max_range) {
        // std::cout << "检测到敌人在范围内，距离: " << distance << "m" << '\n';
        return BT::NodeStatus::SUCCESS;
      }
    }
    // 所有敌人都超出范围
    // std::cout << "检测到敌人但超出范围" << '\n';
    RCLCPP_INFO(rclcpp::get_logger("is_detect_enemy"), "IsDetectEnemy: Armors out of range (max=%.2f), returning FAILURE", max_range);
    return BT::NodeStatus::FAILURE;
  } else {
    // 没有范围限制，只要检测到敌人就返回成功
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsDetectEnemyAction>("IsDetectEnemy");
}