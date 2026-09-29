#include "rm_behavior_tree/plugins/condition/is_attacked.hpp"

namespace rm_behavior_tree
{

IsAttackedAction::IsAttackedAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::ConditionNode(name, config), node_(params.nh)
{
}

BT::NodeStatus IsAttackedAction::tick()
{
  auto msg = getInput<robot_msgs::msg::RobotStatus>("message");

  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  uint16_t hp_threshold = 10;
  getInput("hp_threshold", hp_threshold);

  int timeout_msec = 10000;
  getInput("timeout_msec", timeout_msec);

  uint16_t current_hp = msg->current_hp;
  rclcpp::Time now = node_->now();

  if (!is_initialized_) {
    last_hp_ = current_hp;
    last_hp_change_time_ = now;
    is_initialized_ = true;
    return BT::NodeStatus::FAILURE;
  }

  int16_t hp_drop = static_cast<int16_t>(last_hp_) - static_cast<int16_t>(current_hp);

  if (hp_drop >= static_cast<int16_t>(hp_threshold)) {
    // 血量下降超过阈值 → 判定为被攻击，更新状态
    last_hp_ = current_hp;
    last_hp_change_time_ = now;
    return BT::NodeStatus::SUCCESS;
  }

  // 血量没下降超过阈值，检查是否超时
  double elapsed_sec = (now - last_hp_change_time_).seconds();
  if (elapsed_sec * 1000.0 < timeout_msec) {
    // 超时时间内没检测到新的攻击，返回 FAILURE
    return BT::NodeStatus::FAILURE;
  }

  // 只有血量恢复（上升）时才更新 last_hp_，让下次判断更准确
  if (current_hp > last_hp_) {
    last_hp_ = current_hp;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::IsAttackedAction, "IsAttacked");