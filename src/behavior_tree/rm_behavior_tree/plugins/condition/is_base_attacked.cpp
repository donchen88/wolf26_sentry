#include "rm_behavior_tree/plugins/condition/is_base_attacked.hpp"

namespace rm_behavior_tree
{

IsBaseAttackedAction::IsBaseAttackedAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::ConditionNode(name, config), node_(params.nh)
{
}

BT::NodeStatus IsBaseAttackedAction::tick()
{
  auto msg = getInput<robot_msgs::msg::AllRobotHP>("all_robot_hp");

  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  int hp_threshold = 10;
  getInput("hp_threshold", hp_threshold);

  int timeout_msec = 10000;
  getInput("timeout_msec", timeout_msec);

  uint16_t current_base_hp = msg->base_hp;
  rclcpp::Time now = node_->now();

  if (!is_initialized_) {
    last_base_hp_ = current_base_hp;
    last_hp_change_time_ = now;
    is_initialized_ = true;
    return BT::NodeStatus::FAILURE;
  }

  int16_t hp_drop = static_cast<int16_t>(last_base_hp_) - static_cast<int16_t>(current_base_hp);

  if (hp_drop >= hp_threshold) {
    last_base_hp_ = current_base_hp;
    last_hp_change_time_ = now;
    return BT::NodeStatus::SUCCESS;
  }

  last_base_hp_ = current_base_hp;

  double elapsed_sec = (now - last_hp_change_time_).seconds();
  if (elapsed_sec * 1000.0 < timeout_msec) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::IsBaseAttackedAction, "IsBaseAttacked");
