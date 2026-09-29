#include "rm_behavior_tree/plugins/condition/is_rune_status.hpp"

namespace rm_behavior_tree
{

IsRuneStatusAction::IsRuneStatusAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::ConditionNode(name, config),
  node_(params.nh)
{
}

BT::NodeStatus IsRuneStatusAction::tick()
{
  int target_count = 1;
  getInput("target_count", target_count);

  auto rune_status_msg = getInput<robot_msgs::msg::RuneStatus>("rune_status");
  if (!rune_status_msg) {
    return BT::NodeStatus::FAILURE;
  }

  return (static_cast<int>(rune_status_msg->rune_count) == target_count)
           ? BT::NodeStatus::FAILURE
           : BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::IsRuneStatusAction, "IsRuneStatus");