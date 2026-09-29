#include "rm_behavior_tree/plugins/condition/is_outpost_ok.hpp"

namespace rm_behavior_tree
{

IsOutpostOKAction::IsOutpostOKAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::ConditionNode(name, config),
  node_(params.nh)
{
}

BT::NodeStatus IsOutpostOKAction::tick()
{
  int hp_threshold = 0;
  auto robot_hp_msg = getInput<robot_msgs::msg::AllRobotHP>("robot_hp");
  getInput("hp_threshold", hp_threshold);

  if (!robot_hp_msg) {
    return BT::NodeStatus::FAILURE;
  }

  // 前哨站血量大于阈值返回 SUCCESS，否则返回 FAILURE
  uint16_t outpost_hp = robot_hp_msg->outpost_hp;
  return (outpost_hp > hp_threshold) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::IsOutpostOKAction, "IsOutpostOK");
