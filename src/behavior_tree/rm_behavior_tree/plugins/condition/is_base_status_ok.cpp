#include "rm_behavior_tree/plugins/condition/is_base_status_ok.hpp"

namespace rm_behavior_tree
{

IsBaseStatusOKAction::IsBaseStatusOKAction(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::ConditionNode(name, config)
{
  (void)params;
}

BT::NodeStatus IsBaseStatusOKAction::tick()
{
  robot_msgs::msg::AllRobotHP msg;

  if (!getInput("robot_hp", msg))
  {
    return BT::NodeStatus::SUCCESS;
  }

  int hp_threshold = 0;
  getInput("hp_threshold", hp_threshold);

  if (msg.base_hp >= hp_threshold)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::IsBaseStatusOKAction, "IsBaseStatusOK");
