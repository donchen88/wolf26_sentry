#include "rm_behavior_tree/plugins/condition/is_enemy_outpost_ok.hpp"

namespace rm_behavior_tree
{

IsEnemyOutpostOKAction::IsEnemyOutpostOKAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::ConditionNode(name, config),
  node_(params.nh)
{
}

BT::NodeStatus IsEnemyOutpostOKAction::tick()
{
  int hp_threshold = 0;
  auto robot_hp_msg = getInput<robot_msgs::msg::AllRobotHP>("robot_hp");
  getInput("hp_threshold", hp_threshold);

  if (!robot_hp_msg) {
    return BT::NodeStatus::FAILURE;
  }

  uint16_t enemy_outpost_hp =
    robot_hp_msg->team_color ? robot_hp_msg->blue_outpost_hp : robot_hp_msg->red_outpost_hp;
  return (enemy_outpost_hp > hp_threshold) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::IsEnemyOutpostOKAction, "IsEnemyOutpostOK");

