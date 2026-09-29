#include "rm_behavior_tree/plugins/action/sub_game_status.hpp"

namespace rm_behavior_tree
{

SubGameStatusAction::SubGameStatusAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<robot_msgs::msg::GameStatus>(name, conf, params)
{
}

BT::NodeStatus SubGameStatusAction::onTick(
  const std::shared_ptr<robot_msgs::msg::GameStatus> & last_msg)
{
  if (last_msg)  // empty if no new message received, since the last tick
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new message, game_progress: %s", name().c_str(),
      std::to_string(last_msg->game_progress).c_str());
    setOutput("game_status", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubGameStatusAction, "SubGameStatus");