#include "rm_behavior_tree/plugins/action/sub_rune_status.hpp"

namespace rm_behavior_tree
{

SubRuneStatusAction::SubRuneStatusAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<robot_msgs::msg::RuneStatus>(name, conf, params)
{
}

BT::NodeStatus SubRuneStatusAction::onTick(
  const std::shared_ptr<robot_msgs::msg::RuneStatus> & last_msg)
{
  if (last_msg)
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new message, rune_count: %s", name().c_str(),
      std::to_string(last_msg->rune_count).c_str());
    setOutput("rune_status", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubRuneStatusAction, "SubRuneStatus");