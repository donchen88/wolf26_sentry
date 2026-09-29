#include "rm_behavior_tree/plugins/action/sub_all_robot_hp.hpp"

namespace rm_behavior_tree
{

SubAllRobotHPAction::SubAllRobotHPAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<robot_msgs::msg::AllRobotHP>(name, conf, params)
{
}

BT::NodeStatus SubAllRobotHPAction::onTick(
  const std::shared_ptr<robot_msgs::msg::AllRobotHP> & last_msg)
{
  if (last_msg)  // empty if no new message received, since the last tick
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new message, red_1_robot_hp: %s", name().c_str(),
      std::to_string(last_msg->red_1_robot_hp).c_str());
    setOutput("robot_hp", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubAllRobotHPAction, "SubAllRobotHP");