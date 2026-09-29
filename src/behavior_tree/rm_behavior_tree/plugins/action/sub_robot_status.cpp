#include "rm_behavior_tree/plugins/action/sub_robot_status.hpp"

namespace rm_behavior_tree
{

SubRobotStatusAction::SubRobotStatusAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<robot_msgs::msg::RobotStatus>(name, conf, params)
{
}

BT::NodeStatus SubRobotStatusAction::onTick(
  const std::shared_ptr<robot_msgs::msg::RobotStatus> & last_msg)
{
  if (last_msg)  // empty if no new message received, since the last tick
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new message, current_hp: %s, shooter_heat: %s, shooter_barrel_heat_limit: %s", 
      name().c_str(),
      std::to_string(last_msg->current_hp).c_str(),
      std::to_string(last_msg->shooter_heat).c_str(),
      std::to_string(last_msg->shooter_barrel_heat_limit).c_str());
    setOutput("robot_status", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubRobotStatusAction, "SubRobotStatus");