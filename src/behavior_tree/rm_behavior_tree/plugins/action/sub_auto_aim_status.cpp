#include "rm_behavior_tree/plugins/action/sub_auto_aim_status.hpp"

namespace rm_behavior_tree
{

SubAutoAimStatusAction::SubAutoAimStatusAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<robot_msgs::msg::AutoAimStatus>(name, conf, params)
{
}

BT::NodeStatus SubAutoAimStatusAction::onTick(
  const std::shared_ptr<robot_msgs::msg::AutoAimStatus> & last_msg)
{
  if (last_msg)
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new auto_aim_status message, status: %d",
      name().c_str(), last_msg->auto_aim_status);

    setOutput("auto_aim_status", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubAutoAimStatusAction, "SubAutoAimStatus");

