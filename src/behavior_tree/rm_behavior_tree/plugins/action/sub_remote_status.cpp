#include "rm_behavior_tree/plugins/action/sub_remote_status.hpp"

namespace rm_behavior_tree
{

SubRemoteStatusAction::SubRemoteStatusAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<robot_msgs::msg::RemoteStatus>(name, conf, params)
{
}

BT::NodeStatus SubRemoteStatusAction::onTick(
  const std::shared_ptr<robot_msgs::msg::RemoteStatus> & last_msg)
{
  if (last_msg)
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new remote_status message, status: %d",
      name().c_str(), last_msg->remote_control_status);

    setOutput("remote_status", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubRemoteStatusAction, "SubRemoteStatus");


