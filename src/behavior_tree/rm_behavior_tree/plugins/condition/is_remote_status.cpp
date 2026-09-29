#include "rm_behavior_tree/plugins/condition/is_remote_status.hpp"

namespace rm_behavior_tree
{

IsRemoteStatusCondition::IsRemoteStatusCondition(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::ConditionNode(name, config),
  node_(params.nh)
{
  // 订阅 /remote_status
  sub_ = node_->create_subscription<robot_msgs::msg::RemoteStatus>(
    "/remote_status", 10,
    [this](const robot_msgs::msg::RemoteStatus::SharedPtr msg)
    {
      last_msg_ = *msg;
      has_msg_ = true;
    });
}

BT::NodeStatus IsRemoteStatusCondition::tick()
{
  int target_status;
  getInput("target_status", target_status);

  // XML message 优先
  auto msg = getInput<robot_msgs::msg::RemoteStatus>("message");
  if (!msg && !has_msg_)
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "IsRemoteStatus: waiting for /remote_status ...");
    return BT::NodeStatus::FAILURE;
  }

  const auto & remote_msg = msg ? *msg : last_msg_;

  if (remote_msg.remote_control_status == target_status)
  {
    RCLCPP_DEBUG(
      node_->get_logger(), "Remote status matched: %d == %d",
      remote_msg.remote_control_status, target_status);
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_DEBUG(
      node_->get_logger(), "Remote status not match: %d != %d",
      remote_msg.remote_control_status, target_status);
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::IsRemoteStatusCondition, "IsRemoteStatus");
