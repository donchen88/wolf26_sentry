#include "rm_behavior_tree/plugins/condition/is_auto_aim_ok.hpp"

namespace rm_behavior_tree
{

IsAutoAimOKCondition::IsAutoAimOKCondition(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::ConditionNode(name, config),
  node_(params.nh)
{
  // 订阅 /auto_aim_status
  sub_ = node_->create_subscription<robot_msgs::msg::AutoAimStatus>(
    "/auto_aim_status", 10,
    [this](const robot_msgs::msg::AutoAimStatus::SharedPtr msg)
    {
      last_msg_ = *msg;
      has_msg_ = true;
    });
}

BT::NodeStatus IsAutoAimOKCondition::tick()
{
  int target_status;
  getInput("target_status", target_status);

  // XML message 优先
  auto msg = getInput<robot_msgs::msg::AutoAimStatus>("message");
  if (!msg && !has_msg_)
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "IsAutoAimOK: waiting for /auto_aim_status ...");
    return BT::NodeStatus::FAILURE;
  }

  const auto & auto_aim_msg = msg ? *msg : last_msg_;

  if (auto_aim_msg.auto_aim_status == target_status)
  {
    RCLCPP_DEBUG(
      node_->get_logger(), "Auto aim status matched: %d == %d",
      auto_aim_msg.auto_aim_status, target_status);
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_DEBUG(
      node_->get_logger(), "Auto aim status not match: %d != %d",
      auto_aim_msg.auto_aim_status, target_status);
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::IsAutoAimOKCondition, "IsAutoAimOK");

