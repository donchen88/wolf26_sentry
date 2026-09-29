#include "rm_behavior_tree/plugins/action/pub_auto_aim_mode.hpp"

namespace rm_behavior_tree
{

PubAutoAimModeAction::PubAutoAimModeAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<robot_msgs::msg::AutoAimMode>(name, conf, params)
{
}

bool PubAutoAimModeAction::setMessage(robot_msgs::msg::AutoAimMode & msg)
{
  // 先尝试读取 auto_aim_mode（XML 配置的实际值）
  auto mode_in = getInput<int>("auto_aim_mode");
  // 如果没有，尝试读取 key_name（兼容旧版本）
  if (!mode_in) {
    mode_in = getInput<int>("key_name");
  }
  if (!mode_in)
  {
    RCLCPP_ERROR(node_->get_logger(), "Missing input port [key_name] or [auto_aim_mode]");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "[PubAutoAimMode] Publishing mode=%d", *mode_in);
  msg.mode = static_cast<uint8_t>(*mode_in);
  return true;
}

BT::PortsList PubAutoAimModeAction::providedPorts()
{
  BT::PortsList ports = {
    BT::InputPort<int>(
      "key_name",
      0,
      "0=默认瞄敌人, 1=瞄小符, 2=瞄大符, 3=瞄前哨"),
    BT::InputPort<int>(
      "auto_aim_mode",
      0,
      "兼容旧版本参数名")
  };

  return providedBasicPorts(ports);
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
// 注册为 Groot2 可用插件
CreateRosNodePlugin(rm_behavior_tree::PubAutoAimModeAction, "PubAutoAimMode");

