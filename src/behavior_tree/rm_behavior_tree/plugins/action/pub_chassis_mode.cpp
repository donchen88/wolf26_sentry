#include "rm_behavior_tree/plugins/action/pub_chassis_mode.hpp"

namespace rm_behavior_tree
{

PubChassisModeAction::PubChassisModeAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<robot_msgs::msg::ChassisMode>(name, conf, params)
{
}

bool PubChassisModeAction::setMessage(robot_msgs::msg::ChassisMode & msg)
{
  auto mode_in = getInput<int>("chassis_mode");
  if (!mode_in)
  {
    RCLCPP_ERROR(node_->get_logger(), "Missing input port [chassis_mode]");
    return false;
  }

  msg.mode = static_cast<uint8_t>(*mode_in);
  return true;
}

BT::PortsList PubChassisModeAction::providedPorts()
{
  BT::PortsList ports = {
    BT::InputPort<int>(
      "chassis_mode",
      0,
      "0=正常底盘速度, 1=加速底盘旋转, 2=过起伏路段")
  };

  return providedBasicPorts(ports);
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
// 注册为 Groot2 可用插件
CreateRosNodePlugin(rm_behavior_tree::PubChassisModeAction, "PubChassisMode");

