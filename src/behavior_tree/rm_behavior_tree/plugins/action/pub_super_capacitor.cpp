#include "rm_behavior_tree/plugins/action/pub_super_capacitor.hpp"

namespace rm_behavior_tree
{

PubSuperCapacitorAction::PubSuperCapacitorAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<robot_msgs::msg::SuperCapacitor>(name, conf, params)
{
}

bool PubSuperCapacitorAction::setMessage(robot_msgs::msg::SuperCapacitor & msg)
{
  auto state_in = getInput<int>("super_capacitor_state");
  if (!state_in)
  {
    RCLCPP_ERROR(node_->get_logger(), "Missing input port [super_capacitor_state]");
    return false;
  }

  msg.state = static_cast<uint8_t>(*state_in);
  return true;
}

BT::PortsList PubSuperCapacitorAction::providedPorts()
{
  BT::PortsList ports = {
    BT::InputPort<int>(
      "super_capacitor_state",
      0,
      "0=关闭超电, 1=开启超电")
  };

  return providedBasicPorts(ports);
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
// 注册为 Groot2 可用插件
CreateRosNodePlugin(rm_behavior_tree::PubSuperCapacitorAction, "PubSuperCapacitor");
