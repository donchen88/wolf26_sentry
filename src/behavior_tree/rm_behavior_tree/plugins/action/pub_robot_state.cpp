#include "rm_behavior_tree/plugins/action/pub_robot_state.hpp"

namespace rm_behavior_tree
{

PubRobotStateAction::PubRobotStateAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<robot_msgs::msg::RobotState>(name, conf, params)
{
}

// 从行为树端口读取 state 值并封装消息
bool PubRobotStateAction::setMessage(robot_msgs::msg::RobotState & msg)
{
  auto state = getInput<int>("robot_state");
  if (!state)
  {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input port [robot_state]");
    return false;
  }

  msg.state = *state;
  return true;
}

// 行为树端口定义
BT::PortsList PubRobotStateAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<int>(
      "robot_state", 1,
      "机器人状态发布：1=移动姿态，2=防御姿态，3=进攻姿态")
  };

  return providedBasicPorts(additional_ports);
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
// 注册插件以供 Groot2 使用
CreateRosNodePlugin(rm_behavior_tree::PubRobotStateAction, "PubRobotState");

