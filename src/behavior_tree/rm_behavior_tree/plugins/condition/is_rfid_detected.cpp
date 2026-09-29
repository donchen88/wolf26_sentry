#include "rm_behavior_tree/plugins/condition/is_rfid_detected.hpp"
#include <iostream>

namespace rm_behavior_tree
{

IsRfidDetectedCondition::IsRfidDetectedCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsRfidDetectedCondition::checkRfidStatus, this), config)
{
}

BT::NodeStatus IsRfidDetectedCondition::checkRfidStatus()
{
  bool base_gain_point, friendly_fortress_gain_point, center_gain_point;

  auto msg = getInput<robot_msgs::msg::RfidStatus>("key_port");
  if (!msg) {
    std::cerr << "Missing required input [key_port]: RfidStatus message is not available" << '\n';
    return BT::NodeStatus::FAILURE;
  }

  getInput("base_gain_point", base_gain_point);
  getInput("friendly_fortress_gain_point", friendly_fortress_gain_point);
  getInput("center_gain_point", center_gain_point);

  if (
    (base_gain_point && msg->base_gain_point) ||
    (friendly_fortress_gain_point && msg->friendly_fortress_gain_point) ||
    (center_gain_point && msg->center_gain_point)) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

BT::PortsList IsRfidDetectedCondition::providedPorts()
{
  return {
    BT::InputPort<robot_msgs::msg::RfidStatus>(
      "key_port", "{@referee_rfidStatus}", "RfidStatus port on blackboard"),
    BT::InputPort<bool>("base_gain_point", false, "己方基地增益点"),
    BT::InputPort<bool>("friendly_fortress_gain_point", false, "己方堡垒增益点"),
    BT::InputPort<bool>("center_gain_point", false, "中心增益点（仅 RMUL 适用）"),
  };
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsRfidDetectedCondition>("IsRfidDetected");
}

