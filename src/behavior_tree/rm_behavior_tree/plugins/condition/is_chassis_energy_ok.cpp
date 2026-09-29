#include "rm_behavior_tree/plugins/condition/is_chassis_energy_ok.hpp"
#include <iostream>

namespace rm_behavior_tree
{

IsChassisEnergyOKCondition::IsChassisEnergyOKCondition(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::ConditionNode(name, config)
{
  node_ = params.nh;
  sub_ = node_->create_subscription<robot_msgs::msg::ChassisEnergy>(
    "/chassis_energy",
    10,
    [this](const robot_msgs::msg::ChassisEnergy::SharedPtr msg)
    {
      last_msg_ = *msg;
      has_msg_ = true;
    });
}

BT::NodeStatus IsChassisEnergyOKCondition::tick()
{
  robot_msgs::msg::ChassisEnergy msg;

  if (!getInput("message", msg))
  {
    if (!has_msg_)
    {
      return BT::NodeStatus::FAILURE;
    }
    msg = last_msg_;
  }

  std::cout << "[IsChassisEnergyOK] chassis_energy=" << msg.chassis_energy << std::endl;

  // 下位机发 0: 正常底盘能量 -> FAILURE
  // 下位机发 1: 底盘能量不足 -> SUCCESS
  bool energy_insufficient = (msg.chassis_energy == 1);

  if (energy_insufficient)
  {
    std::cout << "[IsChassisEnergyOK] returning SUCCESS (energy insufficient)" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    std::cout << "[IsChassisEnergyOK] returning FAILURE (energy OK)" << std::endl;
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::IsChassisEnergyOKCondition, "IsChassisEnergyOK");