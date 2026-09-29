#include "rm_behavior_tree/plugins/action/sub_chassis_energy.hpp"

namespace rm_behavior_tree
{

SubChassisEnergyAction::SubChassisEnergyAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<robot_msgs::msg::ChassisEnergy>(name, conf, params)
{
}

BT::NodeStatus SubChassisEnergyAction::onTick(
  const std::shared_ptr<robot_msgs::msg::ChassisEnergy> & last_msg)
{
  if (last_msg)
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new message, chassis_energy: %u",
      name().c_str(),
      last_msg->chassis_energy);
    setOutput("chassis_energy", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubChassisEnergyAction, "SubChassisEnergy");