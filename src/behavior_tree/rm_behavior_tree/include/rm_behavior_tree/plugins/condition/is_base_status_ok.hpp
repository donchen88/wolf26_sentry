#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_BASE_STATUS_OK_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_BASE_STATUS_OK_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/plugins.hpp"
#include "robot_msgs/msg/all_robot_hp.hpp"

namespace rm_behavior_tree
{

class IsBaseStatusOKAction : public BT::ConditionNode
{
public:
  IsBaseStatusOKAction(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<robot_msgs::msg::AllRobotHP>("robot_hp"),
      BT::InputPort<int>("hp_threshold", 0, "HP threshold for base status")
    };
  }
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_BASE_STATUS_OK_HPP_
