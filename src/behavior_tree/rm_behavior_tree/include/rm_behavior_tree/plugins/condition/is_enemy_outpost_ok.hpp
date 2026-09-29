#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_ENEMY_OUTPOST_OK_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_ENEMY_OUTPOST_OK_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "robot_msgs/msg/all_robot_hp.hpp"
#include <rclcpp/rclcpp.hpp>

namespace rm_behavior_tree
{

class IsEnemyOutpostOKAction : public BT::ConditionNode
{
public:
  IsEnemyOutpostOKAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<robot_msgs::msg::AllRobotHP>("robot_hp"),
      BT::InputPort<int>("hp_threshold", 0, "HP threshold for enemy outpost")};
  }

private:
  rclcpp::Node::SharedPtr node_;
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_ENEMY_OUTPOST_OK_HPP_

