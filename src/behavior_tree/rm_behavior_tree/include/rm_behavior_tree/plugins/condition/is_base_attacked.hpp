#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_BASE_ATTACKED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_BASE_ATTACKED_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "robot_msgs/msg/all_robot_hp.hpp"
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace rm_behavior_tree
{

class IsBaseAttackedAction : public BT::ConditionNode
{
public:
  IsBaseAttackedAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<robot_msgs::msg::AllRobotHP>("all_robot_hp"),
      BT::InputPort<int>("hp_threshold", 10, "HP drop threshold for base attacked detection"),
      BT::InputPort<int>("timeout_msec", 10000, "Time in ms to keep SUCCESS after last HP drop (default 10000)")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  uint16_t last_base_hp_{0};
  bool is_initialized_{false};
  rclcpp::Time last_hp_change_time_{0, 0, RCL_CLOCK_UNINITIALIZED};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_BASE_ATTACKED_HPP_
