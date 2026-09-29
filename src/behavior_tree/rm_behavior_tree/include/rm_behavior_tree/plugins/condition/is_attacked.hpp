#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_ATTACKED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_ATTACKED_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "robot_msgs/msg/robot_status.hpp"
#include <rclcpp/rclcpp.hpp>

namespace rm_behavior_tree
{

/**
 * @brief condition节点，用于判断机器人是否被攻击掉血
 * 通过检测血量下降来判断是否被击打
 * @param[in] message 机器人状态消息
 * @param[in] hp_threshold 血量下降阈值，超过此值才判定为被攻击（默认10）
 * @param[in] timeout_msec 被击打后保持SUCCESS的时间（默认10000ms）
 */
class IsAttackedAction : public BT::ConditionNode
{
public:
  IsAttackedAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<robot_msgs::msg::RobotStatus>("message"),
      BT::InputPort<uint16_t>("hp_threshold", 10, "HP drop threshold for attacked detection"),
      BT::InputPort<int>("timeout_msec", 10000, "Time in ms to keep SUCCESS after last HP drop (default 10000)")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  uint16_t last_hp_{0};
  bool is_initialized_{false};
  rclcpp::Time last_hp_change_time_{0, 0, RCL_CLOCK_UNINITIALIZED};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_ATTACKED_HPP_
