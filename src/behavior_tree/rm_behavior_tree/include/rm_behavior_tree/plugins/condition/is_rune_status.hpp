#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_RUNE_STATUS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_RUNE_STATUS_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "robot_msgs/msg/rune_status.hpp"

namespace rm_behavior_tree
{

/**
 * @brief BT Condition节点：判断电控下行的 rune_count 是否等于 target_count。
 *
 * 行为：
 *  - 读不到 blackboard 上的 RuneStatus 消息 → 返回 FAILURE（参考 IsEnemyOutpostOK 的做法）
 *  - rune_count == target_count → 返回 FAILURE（命中，打符完成）
 *  - 否则 → 返回 SUCCESS
 */
class IsRuneStatusAction : public BT::ConditionNode
{
public:
  IsRuneStatusAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<robot_msgs::msg::RuneStatus>("rune_status"),
      BT::InputPort<int>("target_count", 1, "触发的目标 rune_count 值（默认 1）")};
  }

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_RUNE_STATUS_HPP_