#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_REMOTE_STATUS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_REMOTE_STATUS_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "robot_msgs/msg/remote_status.hpp"
#include <rclcpp/rclcpp.hpp>

namespace rm_behavior_tree
{
/**
 * @brief Condition节点，用于检查遥控器状态是否等于目标状态
 * 
 * 该节点从输入端口获取遥控器状态消息，或订阅 /remote_status 话题，
 * 比较其中的 remote_control_status 与 target_status。
 * 若相等返回 SUCCESS，否则返回 FAILURE。
 * 
 * @param[in] message 遥控器状态消息（可选，优先使用）
 * @param[in] target_status 目标遥控器模式：1=正常，2=保守，3=激进
 */
class IsRemoteStatusCondition : public BT::ConditionNode
{
public:
  IsRemoteStatusCondition(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<robot_msgs::msg::RemoteStatus>("message"),
      BT::InputPort<int>("target_status", 1, "Target remote control mode (1=Normal, 2=Conservative, 3=Aggressive)")};
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<robot_msgs::msg::RemoteStatus>::SharedPtr sub_;
  robot_msgs::msg::RemoteStatus last_msg_;
  bool has_msg_{false};
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_REMOTE_STATUS_HPP_
