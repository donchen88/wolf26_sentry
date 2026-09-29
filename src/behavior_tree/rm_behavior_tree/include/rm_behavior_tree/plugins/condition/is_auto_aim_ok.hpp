#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_AUTO_AIM_OK_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_AUTO_AIM_OK_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "robot_msgs/msg/auto_aim_status.hpp"
#include <rclcpp/rclcpp.hpp>

namespace rm_behavior_tree
{
/**
 * @brief Condition节点，用于检查自瞄状态是否等于目标状态
 *
 * 该节点从输入端口获取自瞄状态消息，或订阅 /auto_aim_status 话题，
 * 比较其中的 auto_aim_status 与 target_status。
 * 若相等返回 SUCCESS，否则返回 FAILURE。
 *
 * @param[in] message 自瞄状态消息（可选，优先使用）
 * @param[in] target_status 目标自瞄状态：0=正常，1=不正常（默认值为0）
 */
class IsAutoAimOKCondition : public BT::ConditionNode
{
public:
  IsAutoAimOKCondition(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<robot_msgs::msg::AutoAimStatus>("message"),
      BT::InputPort<int>("target_status", 0, "目标自瞄状态：0=正常，1=不正常")};
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<robot_msgs::msg::AutoAimStatus>::SharedPtr sub_;
  robot_msgs::msg::AutoAimStatus last_msg_;
  bool has_msg_{false};
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_AUTO_AIM_OK_HPP_

