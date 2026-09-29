#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_AROUND_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_AROUND_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace rm_behavior_tree
{

/**
 * @brief 获取当前位置后小范围移动，躲避攻击
 *        以机器人当前位置为圆心，期望距离为半径的圆内随机生成随机点位
 *        使用导航action，等待每个点到达后再发送下一个点
 * @param[in] message 机器人位置信息
 * @param[in] expected_nearby_goal_count 附近随机点位数量
 * @param[in] expected_dis 期望的移动距离（半径）
 * @param[in] action_name 导航action名称，默认为"navigate_to_pose"
 */
class MoveAroundAction : public BT::StatefulActionNode
{
public:
  MoveAroundAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("expected_nearby_goal_count"), 
      BT::InputPort<float>("expected_dis"),
      BT::InputPort<geometry_msgs::msg::TransformStamped>("message"),
      BT::InputPort<std::string>("action_name", "navigate_to_pose", "Navigation action server name")};
  }

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

  void generatePoints(
    geometry_msgs::msg::TransformStamped location, double distance,
    geometry_msgs::msg::PoseStamped & nearby_random_point);

private:
  rclcpp::Node::SharedPtr ros_node_;  // 从 blackboard 获取的 ROS 节点
  int goal_count;
  int expected_nearby_goal_count;
  float expected_dis;
  geometry_msgs::msg::TransformStamped current_location;
  geometry_msgs::msg::PoseStamped current_goal;
  std::string action_name_;
  std::string prev_action_name_;  // 保存之前的action名称，用于比较是否需要重新创建客户端

  // Navigation action client
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
  GoalHandleNav::SharedPtr goal_handle_;
  std::shared_future<GoalHandleNav::WrappedResult> result_future_;
  bool goal_sent_;
  bool navigation_complete_;
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_AROUND_HPP_