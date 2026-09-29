#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__PURSUE_ENEMY_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__PURSUE_ENEMY_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <optional>
#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <deque>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"

namespace rm_behavior_tree
{

/**
 * @brief 追击敌人节点：计算追击位置并导航到达
 *
 * 行为逻辑：
 * - onStart: 发送初始追击目标
 * - onRunning: 敌人移动超过阈值才更新目标，避免震荡
 * - 使用滑动窗口滤波消除装甲板自旋抖动
 * - 到达追击点且敌人稳定时返回 SUCCESS（停下来自瞄）
 * - 敌人消失时取消目标并返回 FAILURE
 */
class PursueEnemyAction : public BT::StatefulActionNode
{
public:
  PursueEnemyAction(const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<auto_aim_interfaces::msg::Armors>("armors"),
      BT::InputPort<geometry_msgs::msg::TransformStamped>("current_location"),
      BT::InputPort<double>("pursue_radius", 2.0, "Distance to maintain from enemy (meters)"),
      BT::InputPort<double>("pursue_distance_threshold", 0.5, "Enemy move threshold to resend goal (meters)"),
      BT::InputPort<int>("max_pursuit_points", 10, "Maximum number of pursuit goals to send before giving up"),
      BT::InputPort<double>("update_interval", 0.5, "Interval to update pursue goal (seconds), 0 means update every tick"),
      BT::InputPort<double>("enemy_timeout", 3.0, "If no enemy data for this duration (seconds), return FAILURE (enemy lost)"),
      BT::InputPort<std::string>("action_name", "navigate_to_pose", "Navigation action server name"),
      BT::InputPort<std::string>("target_frame", "map", "Target frame for navigation (map or odom)"),
      BT::InputPort<std::string>("robot_base_frame", "gimbal_yaw", "Robot base frame"),
      BT::InputPort<std::string>("map_topic", "/map", "Map topic for obstacle detection"),
      BT::InputPort<double>("wall_check_clearance", 0.0, "Min clearance around line to enemy (meters), 0 means disabled")};
  }

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  // 计算最佳追击位置
  geometry_msgs::msg::PoseStamped calculatePursuePose(
    const auto_aim_interfaces::msg::Armors & armors,
    const geometry_msgs::msg::TransformStamped & robot_location);

  // 从armors中获取敌人位置（选择最近的敌人）
  geometry_msgs::msg::Point getEnemyPosition(const auto_aim_interfaces::msg::Armors & armors);

  // 计算从机器人到敌人的角度
  double calculateAngleToEnemy(
    const geometry_msgs::msg::Point & robot_pos,
    const geometry_msgs::msg::Point & enemy_pos);

  // 生成追击位置（在敌人和机器人之间，保持一定距离）
  geometry_msgs::msg::PoseStamped generatePursuePose(
    const geometry_msgs::msg::Point & enemy_pos,
    const geometry_msgs::msg::Point & robot_pos,
    double radius);

  // 检查到敌人之间是否有障碍物
  bool hasObstacleBetweenRobotAndEnemy(
    const geometry_msgs::msg::Point & robot_pos,
    const geometry_msgs::msg::Point & enemy_pos);

  // 初始化地图订阅
  void initMapSubscription();

  // 成员变量
  rclcpp::Node::SharedPtr ros_node_;  // 从 blackboard 获取的 ROS 节点
  std::string action_name_;
  std::string target_frame_;
  std::string robot_base_frame_;
  double pursue_radius_;
  double pursue_distance_threshold_;
  int max_pursuit_points_;
  double update_interval_;  // 更新目标的间隔时间
  double enemy_timeout_;  // 敌人数据超时时间（秒）
  double wall_check_clearance_;  // 障碍物检测的安全间隔（米）
  std::string map_topic_;  // 地图话题名称

  // 滑动窗口滤波（消除装甲板自旋抖动，3帧≈150ms@2Hz）
  static constexpr size_t kEnemyHistorySize = 3;
  std::deque<geometry_msgs::msg::Point> enemy_pos_history_;
  geometry_msgs::msg::Point getFilteredEnemyPos();

  // 用滤波后的敌人位置计算追击点（避免重复调用getEnemyPosition）
  geometry_msgs::msg::PoseStamped calculatePursuePoseWithFilteredPos(
    const geometry_msgs::msg::Point & filtered_enemy_pos,
    const geometry_msgs::msg::TransformStamped & robot_location);

  // Navigation action client
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
  std::optional<std::shared_ptr<GoalHandleNav>> goal_handle_;
  bool goal_sent_;
  bool navigation_complete_;
  bool arrived_at_enemy_;
  rclcpp_action::ResultCode last_result_code_;
  geometry_msgs::msg::Point last_enemy_pos_;
  bool last_enemy_pos_set_;
  geometry_msgs::msg::Point last_sent_enemy_pos_;
  bool last_sent_enemy_pos_set_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 当前目标
  geometry_msgs::msg::PoseStamped current_goal_;
  std::chrono::steady_clock::time_point last_update_time_;

  // 追击点计数
  int pursuit_points_sent_;
  rclcpp::Time last_armor_stamp_;
  rclcpp::Time first_armor_stamp_;
  bool has_received_armor_;

  // 地图订阅（用于障碍物检测）
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  std::mutex map_mutex_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__PURSUE_ENEMY_HPP_

