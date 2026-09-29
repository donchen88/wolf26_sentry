#include "rm_behavior_tree/plugins/action/move_around.hpp"

#include <random>
#include <thread>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;
using namespace std::chrono;

namespace rm_behavior_tree
{

MoveAroundAction::MoveAroundAction(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  // 创建自己的 ROS 节点
  ros_node_ = rclcpp::Node::make_shared("move_around_node");

  // 获取action名称（延迟到 onStart 初始化）
  std::string default_action_name = "navigate_to_pose";
  if (auto action_name_res = getInput<std::string>("action_name")) {
    action_name_ = action_name_res.value();
  } else {
    action_name_ = default_action_name;
  }

  prev_action_name_ = action_name_;
  goal_sent_ = false;
  navigation_complete_ = false;
  nav_action_client_ = nullptr;
}

BT::NodeStatus MoveAroundAction::onStart()
{
  // 初始化默认值
  current_location.header.frame_id = "map";
  current_location.transform.translation.x = 0.0;
  current_location.transform.translation.y = 0.0;
  current_location.transform.translation.z = 0.0;
  current_location.transform.rotation.x = 0.0;
  current_location.transform.rotation.y = 0.0;
  current_location.transform.rotation.z = 0.0;
  current_location.transform.rotation.w = 1.0;

  expected_dis = 0.0;
  expected_nearby_goal_count = 0;
  goal_count = 0;
  goal_sent_ = false;
  navigation_complete_ = false;

  // 获取参数：机器人当前位置坐标的blackboard映射
  if (!getInput("message", current_location)) {
    RCLCPP_WARN(ros_node_->get_logger(), "MoveAround: missing required input [message]");
    return BT::NodeStatus::FAILURE;
  }

  // 获取参数：期望的距离
  if (!getInput("expected_dis", expected_dis)) {
    RCLCPP_WARN(ros_node_->get_logger(), "MoveAround: missing required input [expected_dis]");
    return BT::NodeStatus::FAILURE;
  }

  // 获取参数：期望的点位数量
  if (!getInput("expected_nearby_goal_count", expected_nearby_goal_count)) {
    RCLCPP_WARN(ros_node_->get_logger(), "MoveAround: missing required input [expected_nearby_goal_count]");
    return BT::NodeStatus::FAILURE;
  }

  // 获取action名称
  if (auto action_name_res = getInput<std::string>("action_name")) {
    action_name_ = action_name_res.value();
    // 如果action名称改变，重新创建客户端
    if (nav_action_client_ == nullptr || prev_action_name_ != action_name_) {
      nav_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        ros_node_, action_name_);
      prev_action_name_ = action_name_;
    }
  }

  // 检查导航action服务器是否可用
  if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(ros_node_->get_logger(), "MoveAround: Action server %s not available", action_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (expected_nearby_goal_count <= 0) {
    // 不需要移动，直接返回成功
    return BT::NodeStatus::SUCCESS;
  }

  // 生成第一个目标点并发送
  generatePoints(current_location, expected_dis, current_goal);
  goal_count = 0;
  
  // 发送导航目标
  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose = current_goal;
  // 使用实际位置的 frame_id（可能是 "map" 或 "odom"）
  goal_msg.pose.header.frame_id = current_goal.header.frame_id;
  goal_msg.pose.header.stamp = ros_node_->now();

  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = [this](const GoalHandleNav::WrappedResult & result) {
    navigation_complete_ = true;
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(ros_node_->get_logger(), "MoveAround: Navigation to goal %d succeeded", goal_count);
  } else {
      RCLCPP_WARN(ros_node_->get_logger(), "MoveAround: Navigation to goal %d failed", goal_count);
    }
  };

  auto future_goal_handle = nav_action_client_->async_send_goal(goal_msg, send_goal_options);
  
  // 等待目标发送完成（非阻塞，回调会在主循环的executor中处理）
  // 这里只等待目标被接受，不等待导航完成
  auto start_time = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    if (future_goal_handle.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
      goal_handle_ = future_goal_handle.get();
      if (!goal_handle_) {
        RCLCPP_WARN(ros_node_->get_logger(), "MoveAround: Goal was rejected");
        return BT::NodeStatus::FAILURE;
      }
      break;
    }
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed > std::chrono::seconds(5)) {
      RCLCPP_WARN(ros_node_->get_logger(), "MoveAround: Timeout waiting to send goal (5s)");
      return BT::NodeStatus::FAILURE;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  goal_sent_ = true;
  goal_count++;
  RCLCPP_INFO(ros_node_->get_logger(), "MoveAround: Sent goal %d/%d", goal_count, expected_nearby_goal_count);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveAroundAction::onRunning()
{
  // 检查导航是否完成
  if (navigation_complete_) {
    // 检查是否还有更多点要发送
    if (goal_count >= expected_nearby_goal_count) {
      // 所有点都已完成
      RCLCPP_INFO(ros_node_->get_logger(), "MoveAround: All %d goals completed", expected_nearby_goal_count);
    return BT::NodeStatus::SUCCESS;
  } else {
      // 发送下一个目标点
      // 更新当前位置（使用当前目标点作为新的起点）
      current_location.transform.translation.x = current_goal.pose.position.x;
      current_location.transform.translation.y = current_goal.pose.position.y;
      current_location.transform.translation.z = current_goal.pose.position.z;
      current_location.transform.rotation = current_goal.pose.orientation;
      
      // 生成下一个随机点
      generatePoints(current_location, expected_dis, current_goal);
      
      // 发送新的导航目标
      auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
      goal_msg.pose = current_goal;
      // 使用实际位置的 frame_id（可能是 "map" 或 "odom"）
      goal_msg.pose.header.frame_id = current_goal.header.frame_id;
      goal_msg.pose.header.stamp = ros_node_->now();

      auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
      send_goal_options.result_callback = [this](const GoalHandleNav::WrappedResult & result) {
        navigation_complete_ = true;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(ros_node_->get_logger(), "MoveAround: Navigation to goal %d succeeded", goal_count);
        } else {
          RCLCPP_WARN(ros_node_->get_logger(), "MoveAround: Navigation to goal %d failed", goal_count);
        }
      };

      auto future_goal_handle = nav_action_client_->async_send_goal(goal_msg, send_goal_options);
      
      // 等待目标发送完成（非阻塞，回调会在主循环的executor中处理）
      auto start_time = std::chrono::steady_clock::now();
      while (rclcpp::ok()) {
        if (future_goal_handle.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
          goal_handle_ = future_goal_handle.get();
          if (!goal_handle_) {
            RCLCPP_WARN(ros_node_->get_logger(), "MoveAround: Goal %d was rejected", goal_count + 1);
            return BT::NodeStatus::FAILURE;
          }
          break;
        }
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed > std::chrono::seconds(1)) {
          RCLCPP_WARN(ros_node_->get_logger(), "MoveAround: Timeout waiting to send goal %d", goal_count + 1);
          return BT::NodeStatus::FAILURE;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      goal_sent_ = true;
      navigation_complete_ = false;
    goal_count++;
      RCLCPP_INFO(ros_node_->get_logger(), "MoveAround: Sent goal %d/%d", goal_count, expected_nearby_goal_count);
      
    return BT::NodeStatus::RUNNING;
  }
  }
  
  // 导航还在进行中，继续等待
  return BT::NodeStatus::RUNNING;
}

void MoveAroundAction::onHalted()
{
  // 取消当前的导航目标
  if (goal_handle_ && goal_sent_) {
    auto future_cancel = nav_action_client_->async_cancel_goal(goal_handle_);
    RCLCPP_INFO(ros_node_->get_logger(), "MoveAround: Canceling navigation goal");
    // 等待取消完成（非阻塞）
    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      if (future_cancel.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
        break;
      }
      auto elapsed = std::chrono::steady_clock::now() - start_time;
      if (elapsed > std::chrono::seconds(1)) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  goal_sent_ = false;
  navigation_complete_ = false;
}

void MoveAroundAction::generatePoints(
  geometry_msgs::msg::TransformStamped location, double distance,
  geometry_msgs::msg::PoseStamped & nearby_random_point)
{
  // 创建随机数生成器
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0, 2 * M_PI);

  // 生成随机角度
  double angle = dis(gen);

  nearby_random_point.header.stamp = rclcpp::Clock().now();
  // 使用位置的 frame_id（可能是 "map" 或 "odom"）
  nearby_random_point.header.frame_id = location.header.frame_id;
  nearby_random_point.pose.position.x = location.transform.translation.x + distance * sin(angle);
  nearby_random_point.pose.position.y = location.transform.translation.y + distance * cos(angle);
  nearby_random_point.pose.position.z = location.transform.translation.z;
  nearby_random_point.pose.orientation.x = location.transform.rotation.x;
  nearby_random_point.pose.orientation.y = location.transform.rotation.y;
  nearby_random_point.pose.orientation.z = location.transform.rotation.z;
  nearby_random_point.pose.orientation.w = location.transform.rotation.w;
}


}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::MoveAroundAction>("MoveAround");//注册为 BehaviorTree.CPP 的“普通节点”插件
}
