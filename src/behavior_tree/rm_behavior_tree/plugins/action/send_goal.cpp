#include "rm_behavior_tree/plugins/action/send_goal.hpp"
#include "rm_behavior_tree/bt_conversions.hpp"
#include <sstream>
#include <algorithm>
#include <limits>

namespace rm_behavior_tree
{

SendGoalAction::SendGoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
{
  last_distance_remaining_ = std::numeric_limits<double>::max();
  close_count_ = 0;
}

bool SendGoalAction::setGoal(nav2_msgs::action::NavigateToPose::Goal & goal)
{
  auto res = getInput<std::string>("goal_pose");
  if (!res) {
    throw BT::RuntimeError("error reading port [goal_pose]:", res.error());
  }

  std::string pose_str = res.value();

  std::string pose_str_for_parse = pose_str;
  std::replace(pose_str_for_parse.begin(), pose_str_for_parse.end(), ';', ' ');
  std::stringstream ss(pose_str_for_parse);

  geometry_msgs::msg::Pose pose;
  ss >> pose.position.x >> pose.position.y >> pose.position.z
     >> pose.orientation.x >> pose.orientation.y >> pose.orientation.z >> pose.orientation.w;

  if (ss.fail()) {
    throw BT::RuntimeError("invalid goal_pose format: " + pose_str);
  }

  goal.pose.pose = pose;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = rclcpp::Clock().now();

  goal_pose_ = pose;
  last_distance_remaining_ = std::numeric_limits<double>::max();
  close_count_ = 0;
  last_goal_pose_str_ = pose_str;

  RCLCPP_INFO(node_->get_logger(), "[SendGoal] Sending goal: [%.1f, %.1f]",
    goal.pose.pose.position.x, goal.pose.pose.position.y);

  return true;
}

void SendGoalAction::onHalt()
{
  RCLCPP_INFO(node_->get_logger(), "[SendGoal] onHalt called.");
  close_count_ = 0;
}

BT::NodeStatus SendGoalAction::onResultReceived(const WrappedResult & wr)
{
  switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(),
                  "[SendGoal] onResultReceived: SUCCEEDED, distance=%.3f",
                  last_distance_remaining_);
      close_count_ = 0;
      return BT::NodeStatus::SUCCESS;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(node_->get_logger(), "[SendGoal] onResultReceived: ABORTED -> FAILURE");
      return BT::NodeStatus::FAILURE;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(node_->get_logger(), "[SendGoal] Goal was canceled");
      return BT::NodeStatus::FAILURE;
    default:
      RCLCPP_INFO(node_->get_logger(), "[SendGoal] Unknown result code -> FAILURE");
      return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus SendGoalAction::onFeedback(
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  // Check if goal changed
  auto res = getInput<std::string>("goal_pose");
  if (res && !res.value().empty() && res.value() != last_goal_pose_str_)
  {
    RCLCPP_INFO(node_->get_logger(),
                "[SendGoal] onFeedback: Goal changed from [%s] to [%s], halting to allow new goal",
                last_goal_pose_str_.c_str(), res.value().c_str());
    halt();  // This will reset the node and clear internal state
    return BT::NodeStatus::FAILURE;
  }

  last_distance_remaining_ = feedback->distance_remaining;

  if (feedback->distance_remaining > 1e10) {
    RCLCPP_WARN(node_->get_logger(),
                "[SendGoal] distance=%.3e (abnormal)",
                feedback->distance_remaining);
    return BT::NodeStatus::RUNNING;
  }

  if (feedback->distance_remaining < 0.3) {
    close_count_++;
    if (close_count_ >= 3) {
      RCLCPP_INFO(node_->get_logger(),
                  "[SendGoal] Arrived! distance=%.3f, returning SUCCESS",
                  feedback->distance_remaining);
      close_count_ = 0;
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_DEBUG(node_->get_logger(),
                "[SendGoal] Close: distance=%.3f, count=%d/3",
                feedback->distance_remaining, close_count_);
  } else {
    close_count_ = 0;
    RCLCPP_DEBUG(node_->get_logger(),
                 "[SendGoal] distance=%.3f",
                 feedback->distance_remaining);
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SendGoalAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(node_->get_logger(), "[SendGoal] onFailure: error_code=%d", error);
  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SendGoalAction, "SendGoal");
