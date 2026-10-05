// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_PLUGINS__BEHAVIORS__COMPUTE_PATH_TO_POSE_WITH_EDGE_WAYPOINT_HPP_
#define NAV2_PLUGINS__BEHAVIORS__COMPUTE_PATH_TO_POSE_WITH_EDGE_WAYPOINT_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_plugins
{

class ComputePathToPoseWithEdgeWaypoint : public nav2_behavior_tree::BtActionNode<
    nav2_msgs::action::ComputePathToPose>
{
public:
  ComputePathToPoseWithEdgeWaypoint(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
    : nav2_behavior_tree::BtActionNode<nav2_msgs::action::ComputePathToPose>(
      xml_tag_name, action_name, conf),
    goal_was_outside_map_(false),
    last_goal_id_(),
    temporary_goal_in_progress_(false),
    force_original_goal_after_temporary_(false),
    has_last_robot_pose_(false)
  {
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    initCostmapClient();
  }

  ComputePathToPoseWithEdgeWaypoint(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
    : nav2_behavior_tree::BtActionNode<nav2_msgs::action::ComputePathToPose>(
      xml_tag_name,
      [conf]() -> std::string {
        auto it = conf.input_ports.find("server_name");
        return (it != conf.input_ports.end() && !it->second.empty()) ?
               it->second : "compute_path_to_pose";
    }(),
    conf),
    goal_was_outside_map_(false),
    last_goal_id_(),
    temporary_goal_in_progress_(false),
    force_original_goal_after_temporary_(false),
    has_last_robot_pose_(false)
  {
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    initCostmapClient();
  }

  ~ComputePathToPoseWithEdgeWaypoint() = default;

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
        BT::OutputPort<nav_msgs::msg::Path>("path", "Path created by planner"),
        BT::InputPort<std::string>(
            "planner_id", "",
            "Planner plugin ID to use (default: uses server's default)"),
        BT::InputPort<std::string>(
            "server_name", "compute_path_to_pose",
            "Action server name (default: compute_path_to_pose)"),
        BT::InputPort<std::string>(
            "global_costmap_service", "global_costmap/get_costmap",
            "Service name for global costmap (e.g., global_costmap/get_costmap)"),
        BT::InputPort<bool>("input_goal_is_temporary", false,
          "Input goal is a temporary goal"),
        BT::InputPort<std::string>(
          "robot_base_frame", "gimbal_yaw_fake",
          "Robot base frame used for temporary-goal reach checks"),
        BT::InputPort<double>("temporary_goal_reached_tolerance", 0.30,
          "Distance threshold to consider temporary target reached"),
        BT::InputPort<double>("temporary_goal_stall_timeout", 1.5,
          "Max time (s) to keep trying a temporary goal before forcing original goal"),
        BT::InputPort<double>("temporary_goal_stall_progress_distance", 0.05,
          "Minimum distance improvement needed to consider temporary goal progress"),
        BT::InputPort<double>("search_radius", 1.0,
          "Maximum radius to sample around blocked goals"),
        BT::InputPort<double>("search_resolution", 0.05,
          "Sampling resolution for blocked-goal neighbor search"),
        BT::InputPort<double>("inflation_gradient_weight", 1.0,
          "Penalty for samples with higher inflation cost"),
        BT::InputPort<double>("edge_search_resolution", 0.1,
          "Resolution (m) when searching for the nearest edge point"),
        BT::InputPort<double>("edge_max_search_dist", 5.0,
          "Maximum distance (m) to search for edge fallback"),
        BT::InputPort<double>("edge_minimum_travel_distance", 0.35,
          "Minimum robot-travel distance before treating edge waypoint as temporary target reached"),
        BT::OutputPort<bool>("output_is_temporary_goal", "Whether output_goal is temporary"),
        BT::OutputPort<std::string>("goal_source", "Source tag for selected goal"),
      });
  }

private:
  void initCostmapClient();

  bool processGoalWithCachedCostmap(
    const geometry_msgs::msg::PoseStamped & goal,
    const nav2_msgs::msg::Costmap & costmap,
    geometry_msgs::msg::PoseStamped & effective_goal,
    std::string & effective_planner_id,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    bool force_original_goal,
    bool input_goal_is_temporary,
    bool & output_is_temporary_goal,
    std::string & goal_source,
    double search_radius,
    double search_resolution,
    double inflation_gradient_weight,
    double edge_search_resolution,
    double edge_max_search_dist,
    double edge_minimum_travel_distance);

  bool isPoseInCostmap(
    const geometry_msgs::msg::PoseStamped & pose,
    const nav2_msgs::msg::Costmap & costmap) const;

  bool isPoseInCostmap(
    double world_x,
    double world_y,
    const nav2_msgs::msg::Costmap & costmap) const;

  int getCostAtPosition(
    const nav2_msgs::msg::Costmap & costmap,
    double world_x,
    double world_y) const;

  bool isPositionReachable(
    const nav2_msgs::msg::Costmap & costmap,
    double world_x,
    double world_y) const;

  bool findNearestEdgeWaypoint(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & goal,
    const nav2_msgs::msg::Costmap & costmap,
    geometry_msgs::msg::PoseStamped & out_waypoint) const;

  bool findReachableEdgeWaypointWithTravelDistance(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & goal,
    const nav2_msgs::msg::Costmap & costmap,
    double minimum_travel_distance,
    double edge_search_resolution,
    double edge_max_search_dist,
    geometry_msgs::msg::PoseStamped & out_waypoint) const;

  bool findNearestFreeGoalAroundBlockedPoint(
    const geometry_msgs::msg::PoseStamped & blocked_goal,
    const nav2_msgs::msg::Costmap & costmap,
    double search_radius,
    double search_resolution,
    double inflation_gradient_weight,
    geometry_msgs::msg::PoseStamped & out_goal) const;

  bool isGoalBlockedByInflation(const nav2_msgs::msg::Costmap & costmap, double world_x, double world_y) const;

  bool hasReachedPose(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & target_pose,
    double tolerance) const;

  // Shared future stored between ticks to avoid re-sending requests.
  // Kept alive by std::shared_ptr so it survives across on_tick calls.
  // After response is received, this is reset and cached_costmap_ is used instead.
  std::shared_ptr<
      rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedFuture > pending_future_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;

  std::mutex costmap_mutex_;
  nav2_msgs::msg::Costmap::SharedPtr cached_costmap_;
  std::string cached_goal_id_;

  bool goal_was_outside_map_;
  std::string last_goal_id_;
  bool temporary_goal_in_progress_;
  geometry_msgs::msg::PoseStamped temporary_goal_;
  bool force_original_goal_after_temporary_;
  geometry_msgs::msg::PoseStamped last_robot_pose_;
  bool has_last_robot_pose_;

  rclcpp::Time temporary_goal_in_progress_since_;
  rclcpp::Time temporary_goal_last_progress_time_;
  double temporary_goal_best_distance_to_goal_{0.0};
  bool temporary_goal_distance_valid_{false};
  };
}  // namespace nav2_plugins

#endif  // NAV2_PLUGINS__BEHAVIORS__COMPUTE_PATH_TO_POSE_WITH_EDGE_WAYPOINT_HPP_
