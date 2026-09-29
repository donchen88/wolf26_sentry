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

#include "pb_nav2_plugins/behaviors/compute_path_to_pose_with_edge_waypoint.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <sstream>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_costmap_2d/cost_values.hpp"

namespace pb_nav2_behaviors
{

void ComputePathToPoseWithEdgeWaypoint::on_tick()
{
  if (!getInput("goal", goal_.goal)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get 'goal' from blackboard");
    should_send_goal_ = false;
    return;
  }

  if (!getInput("planner_id", goal_.planner_id)) {
    goal_.planner_id = "";
  }

  bool input_goal_is_temporary = false;
  getInput("input_goal_is_temporary", input_goal_is_temporary);

  std::string robot_base_frame_input;
  if (!getInput("robot_base_frame", robot_base_frame_input)) {
    robot_base_frame_input.clear();
  }

  double search_radius = 1.0;
  double search_resolution = 0.05;
  double inflation_gradient_weight = 1.0;
  double edge_search_resolution = 0.1;
  double edge_max_search_dist = 5.0;
  double edge_minimum_travel_distance = 0.35;
  double temporary_goal_reached_tolerance = 0.30;
  double temporary_goal_stall_timeout = 1.5;
  double temporary_goal_stall_progress_distance = 0.05;
  getInput("search_radius", search_radius);
  getInput("search_resolution", search_resolution);
  getInput("inflation_gradient_weight", inflation_gradient_weight);
  getInput("edge_search_resolution", edge_search_resolution);
  getInput("edge_max_search_dist", edge_max_search_dist);
  getInput("edge_minimum_travel_distance", edge_minimum_travel_distance);
  getInput("temporary_goal_reached_tolerance", temporary_goal_reached_tolerance);
  getInput("temporary_goal_stall_timeout", temporary_goal_stall_timeout);
  getInput("temporary_goal_stall_progress_distance", temporary_goal_stall_progress_distance);
  temporary_goal_stall_progress_distance = std::max(0.0, temporary_goal_stall_progress_distance);

  goal_.use_start = false;

  std::string robot_base_frame = robot_base_frame_input;
  if (robot_base_frame.empty()) {
    if (!node_->get_parameter("robot_base_frame", robot_base_frame) ||
      robot_base_frame.empty())
    {
      robot_base_frame = "gimbal_yaw_fake";
    }
  }

  std::ostringstream goal_id_ss;
  goal_id_ss << goal_.goal.header.frame_id << ";"
              << goal_.goal.pose.position.x << ";"
              << goal_.goal.pose.position.y;
  std::string current_goal_id = goal_id_ss.str();

  bool new_goal = (current_goal_id != last_goal_id_);
  if (new_goal) {
    goal_was_outside_map_ = false;
    last_goal_id_ = current_goal_id;
    pending_future_.reset();
    temporary_goal_in_progress_ = false;
    temporary_goal_ = geometry_msgs::msg::PoseStamped();
    force_original_goal_after_temporary_ = false;
    has_last_robot_pose_ = false;
    input_goal_is_temporary = false;
    temporary_goal_distance_valid_ = false;
  }

  RCLCPP_INFO(node_->get_logger(),
    "on_tick: goal=[%.2f, %.2f], frame=%s, planner=%s, edge_mode=%s, new_goal=%d",
    goal_.goal.pose.position.x, goal_.goal.pose.position.y,
    goal_.goal.header.frame_id.c_str(),
    goal_.planner_id.c_str(),
    (goal_was_outside_map_ ? "EDGE" : "NORMAL"),
    (int)new_goal);

  // Step 1: Try to get/use cached costmap (non-blocking for repeat ticks)
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (cached_costmap_ && !cached_costmap_->data.empty() && cached_goal_id_ == current_goal_id) {
      // Already have a valid cached costmap for this exact goal — skip fetch
      RCLCPP_INFO(node_->get_logger(),
        "Using cached costmap for goal [%.2f, %.2f].",
        goal_.goal.pose.position.x, goal_.goal.pose.position.y);
    } else {
      // Need a fresh costmap — fetch synchronously with short timeout
      if (!costmap_client_->service_is_ready()) {
        RCLCPP_INFO(node_->get_logger(),
          "Costmap service not ready — sending original goal.");
        return;
      }
      if (!pending_future_) {
        pending_future_ = std::make_shared<
            rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedFuture>(
            costmap_client_->async_send_request(
                std::make_shared<nav2_msgs::srv::GetCostmap::Request>()));
      }
      // Use spin_until_future_complete instead of wait_for so that the executor
      // can dispatch the response callback while we wait. wait_for() alone does
      // NOT spin the executor, so callbacks never run and the future never
      // becomes ready.
      auto ret = rclcpp::spin_until_future_complete(
          node_, *pending_future_, std::chrono::milliseconds(50));
      if (ret == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = pending_future_->get();
        pending_future_.reset();
        if (response && !response->map.data.empty()) {
          cached_costmap_ = std::make_shared<nav2_msgs::msg::Costmap>(response->map);
          cached_goal_id_ = current_goal_id;
          RCLCPP_INFO(node_->get_logger(),
            "Costmap received for goal [%.2f, %.2f]: %ux%u, "
            "origin=(%.2f,%.2f), bounds=[%.2f,%.2f]x[%.2f,%.2f]",
            goal_.goal.pose.position.x, goal_.goal.pose.position.y,
            cached_costmap_->metadata.size_x, cached_costmap_->metadata.size_y,
            cached_costmap_->metadata.origin.position.x,
            cached_costmap_->metadata.origin.position.y,
            cached_costmap_->metadata.origin.position.x,
            cached_costmap_->metadata.origin.position.x + cached_costmap_->metadata.size_x * cached_costmap_->metadata.resolution,
            cached_costmap_->metadata.origin.position.y,
            cached_costmap_->metadata.origin.position.y + cached_costmap_->metadata.size_y * cached_costmap_->metadata.resolution);
        } else {
          RCLCPP_WARN(node_->get_logger(),
            "Costmap service returned empty — sending original goal.");
          return;
        }
      } else {
        RCLCPP_INFO(node_->get_logger(),
          "Costmap not ready yet (timeout) — sending original goal.");
        return;
      }
    }
  }

  // Step 2: Query robot pose via TF
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.stamp = node_->now();
  robot_pose.header.frame_id = goal_.goal.header.frame_id.empty() ? "map" : goal_.goal.header.frame_id;
  bool has_robot_pose = true;
  std::vector<std::string> robot_frame_candidates;
  auto add_frame_candidate = [&](const std::string & frame) {
    if (frame.empty()) {
      return;
    }

    auto add_unique = [&](const std::string & candidate) {
      if (std::find(robot_frame_candidates.begin(), robot_frame_candidates.end(), candidate) ==
          robot_frame_candidates.end()) {
        robot_frame_candidates.push_back(candidate);
      }
    };

    add_unique(frame);

    const std::string ns = node_->get_namespace();
    if (!ns.empty() && ns != "/") {
      std::string namespaced_frame = ns;
      if (!namespaced_frame.empty() && namespaced_frame.back() == '/') {
        namespaced_frame.pop_back();
      }
      namespaced_frame += "/" + frame;
      add_unique(namespaced_frame);
    }
  };

  add_frame_candidate(robot_base_frame);
  if (robot_base_frame != "gimbal_yaw_fake") {
    add_frame_candidate("gimbal_yaw_fake");
  }
  if (robot_base_frame != "base_footprint") {
    add_frame_candidate("base_footprint");
  }
  if (robot_base_frame != "base_link") {
    add_frame_candidate("base_link");
  }

  for (const auto & frame : robot_frame_candidates) {
    try {
      auto transform = tf_buffer_->lookupTransform(
        robot_pose.header.frame_id, frame,
        tf2::TimePointZero);
      robot_pose.pose.position.x = transform.transform.translation.x;
      robot_pose.pose.position.y = transform.transform.translation.y;
      robot_pose.pose.position.z = transform.transform.translation.z;
      robot_pose.pose.orientation = transform.transform.rotation;
      has_robot_pose = true;
      RCLCPP_INFO(node_->get_logger(),
        "Robot pose from TF: [%.2f, %.2f] using frame=%s",
        robot_pose.pose.position.x, robot_pose.pose.position.y, frame.c_str());
      break;
    } catch (const tf2::TransformException & ex) {
      has_robot_pose = false;
      RCLCPP_DEBUG(node_->get_logger(), "TF lookup failed using frame=%s: %s", frame.c_str(), ex.what());
    }
  }

  if (!has_robot_pose) {
    if (has_last_robot_pose_) {
      robot_pose = last_robot_pose_;
      has_robot_pose = true;
      RCLCPP_WARN(node_->get_logger(),
        "Could not resolve robot pose from TF candidates. Reusing last known pose [%.2f, %.2f].",
        robot_pose.pose.position.x, robot_pose.pose.position.y);
    } else {
      robot_pose.pose.position.x = 0.0;
      robot_pose.pose.position.y = 0.0;
      robot_pose.pose.position.z = 0.0;
      robot_pose.pose.orientation.w = 1.0;
      RCLCPP_WARN(node_->get_logger(),
        "Could not resolve robot pose from TF candidates and no historical pose is available. Using (0,0).");
    }
  } else {
    last_robot_pose_ = robot_pose;
    has_last_robot_pose_ = true;
  }

  // Step 3: Process goal with cached costmap
  nav2_msgs::msg::Costmap::SharedPtr local_costmap;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    local_costmap = cached_costmap_;
  }

  if (!local_costmap || local_costmap->data.empty()) {
    RCLCPP_WARN(node_->get_logger(),
      "No valid costmap — sending original goal.");
    return;
  }

  geometry_msgs::msg::PoseStamped effective_goal;
  std::string effective_planner_id;
  std::string goal_source = "normal";
  bool output_is_temporary_goal = false;

  // Track temporary goal mode across ticks; once a temporary goal is selected we
  // keep treating subsequent ticks as temporary-mode until it is reclassified as
  // non-temporary by this node.
  bool treat_input_as_temporary = input_goal_is_temporary || temporary_goal_in_progress_;
  if (temporary_goal_in_progress_ && has_robot_pose) {
    const double current_distance_to_temporary_goal = std::hypot(
      robot_pose.pose.position.x - temporary_goal_.pose.position.x,
      robot_pose.pose.position.y - temporary_goal_.pose.position.y);

    bool temporary_stalled = false;
    if (hasReachedPose(robot_pose, temporary_goal_, temporary_goal_reached_tolerance)) {
      temporary_goal_in_progress_ = false;
      treat_input_as_temporary = false;
      force_original_goal_after_temporary_ = true;
      temporary_goal_distance_valid_ = false;
      RCLCPP_INFO(node_->get_logger(),
        "Temporary goal [%.2f, %.2f] reached. Re-evaluating original goal [%.2f, %.2f]",
        temporary_goal_.pose.position.x, temporary_goal_.pose.position.y,
        goal_.goal.pose.position.x, goal_.goal.pose.position.y);
    } else {
      const auto now = node_->now();
      if (!temporary_goal_distance_valid_ || temporary_goal_best_distance_to_goal_ <= 0.0) {
        temporary_goal_distance_valid_ = true;
        temporary_goal_best_distance_to_goal_ = current_distance_to_temporary_goal;
        temporary_goal_in_progress_since_ = now;
        temporary_goal_last_progress_time_ = now;
      } else {
        if (current_distance_to_temporary_goal <
          temporary_goal_best_distance_to_goal_ - temporary_goal_stall_progress_distance)
        {
          temporary_goal_best_distance_to_goal_ = current_distance_to_temporary_goal;
          temporary_goal_last_progress_time_ = now;
        }
      }

      if ((now - temporary_goal_last_progress_time_) >=
          rclcpp::Duration::from_seconds(std::max(0.0, temporary_goal_stall_timeout)))
      {
        temporary_stalled = true;
      }

      if (temporary_stalled) {
        temporary_goal_in_progress_ = false;
        treat_input_as_temporary = false;
        force_original_goal_after_temporary_ = true;
        temporary_goal_distance_valid_ = false;
        RCLCPP_WARN(node_->get_logger(),
          "Temporary goal [%.2f, %.2f] stalled for %.2fs (no progress > %.2fm). "
          "Forcing re-evaluation of original goal [%.2f, %.2f]",
          temporary_goal_.pose.position.x,
          temporary_goal_.pose.position.y,
          temporary_goal_stall_timeout,
          temporary_goal_stall_progress_distance,
          goal_.goal.pose.position.x,
          goal_.goal.pose.position.y);
      } else {
        treat_input_as_temporary = true;
      }
    }
  } else if (temporary_goal_in_progress_) {
      const auto now = node_->now();
      if (!temporary_goal_distance_valid_) {
        temporary_goal_distance_valid_ = true;
        temporary_goal_best_distance_to_goal_ = 0.0;
        temporary_goal_in_progress_since_ = now;
        temporary_goal_last_progress_time_ = now;
      } else if ((now - temporary_goal_last_progress_time_) >=
        rclcpp::Duration::from_seconds(std::max(0.0, temporary_goal_stall_timeout)))
      {
        temporary_goal_in_progress_ = false;
        treat_input_as_temporary = false;
        force_original_goal_after_temporary_ = true;
        temporary_goal_distance_valid_ = false;
        RCLCPP_WARN(node_->get_logger(),
          "Temporary goal [%.2f, %.2f] has no valid TF pose update for %.2fs. "
          "Forcing re-evaluation of original goal [%.2f, %.2f]",
          temporary_goal_.pose.position.x,
          temporary_goal_.pose.position.y,
          temporary_goal_stall_timeout,
          goal_.goal.pose.position.x,
          goal_.goal.pose.position.y);
      }
  }

  if (!treat_input_as_temporary && force_original_goal_after_temporary_) {
    RCLCPP_INFO(node_->get_logger(),
      "Force re-evaluate original goal [%.2f, %.2f] after temporary target." ,
      goal_.goal.pose.position.x, goal_.goal.pose.position.y);
  }

  if (!processGoalWithCachedCostmap(
       goal_.goal, *local_costmap, effective_goal, effective_planner_id, robot_pose,
       force_original_goal_after_temporary_, treat_input_as_temporary,
       output_is_temporary_goal, goal_source,
       search_radius, search_resolution, inflation_gradient_weight,
       edge_search_resolution, edge_max_search_dist,
       edge_minimum_travel_distance)) {
    RCLCPP_WARN(node_->get_logger(),
      "Could not process goal — sending original goal.");
    return;
  }

  goal_.goal = effective_goal;
  if (!effective_planner_id.empty()) {
    goal_.planner_id = effective_planner_id;
  }

  if (!setOutput<bool>("output_is_temporary_goal", output_is_temporary_goal)) {
    RCLCPP_WARN(node_->get_logger(), "Failed to set output port: output_is_temporary_goal");
  }

  if (!setOutput<std::string>("goal_source", goal_source)) {
    RCLCPP_WARN(node_->get_logger(), "Failed to set output port: goal_source");
  }

  if (output_is_temporary_goal) {
      temporary_goal_in_progress_ = true;
      temporary_goal_ = effective_goal;
      force_original_goal_after_temporary_ = false;
      temporary_goal_distance_valid_ = false;
      temporary_goal_in_progress_since_ = node_->now();
      temporary_goal_last_progress_time_ = node_->now();
      temporary_goal_best_distance_to_goal_ = 0.0;
  } else {
      temporary_goal_in_progress_ = false;
      temporary_goal_ = geometry_msgs::msg::PoseStamped();
      force_original_goal_after_temporary_ = false;
      temporary_goal_distance_valid_ = false;
  }

  if (goal_was_outside_map_) {
    RCLCPP_WARN(node_->get_logger(),
      "Goal is outside map. Sending edge waypoint [%.2f, %.2f].",
      effective_goal.pose.position.x, effective_goal.pose.position.y);
  }
}

BT::NodeStatus ComputePathToPoseWithEdgeWaypoint::on_success()
{
  if (!setOutput<nav_msgs::msg::Path>("path", result_.result->path)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to set output path on blackboard");
    return BT::NodeStatus::FAILURE;
  }

  if (goal_was_outside_map_) {
    RCLCPP_WARN(node_->get_logger(),
      "Original goal is outside the global costmap. Keeping edge-mode and "
      "re-evaluating original target on next tick.");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Path computed successfully, %zu poses",
      result_.result->path.poses.size());
  }

  return BT::NodeStatus::SUCCESS;
}

void ComputePathToPoseWithEdgeWaypoint::initCostmapClient()
{
  std::string service_name = "global_costmap/get_costmap";
  getInput<std::string>("global_costmap_service", service_name);

  std::string ns = node_->get_namespace();
  if (!ns.empty() && ns != "/" && service_name[0] != '/') {
    service_name = ns + "/" + service_name;
  }
  costmap_client_ = node_->create_client<nav2_msgs::srv::GetCostmap>(service_name);
  RCLCPP_INFO(node_->get_logger(), "Costmap service client created: %s", service_name.c_str());
}

bool ComputePathToPoseWithEdgeWaypoint::processGoalWithCachedCostmap(
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
  double edge_minimum_travel_distance)
{
  effective_planner_id = "";
  output_is_temporary_goal = false;
  goal_source = "normal";

  if (search_radius < 0.0) {
    search_radius = 0.0;
  }
  if (search_resolution < 1e-3) {
    search_resolution = 0.01;
  }
  if (inflation_gradient_weight < 0.0) {
    inflation_gradient_weight = 0.0;
  }

  const bool goal_in_costmap = isPoseInCostmap(goal, costmap);

  if (goal_was_outside_map_) {
    if (!goal_in_costmap) {
      geometry_msgs::msg::PoseStamped edge_waypoint;
      if (findNearestEdgeWaypoint(robot_pose, goal, costmap, edge_waypoint)) {
        const double edge_distance = std::hypot(edge_waypoint.pose.position.x - robot_pose.pose.position.x,
                                               edge_waypoint.pose.position.y - robot_pose.pose.position.y);
        if (edge_minimum_travel_distance > 0.0 && edge_distance < edge_minimum_travel_distance) {
          geometry_msgs::msg::PoseStamped fallback_edge_waypoint;
          if (findReachableEdgeWaypointWithTravelDistance(
              robot_pose, goal, costmap, edge_minimum_travel_distance,
              edge_search_resolution, edge_max_search_dist, fallback_edge_waypoint))
          {
            edge_waypoint = fallback_edge_waypoint;
            RCLCPP_WARN(
              node_->get_logger(),
              "Edge waypoint near current robot pose (%.2fm). Using fallback with travel distance %.2fm -> [%.2f, %.2f]",
              edge_distance, edge_minimum_travel_distance,
              edge_waypoint.pose.position.x, edge_waypoint.pose.position.y);
          }
        }

        effective_goal = edge_waypoint;
        goal_source = "outside_map_edge";
        goal_was_outside_map_ = true;
        output_is_temporary_goal = true;
        RCLCPP_WARN(node_->get_logger(),
          "Still in edge mode. Original goal [%.2f, %.2f] still outside, "
          "sending updated edge waypoint [%.2f, %.2f].",
          goal.pose.position.x, goal.pose.position.y,
          edge_waypoint.pose.position.x, edge_waypoint.pose.position.y);
        return true;
      }
      RCLCPP_WARN(node_->get_logger(),
        "Could not find reachable edge point. Sending original goal.");
      goal_source = "outside_map_fallback";
      effective_goal = goal;
      goal_was_outside_map_ = false;
      output_is_temporary_goal = true;
      return true;
    }

    RCLCPP_WARN(node_->get_logger(),
      "Original goal [%.2f, %.2f] is now INSIDE the costmap. "
      "Exiting edge mode and checking blocked-goal fallback.",
      goal.pose.position.x, goal.pose.position.y);
    goal_was_outside_map_ = false;
  }

  // If this is a temporary goal and it is already inside global bounds,
  // first try the original goal directly (to keep the original target's
  // semantics such as frame/id/orientation). Only fall back to a nearby
  // free target when the original one is still blocked by inflation.
  if (input_goal_is_temporary && goal_in_costmap) {
    if (!isGoalBlockedByInflation(costmap, goal.pose.position.x, goal.pose.position.y)) {
      output_is_temporary_goal = false;
      goal_source = "temporary_goal_in_range";
      effective_goal = goal;
      return true;
    }

    geometry_msgs::msg::PoseStamped free_goal;
    if (findNearestFreeGoalAroundBlockedPoint(
        goal, costmap, search_radius, search_resolution, inflation_gradient_weight,
        free_goal))
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "Temporary goal [%.2f, %.2f] is in inflated area. "
        "Using nearest free point [%.2f, %.2f] instead.",
        goal.pose.position.x, goal.pose.position.y,
        free_goal.pose.position.x, free_goal.pose.position.y);
      output_is_temporary_goal = true;
      goal_source = "temporary_goal_inflation_adjusted";
      effective_goal = free_goal;
      return true;
    }

    RCLCPP_WARN(
      node_->get_logger(),
      "Temporary goal [%.2f, %.2f] is in inflated area and no free point was found "
      "within radius %.2f. Keeping the original temporary target.",
      goal.pose.position.x, goal.pose.position.y, search_radius);
    output_is_temporary_goal = true;
    goal_source = "temporary_goal_inflation_fallback";
    effective_goal = goal;
    return true;
  }

  if (!goal_in_costmap) {
    RCLCPP_WARN(node_->get_logger(),
      "Goal [%.2f, %.2f] is OUTSIDE global costmap.",
      goal.pose.position.x, goal.pose.position.y);

    geometry_msgs::msg::PoseStamped edge_waypoint;
    if (findNearestEdgeWaypoint(robot_pose, goal, costmap, edge_waypoint)) {
      const double edge_distance = std::hypot(edge_waypoint.pose.position.x - robot_pose.pose.position.x,
                                             edge_waypoint.pose.position.y - robot_pose.pose.position.y);
      if (edge_minimum_travel_distance > 0.0 && edge_distance < edge_minimum_travel_distance) {
        geometry_msgs::msg::PoseStamped fallback_edge_waypoint;
        if (findReachableEdgeWaypointWithTravelDistance(
            robot_pose, goal, costmap, edge_minimum_travel_distance,
            edge_search_resolution, edge_max_search_dist, fallback_edge_waypoint))
        {
          edge_waypoint = fallback_edge_waypoint;
          RCLCPP_WARN(
            node_->get_logger(),
            "Edge waypoint near current robot pose (%.2fm). Using fallback with travel distance %.2fm -> [%.2f, %.2f]",
            edge_distance, edge_minimum_travel_distance,
            edge_waypoint.pose.position.x, edge_waypoint.pose.position.y);
        }
      }

      effective_goal = edge_waypoint;
      goal_source = "outside_map_edge";
      goal_was_outside_map_ = true;
      output_is_temporary_goal = true;
      return true;
    }
    RCLCPP_WARN(node_->get_logger(),
      "Could not find reachable edge point. Sending original goal.");
    goal_source = "outside_map_fallback";
    effective_goal = goal;
    goal_was_outside_map_ = false;
    output_is_temporary_goal = true;
    return true;
  }

  if (isGoalBlockedByInflation(costmap, goal.pose.position.x, goal.pose.position.y)) {
    if (force_original_goal) {
      RCLCPP_WARN(node_->get_logger(),
        "Forcing direct retry of original goal [%.2f, %.2f] after temporary target reached.",
        goal.pose.position.x, goal.pose.position.y);
      output_is_temporary_goal = false;
      goal_source = "force_original_goal_after_temporary";
      effective_goal = goal;
      return true;
    }

    geometry_msgs::msg::PoseStamped free_goal;
    if (findNearestFreeGoalAroundBlockedPoint(
        goal, costmap, search_radius, search_resolution, inflation_gradient_weight,
        free_goal))
    {
      RCLCPP_WARN(node_->get_logger(),
        "Goal [%.2f, %.2f] is blocked by inflation. "
        "Using nearest free goal [%.2f, %.2f] (radius %.2f, resolution %.2f).",
        goal.pose.position.x, goal.pose.position.y,
        free_goal.pose.position.x, free_goal.pose.position.y,
        search_radius, search_resolution);
      output_is_temporary_goal = true;
      goal_source = "inflation_adjusted";
      effective_goal = free_goal;
      return true;
    }

    RCLCPP_WARN(node_->get_logger(),
      "No free candidate found around blocked goal [%.2f, %.2f] in search_radius=%.2f. "
      "Sending original blocked goal.",
      goal.pose.position.x, goal.pose.position.y, search_radius);
    goal_source = "inflation_fallback";
    effective_goal = goal;
    return true;
  }

  effective_goal = goal;
  output_is_temporary_goal = input_goal_is_temporary;
  goal_source = input_goal_is_temporary ? "temporary_goal" : "normal_goal";
  goal_was_outside_map_ = false;
  return true;
}

bool ComputePathToPoseWithEdgeWaypoint::findReachableEdgeWaypointWithTravelDistance(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::PoseStamped & goal,
  const nav2_msgs::msg::Costmap & costmap,
  double minimum_travel_distance,
  double edge_search_resolution,
  double edge_max_search_dist,
  geometry_msgs::msg::PoseStamped & out_waypoint) const
{
  if (minimum_travel_distance <= 0.0) {
    return false;
  }

  if (edge_search_resolution < 1e-3) {
    edge_search_resolution = 0.05;
  }

  if (edge_max_search_dist < 0.0) {
    edge_max_search_dist = 0.0;
  }

  const auto & meta = costmap.metadata;
  const double origin_x = meta.origin.position.x;
  const double origin_y = meta.origin.position.y;
  const double max_x = origin_x + meta.size_x * meta.resolution;
  const double max_y = origin_y + meta.size_y * meta.resolution;

  const double clamped_goal_x = std::max(origin_x, std::min(max_x, goal.pose.position.x));
  const double clamped_goal_y = std::max(origin_y, std::min(max_y, goal.pose.position.y));

  double scan_x_min = origin_x;
  double scan_x_max = max_x;
  double scan_y_min = origin_y;
  double scan_y_max = max_y;
  if (edge_max_search_dist > 0.0) {
    scan_x_min = std::max(origin_x, clamped_goal_x - edge_max_search_dist);
    scan_x_max = std::min(max_x, clamped_goal_x + edge_max_search_dist);
    scan_y_min = std::max(origin_y, clamped_goal_y - edge_max_search_dist);
    scan_y_max = std::min(max_y, clamped_goal_y + edge_max_search_dist);
  }

  bool found = false;
  double best_score = std::numeric_limits<double>::infinity();
  double best_robot_distance = 0.0;

  auto evaluate_candidate = [&](double x, double y) {
    if (!isPositionReachable(costmap, x, y)) {
      return;
    }

    const double distance_to_goal = std::hypot(x - goal.pose.position.x, y - goal.pose.position.y);
    const double distance_to_robot = std::hypot(x - robot_pose.pose.position.x,
                                              y - robot_pose.pose.position.y);
    if (distance_to_robot < minimum_travel_distance) {
      return;
    }

    if (!found || distance_to_goal < best_score ||
        (std::fabs(distance_to_goal - best_score) < 1e-9 &&
         distance_to_robot > best_robot_distance))
    {
      out_waypoint.pose.position.x = x;
      out_waypoint.pose.position.y = y;
      out_waypoint.pose.orientation = goal.pose.orientation;
      out_waypoint.header = goal.header;
      best_score = distance_to_goal;
      best_robot_distance = distance_to_robot;
      found = true;
    }
  };

  for (double x = scan_x_min; x <= scan_x_max; x += edge_search_resolution) {
    evaluate_candidate(x, origin_y);
  }
  if (scan_y_min != origin_y) {
    for (double x = scan_x_min; x <= scan_x_max; x += edge_search_resolution) {
      evaluate_candidate(x, max_y);
    }
  }

  for (double y = scan_y_min; y <= scan_y_max; y += edge_search_resolution) {
    evaluate_candidate(origin_x, y);
  }
  if (scan_x_min != origin_x) {
    for (double y = scan_y_min; y <= scan_y_max; y += edge_search_resolution) {
      evaluate_candidate(max_x, y);
    }
  }

  if (!found) {
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Fallback edge waypoint selected [%.2f, %.2f] with distance_to_goal %.2fm and distance_to_robot %.2fm",
    out_waypoint.pose.position.x, out_waypoint.pose.position.y,
    best_score,
    best_robot_distance);

  return true;
}

bool ComputePathToPoseWithEdgeWaypoint::isPoseInCostmap(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav2_msgs::msg::Costmap & costmap) const
{
  const auto & meta = costmap.metadata;
  double origin_x = meta.origin.position.x;
  double origin_y = meta.origin.position.y;
  double max_x = origin_x + meta.size_x * meta.resolution;
  double max_y = origin_y + meta.size_y * meta.resolution;

  bool inside = (
    pose.pose.position.x >= origin_x && pose.pose.position.x <= max_x &&
    pose.pose.position.y >= origin_y && pose.pose.position.y <= max_y);

  RCLCPP_INFO(node_->get_logger(),
    "Goal [%.2f, %.2f] is %s costmap (bounds [%.2f,%.2f] x [%.2f,%.2f])",
    pose.pose.position.x, pose.pose.position.y,
    inside ? "INSIDE" : "OUTSIDE",
    origin_x, max_x, origin_y, max_y);

  return inside;
}

bool ComputePathToPoseWithEdgeWaypoint::isPoseInCostmap(
  double world_x,
  double world_y,
  const nav2_msgs::msg::Costmap & costmap) const
{
  const auto & meta = costmap.metadata;
  double origin_x = meta.origin.position.x;
  double origin_y = meta.origin.position.y;
  double max_x = origin_x + meta.size_x * meta.resolution;
  double max_y = origin_y + meta.size_y * meta.resolution;

  return (
    world_x >= origin_x && world_x <= max_x &&
    world_y >= origin_y && world_y <= max_y);
}

bool ComputePathToPoseWithEdgeWaypoint::isGoalBlockedByInflation(
  const nav2_msgs::msg::Costmap & costmap,
  double world_x,
  double world_y) const
{
  const int cost = getCostAtPosition(costmap, world_x, world_y);
  if (cost < 0) {
    return true;
  }

  return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

bool ComputePathToPoseWithEdgeWaypoint::hasReachedPose(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::PoseStamped & target_pose,
  double tolerance) const
{
  const double dx = current_pose.pose.position.x - target_pose.pose.position.x;
  const double dy = current_pose.pose.position.y - target_pose.pose.position.y;
  return std::hypot(dx, dy) <= std::max(0.0, tolerance);
}

bool ComputePathToPoseWithEdgeWaypoint::findNearestFreeGoalAroundBlockedPoint(
  const geometry_msgs::msg::PoseStamped & blocked_goal,
  const nav2_msgs::msg::Costmap & costmap,
  double search_radius,
  double search_resolution,
  double inflation_gradient_weight,
  geometry_msgs::msg::PoseStamped & out_goal) const
{
  if (search_radius <= 0.0) {
    return false;
  }

  if (search_resolution < 1e-4) {
    search_resolution = 0.01;
  }

  const double goal_x = blocked_goal.pose.position.x;
  const double goal_y = blocked_goal.pose.position.y;
  const double inflated_weight = std::max(0.0, inflation_gradient_weight);

  double best_score = std::numeric_limits<double>::infinity();
  double best_distance = std::numeric_limits<double>::infinity();
  bool found = false;

  out_goal = blocked_goal;

  for (double x = goal_x - search_radius; x <= goal_x + search_radius; x += search_resolution) {
    for (double y = goal_y - search_radius; y <= goal_y + search_radius; y += search_resolution) {
      if (!isPoseInCostmap(x, y, costmap)) {
        continue;
      }

      if (!isPositionReachable(costmap, x, y)) {
        continue;
      }

      const double distance = std::hypot(x - goal_x, y - goal_y);
      if (distance > search_radius) {
        continue;
      }

      const double cost = static_cast<double>(getCostAtPosition(costmap, x, y));
      const double normalized_cost = std::clamp(cost, 0.0, 255.0) /
        static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
      const double score = distance + inflated_weight * normalized_cost;

      if (!found || score < best_score ||
        (std::fabs(score - best_score) < 1e-9 && distance < best_distance))
      {
        found = true;
        best_score = score;
        best_distance = distance;
        out_goal.pose.position.x = x;
        out_goal.pose.position.y = y;
      }
    }
  }

  return found;
}

int ComputePathToPoseWithEdgeWaypoint::getCostAtPosition(
  const nav2_msgs::msg::Costmap & costmap,
  double world_x,
  double world_y) const
{
  const auto & meta = costmap.metadata;
  double resolution = meta.resolution;
  double origin_x = meta.origin.position.x;
  double origin_y = meta.origin.position.y;
  int size_x = static_cast<int>(meta.size_x);
  int size_y = static_cast<int>(meta.size_y);

  int mx = static_cast<int>((world_x - origin_x) / resolution);
  int my = static_cast<int>((world_y - origin_y) / resolution);

  if (mx < 0 || mx >= size_x || my < 0 || my >= size_y) {
    return -1;
  }

  int index = mx + my * size_x;
  if (index < 0 || index >= static_cast<int>(costmap.data.size())) {
    return -1;
  }

  return static_cast<int>(costmap.data[index]);
}

bool ComputePathToPoseWithEdgeWaypoint::isPositionReachable(
  const nav2_msgs::msg::Costmap & costmap,
  double world_x,
  double world_y) const
{
  int cost = getCostAtPosition(costmap, world_x, world_y);
  if (cost < 0) {
    return false;
  }
  return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

bool ComputePathToPoseWithEdgeWaypoint::findNearestEdgeWaypoint(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::PoseStamped & goal,
  const nav2_msgs::msg::Costmap & costmap,
  geometry_msgs::msg::PoseStamped & out_waypoint) const
{
  const auto & meta = costmap.metadata;
  double origin_x = meta.origin.position.x;
  double origin_y = meta.origin.position.y;
  double max_x = origin_x + meta.size_x * meta.resolution;
  double max_y = origin_y + meta.size_y * meta.resolution;

  double goal_x = goal.pose.position.x;
  double goal_y = goal.pose.position.y;
  double robot_x = robot_pose.pose.position.x;
  double robot_y = robot_pose.pose.position.y;

  double best_t = std::numeric_limits<double>::infinity();
  bool found = false;

  // Ray from ROBOT POSITION toward GOAL.
  double dir_x = goal_x - robot_x;
  double dir_y = goal_y - robot_y;
  double dir_len = std::hypot(dir_x, dir_y);

  RCLCPP_INFO(node_->get_logger(),
    "DEBUG edge: map [%.2f,%.2f] x [%.2f,%.2f], robot [%.2f,%.2f], goal [%.2f,%.2f], dir [%.4f,%.4f] len=%.4f",
    origin_x, origin_y, max_x, max_y, robot_x, robot_y, goal_x, goal_y, dir_x, dir_y, dir_len);

  if (dir_len < 1e-6) {
    RCLCPP_WARN(node_->get_logger(),
      "Robot and goal are at the same position. Cannot determine edge direction.");
    return false;
  }
  dir_x /= dir_len;
  dir_y /= dir_len;

  // Helper: search along a vertical boundary (x = boundary_x) for the nearest
  // reachable point starting from (boundary_x, start_y) and expanding outward
  // in both +/- y directions. Returns true if a reachable point is found.
  auto searchVerticalBoundary = [&](double boundary_x, double start_y,
                                    double y_min, double y_max,
                                    double step, geometry_msgs::msg::PoseStamped & out) {
    // Search positive y direction
    for (double wy = start_y; wy <= y_max + step; wy += step) {
      double cy = std::min(wy, y_max);
      if (isPositionReachable(costmap, boundary_x, cy)) {
        out.pose.position.x = boundary_x;
        out.pose.position.y = cy;
        return true;
      }
    }
    // Search negative y direction
    for (double wy = start_y - step; wy >= y_min - step; wy -= step) {
      double cy = std::max(wy, y_min);
      if (isPositionReachable(costmap, boundary_x, cy)) {
        out.pose.position.x = boundary_x;
        out.pose.position.y = cy;
        return true;
      }
    }
    return false;
  };

  // Helper: search along a horizontal boundary (y = boundary_y) for the nearest
  // reachable point starting from (start_x, boundary_y) and expanding outward
  // in both +/- x directions. Returns true if a reachable point is found.
  auto searchHorizontalBoundary = [&](double boundary_y, double start_x,
                                      double x_min, double x_max,
                                      double step, geometry_msgs::msg::PoseStamped & out) {
    // Search positive x direction
    for (double wx = start_x; wx <= x_max + step; wx += step) {
      double cx = std::min(wx, x_max);
      if (isPositionReachable(costmap, cx, boundary_y)) {
        out.pose.position.x = cx;
        out.pose.position.y = boundary_y;
        return true;
      }
    }
    // Search negative x direction
    for (double wx = start_x - step; wx >= x_min - step; wx -= step) {
      double cx = std::max(wx, x_min);
      if (isPositionReachable(costmap, cx, boundary_y)) {
        out.pose.position.x = cx;
        out.pose.position.y = boundary_y;
        return true;
      }
    }
    return false;
  };

  const double search_step = 0.05;  // 5cm resolution for boundary search

  // RIGHT edge (x = max_x)
  if (dir_x > 1e-6) {
    double t = (max_x - robot_x) / dir_x;
    double wx = max_x;
    double wy = robot_y + dir_y * t;
    geometry_msgs::msg::PoseStamped candidate = goal;
    candidate.pose.position.x = wx;
    candidate.pose.position.y = wy;

    if (t >= 0 && wy >= origin_y - 1e-3 && wy <= max_y + 1e-3) {
      if (isPositionReachable(costmap, wx, wy)) {
        best_t = t;
        out_waypoint = candidate;
        found = true;
        RCLCPP_INFO(node_->get_logger(),
          "DEBUG edge RIGHT: direct hit t=%.2f, wy=%.2f, reachable",
          t, wy);
      } else {
        // Fallback: search along the boundary for nearest reachable point
        geometry_msgs::msg::PoseStamped fallback_candidate;
        if (searchVerticalBoundary(wx, wy, origin_y, max_y, search_step, fallback_candidate)) {
          fallback_candidate.header = goal.header;
          fallback_candidate.pose.orientation = goal.pose.orientation;
          double fallback_t = std::hypot(
              fallback_candidate.pose.position.x - robot_x,
              fallback_candidate.pose.position.y - robot_y);
          if (!found || fallback_t < best_t) {
            best_t = fallback_t;
            out_waypoint = fallback_candidate;
            found = true;
            RCLCPP_WARN(node_->get_logger(),
              "DEBUG edge RIGHT: direct hit not reachable, using fallback [%.2f, %.2f] (t=%.2f)",
              fallback_candidate.pose.position.x, fallback_candidate.pose.position.y, fallback_t);
          }
        } else {
          RCLCPP_INFO(node_->get_logger(),
            "DEBUG edge RIGHT: t=%.2f, wy=%.2f, NOT reachable and no fallback found",
            t, wy);
        }
      }
    }
  }

  // LEFT edge (x = origin_x)
  if (dir_x < -1e-6) {
    double t = (origin_x - robot_x) / dir_x;
    double wx = origin_x;
    double wy = robot_y + dir_y * t;
    geometry_msgs::msg::PoseStamped candidate = goal;
    candidate.pose.position.x = wx;
    candidate.pose.position.y = wy;

    if (t >= 0 && wy >= origin_y - 1e-3 && wy <= max_y + 1e-3) {
      if (isPositionReachable(costmap, wx, wy)) {
        best_t = t;
        out_waypoint = candidate;
        found = true;
        RCLCPP_INFO(node_->get_logger(),
          "DEBUG edge LEFT: direct hit t=%.2f, wy=%.2f, reachable",
          t, wy);
      } else {
        geometry_msgs::msg::PoseStamped fallback_candidate;
        if (searchVerticalBoundary(wx, wy, origin_y, max_y, search_step, fallback_candidate)) {
          fallback_candidate.header = goal.header;
          fallback_candidate.pose.orientation = goal.pose.orientation;
          double fallback_t = std::hypot(
              fallback_candidate.pose.position.x - robot_x,
              fallback_candidate.pose.position.y - robot_y);
          if (!found || fallback_t < best_t) {
            best_t = fallback_t;
            out_waypoint = fallback_candidate;
            found = true;
            RCLCPP_WARN(node_->get_logger(),
              "DEBUG edge LEFT: direct hit not reachable, using fallback [%.2f, %.2f] (t=%.2f)",
              fallback_candidate.pose.position.x, fallback_candidate.pose.position.y, fallback_t);
          }
        } else {
          RCLCPP_INFO(node_->get_logger(),
            "DEBUG edge LEFT: t=%.2f, wy=%.2f, NOT reachable and no fallback found",
            t, wy);
        }
      }
    }
  }

  // TOP edge (y = max_y)
  if (dir_y > 1e-6) {
    double t = (max_y - robot_y) / dir_y;
    double wy = max_y;
    double wx = robot_x + dir_x * t;
    geometry_msgs::msg::PoseStamped candidate = goal;
    candidate.pose.position.x = wx;
    candidate.pose.position.y = wy;

    if (t >= 0 && wx >= origin_x - 1e-3 && wx <= max_x + 1e-3) {
      if (isPositionReachable(costmap, wx, wy)) {
        best_t = t;
        out_waypoint = candidate;
        found = true;
        RCLCPP_INFO(node_->get_logger(),
          "DEBUG edge TOP: direct hit t=%.2f, wx=%.2f, reachable",
          t, wx);
      } else {
        geometry_msgs::msg::PoseStamped fallback_candidate;
        if (searchHorizontalBoundary(wy, wx, origin_x, max_x, search_step, fallback_candidate)) {
          fallback_candidate.header = goal.header;
          fallback_candidate.pose.orientation = goal.pose.orientation;
          double fallback_t = std::hypot(
              fallback_candidate.pose.position.x - robot_x,
              fallback_candidate.pose.position.y - robot_y);
          if (!found || fallback_t < best_t) {
            best_t = fallback_t;
            out_waypoint = fallback_candidate;
            found = true;
            RCLCPP_WARN(node_->get_logger(),
              "DEBUG edge TOP: direct hit not reachable, using fallback [%.2f, %.2f] (t=%.2f)",
              fallback_candidate.pose.position.x, fallback_candidate.pose.position.y, fallback_t);
          }
        } else {
          RCLCPP_INFO(node_->get_logger(),
            "DEBUG edge TOP: t=%.2f, wx=%.2f, NOT reachable and no fallback found",
            t, wx);
        }
      }
    }
  }

  // BOTTOM edge (y = origin_y)
  if (dir_y < -1e-6) {
    double t = (origin_y - robot_y) / dir_y;
    double wy = origin_y;
    double wx = robot_x + dir_x * t;
    geometry_msgs::msg::PoseStamped candidate = goal;
    candidate.pose.position.x = wx;
    candidate.pose.position.y = wy;

    if (t >= 0 && wx >= origin_x - 1e-3 && wx <= max_x + 1e-3) {
      if (isPositionReachable(costmap, wx, wy)) {
        best_t = t;
        out_waypoint = candidate;
        found = true;
        RCLCPP_INFO(node_->get_logger(),
          "DEBUG edge BOTTOM: direct hit t=%.2f, wx=%.2f, reachable",
          t, wx);
      } else {
        geometry_msgs::msg::PoseStamped fallback_candidate;
        if (searchHorizontalBoundary(wy, wx, origin_x, max_x, search_step, fallback_candidate)) {
          fallback_candidate.header = goal.header;
          fallback_candidate.pose.orientation = goal.pose.orientation;
          double fallback_t = std::hypot(
              fallback_candidate.pose.position.x - robot_x,
              fallback_candidate.pose.position.y - robot_y);
          if (!found || fallback_t < best_t) {
            best_t = fallback_t;
            out_waypoint = fallback_candidate;
            found = true;
            RCLCPP_WARN(node_->get_logger(),
              "DEBUG edge BOTTOM: direct hit not reachable, using fallback [%.2f, %.2f] (t=%.2f)",
              fallback_candidate.pose.position.x, fallback_candidate.pose.position.y, fallback_t);
          }
        } else {
          RCLCPP_INFO(node_->get_logger(),
            "DEBUG edge BOTTOM: t=%.2f, wx=%.2f, NOT reachable and no fallback found",
            t, wx);
        }
      }
    }
  }

  if (found) {
    RCLCPP_INFO(node_->get_logger(),
      "Edge waypoint found: [%.2f, %.2f] (t=%.2f)",
      out_waypoint.pose.position.x, out_waypoint.pose.position.y, best_t);
  } else {
    RCLCPP_WARN(node_->get_logger(),
      "No reachable edge point found along direction [%.4f, %.4f]",
      dir_x, dir_y);
  }

  return found;
}

}  // namespace pb_nav2_behaviors

// BT Plugin Registration
BT_REGISTER_NODES(factory)
{
  factory.registerBuilder<pb_nav2_behaviors::ComputePathToPoseWithEdgeWaypoint>(
    "ComputePathToPoseWithEdgeWaypoint",
    [](const std::string & name, const BT::NodeConfiguration & conf) {
      return std::make_unique<pb_nav2_behaviors::ComputePathToPoseWithEdgeWaypoint>(
        name, "compute_path_to_pose", conf);
    });
}
