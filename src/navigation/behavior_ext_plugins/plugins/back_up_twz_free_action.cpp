// Copyright (c) 2022 Joshua Wallace
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

#include "behavior_ext_plugins/back_up_twz_free_action.hpp"
#include <cmath>
#include <limits>

namespace nav2_behaviors
{
  void BackUpTwzFree::onConfigure()
  {
    auto node = this->node_.lock();
    if (!node)
    {
      throw std::runtime_error{"Failed to lock node"};
    }

    nav2_util::declare_parameter_if_not_declared(
      node,
      "robot_radius", rclcpp::ParameterValue(0.1));
    node->get_parameter("robot_radius", robot_radius_);

    nav2_util::declare_parameter_if_not_declared(
      node,
      "max_radius", rclcpp::ParameterValue(1.0));
    node->get_parameter("max_radius", max_radius_);

    if(max_radius_ < robot_radius_)
    {
      RCLCPP_WARN(node->get_logger(), "max_radius is smaller than robot_radius. Setting max_radius to robot_radius");
      max_radius_ = robot_radius_;
    }

    nav2_util::declare_parameter_if_not_declared(
      node,
      "service_name", rclcpp::ParameterValue(std::string("local_costmap/get_costmap")));
    node->get_parameter("service_name", service_name_);

    nav2_util::declare_parameter_if_not_declared(
      node,
      "free_threshold", rclcpp::ParameterValue(5));
    node->get_parameter("free_threshold", free_threshold_);

    nav2_util::declare_parameter_if_not_declared(
      node,
      "cost_threshold", rclcpp::ParameterValue(0.1));
    node->get_parameter("cost_threshold", cost_threshold_);

    nav2_util::declare_parameter_if_not_declared(
      node,
      "visualization", rclcpp::ParameterValue(false));
    node->get_parameter("visualization", visualization_);
    
    costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);
    marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("back_up_twz_free_markers", 1);

    RCLCPP_DEBUG(node->get_logger(), "back_up_twz_free_action plugin initialized.");
  }

  Status BackUpTwzFree::onRun(const std::shared_ptr<const BackUpAction::Goal> command)
  {
    auto node = this->node_.lock();
    if (!node)
    {
      throw std::runtime_error{"Failed to lock node"};
    }

    // send request to get costmap

    while (!costmap_client_->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return Status::FAILED;
      }
      RCLCPP_WARN(node->get_logger(), "service not available, waiting again...");
    }

    auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
    auto result = costmap_client_->async_send_request(request);
    if(result.wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return Status::FAILED;
    }

    RCLCPP_DEBUG(node->get_logger(), "Got costmap");

    // get costmap
    auto costmap = result.get()->map;

    // Get costmap frame (local_costmap uses 'odom' as global_frame)
    std::string costmap_frame = costmap.header.frame_id;

    if (!nav2_util::getCurrentPose(
            initial_pose_, *tf_, costmap_frame, robot_base_frame_,
            transform_tolerance_))
    {
      RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
      return Status::FAILED;
    }

    // move towards free space
    // get current pose
    auto pose_x = initial_pose_.pose.position.x;
    auto pose_y = initial_pose_.pose.position.y;
    auto yaw = tf2::getYaw(initial_pose_.pose.orientation);

    // Calculate goal direction (the direction we ultimately want to go towards)
    double goal_x = command->target.x;
    double goal_y = command->target.y;
    double goal_direction = std::atan2(goal_y - pose_y, goal_x - pose_x);

    RCLCPP_INFO(node->get_logger(), "Goal direction: %.1f deg (%.2f, %.2f) -> (%.2f, %.2f)",
                 goal_direction * 180.0 / M_PI, pose_x, pose_y, goal_x, goal_y);

    // Search for minimum cost direction: sample rays in all directions and find the one with lowest cost
    const int num_directions = 360;  // Sample every 1 degree
    const double ray_length = max_radius_;
    const int rays_per_direction = 20;  // More points for better resolution
    const double min_sample_dist = 0.05;  // Start from close to robot, not from robot_radius_

    double best_direction = yaw;
    double best_cost = std::numeric_limits<double>::max();
    double best_distance = 0.0;
    double best_avg_cost = 255.0;
    double best_gradient = 0.0;

    std::vector<geometry_msgs::msg::Point> best_points;

    for (int dir_idx = 0; dir_idx < num_directions; dir_idx++)
    {
      double angle = dir_idx * 2.0 * M_PI / num_directions;
      double sum_cost = 0.0;
      double max_cost = 0.0;
      int valid_points = 0;
      double exit_distance = 0.0;
      double min_cost_along_ray = 255.0;
      std::vector<geometry_msgs::msg::Point> ray_points;

      // For gradient calculation
      double inner_cost_sum = 0.0, outer_cost_sum = 0.0;
      int inner_count = 0, outer_count = 0;
      std::vector<unsigned char> ray_costs;  // Store costs along the ray for gradient calculation

      for (int ray_idx = 1; ray_idx <= rays_per_direction; ray_idx++)
      {
        // Sample from close to robot outwards
        double dist = min_sample_dist + (ray_length - min_sample_dist) * ray_idx / rays_per_direction;
        double px = pose_x + dist * std::cos(angle);
        double py = pose_y + dist * std::sin(angle);

        // Convert world coords to costmap grid
        int mx = static_cast<int>((px - costmap.metadata.origin.position.x) / costmap.metadata.resolution);
        int my = static_cast<int>((py - costmap.metadata.origin.position.y) / costmap.metadata.resolution);

        if (mx < 0 || mx >= static_cast<int>(costmap.metadata.size_x) ||
            my < 0 || my >= static_cast<int>(costmap.metadata.size_y))
        {
          continue;
        }

        auto costmap_index = mx + my * costmap.metadata.size_x;
        unsigned char cost = costmap.data[costmap_index];

        ray_points.push_back(geometry_msgs::msg::Point());
        ray_points.back().x = px;
        ray_points.back().y = py;

        sum_cost += cost;
        max_cost = std::max(max_cost, static_cast<double>(cost));
        min_cost_along_ray = std::min(min_cost_along_ray, static_cast<double>(cost));
        valid_points++;
        ray_costs.push_back(cost);

        // Track first free space (exit point)
        if (cost < cost_threshold_ && exit_distance == 0.0)
        {
          exit_distance = dist;
        }

        // If we hit lethal obstacle, stop this ray
        if (cost >= 253)
        {
          break;
        }
      }

      // Calculate gradient: inner (close) vs outer (far) cost
      // If outer_cost < inner_cost, the direction leads to free space (positive gradient)
      int mid_point = ray_costs.size() / 2;
      for (size_t i = 0; i < ray_costs.size(); i++) {
        if (i < mid_point) {
          inner_cost_sum += ray_costs[i];
          inner_count++;
        } else {
          outer_cost_sum += ray_costs[i];
          outer_count++;
        }
      }
      double gradient = 0.0;
      if (inner_count > 0 && outer_count > 0) {
        double inner_avg = inner_cost_sum / inner_count;
        double outer_avg = outer_cost_sum / outer_count;
        gradient = inner_avg - outer_avg;  // Positive = cost decreases outward = good
      }

      // Calculate score for this direction
      double score = 1000.0;  // Default high score for invalid directions

      if (valid_points > 0)
      {
        double avg_cost = sum_cost / valid_points;

        // Core score based on minimum cost along ray (find thinnest obstacle)
        score = 100.0 * min_cost_along_ray / 253.0;  // Normalize: 0 (free) to ~100 (lethal)

        // Bonus for reaching free space (closer exit = better)
        if (exit_distance > 0.0)
        {
          score -= 30.0 * std::max(0.0, 1.0 - exit_distance / ray_length);
        }
        else
        {
          // No free space found - penalize based on max cost
          score += 50.0 * (max_cost / 253.0);
        }

        // Bonus for positive gradient (cost decreasing outward)
        // Positive gradient means we're moving toward free space
        if (gradient > 0) {
          score -= 40.0 * std::min(1.0, gradient / 100.0);  // Max bonus of 40
        } else {
          score += 40.0 * std::min(1.0, -gradient / 100.0);  // Penalty for negative gradient
        }

        // Bonus for directions closer to goal direction
        double angle_diff = std::abs(angle - goal_direction);
        if (angle_diff > M_PI)
        {
          angle_diff = 2.0 * M_PI - angle_diff;
        }
        double angle_bonus = 20.0 * std::max(0.0, 1.0 - 2.0 * angle_diff / M_PI);
        score -= angle_bonus;
      }

      if (score < best_cost)
      {
        best_cost = score;
        best_direction = angle;
        best_avg_cost = valid_points > 0 ? sum_cost / valid_points : 255.0;
        best_distance = exit_distance > 0.0 ? exit_distance : ray_length;
        best_gradient = gradient;
        best_points = ray_points;
      }
    }

    RCLCPP_INFO(node->get_logger(), "Best escape direction: %.1f deg, score: %.2f, gradient: %.1f, avg_cost: %.2f, distance: %.2f",
                 best_direction * 180.0 / M_PI, best_cost, best_gradient, best_avg_cost, best_distance);

    // Calculate target point along the best direction
    double target_dist = std::min(best_distance, max_radius_);
    auto avg_x = pose_x + target_dist * std::cos(best_direction);
    auto avg_y = pose_y + target_dist * std::sin(best_direction);

    RCLCPP_WARN(node->get_logger(), "avg_x: %f, avg_y: %f", avg_x, avg_y);

    // visualize best escape direction and points
    if(visualization_){
      visualization_msgs::msg::MarkerArray markers;
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = global_frame_;
      marker.header.stamp = node->now();
      marker.ns = "escape_points";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::POINTS;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = costmap.metadata.resolution;
      marker.scale.y = costmap.metadata.resolution;
      marker.color.r = 1.0;
      marker.color.a = 1.0;
      for (auto i = 0; i < best_points.size(); i++)
      {
        marker.points.push_back(best_points[i]);
      }
      markers.markers.push_back(marker);
      visualization_msgs::msg::Marker destination_marker;
      destination_marker.header.frame_id = global_frame_;
      destination_marker.header.stamp = node->now();
      destination_marker.ns = "destination";
      destination_marker.id = 0;
      destination_marker.type = visualization_msgs::msg::Marker::POINTS;
      destination_marker.action = visualization_msgs::msg::Marker::ADD;
      destination_marker.pose.orientation.w = 1.0;
      destination_marker.scale.x = costmap.metadata.resolution;
      destination_marker.scale.y = costmap.metadata.resolution;
      destination_marker.color.g = 1.0;
      destination_marker.color.a = 1.0;
      destination_marker.points.push_back(geometry_msgs::msg::Point());
      destination_marker.points.back().x = avg_x;
      destination_marker.points.back().y = avg_y;
      markers.markers.push_back(destination_marker);
      marker_pub_->publish(markers);
    }
    
    // calculate angle to free space
    auto angle_to_free_space = std::atan2(avg_y - pose_y, avg_x - pose_x);
    auto angle_diff = angle_to_free_space - yaw;
    if (angle_diff > M_PI)
    {
      angle_diff -= 2 * M_PI;
    }
    else if (angle_diff < -M_PI)
    {
      angle_diff += 2 * M_PI;
    }
    RCLCPP_WARN(node->get_logger(), "angle_diff: %f deg", angle_diff*180/M_PI);

    // calculate move command
    twist_x_ = std::cos(angle_diff) * command->speed;
    twist_y_ = std::sin(angle_diff) * command->speed;
    command_x_ = command->target.x;
    command_time_allowance_ = command->time_allowance;

    end_time_ = this->clock_->now() + command_time_allowance_;

    if (!nav2_util::getCurrentPose(
            initial_pose_, *tf_, global_frame_, robot_base_frame_,
            transform_tolerance_))
    {
      RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
      return Status::FAILED;
    }
    RCLCPP_WARN(
        this->logger_, "backing up %f meters towards free space at angle %f", command_x_, angle_diff);

    return Status::SUCCEEDED;
  }

  Status BackUpTwzFree::onCycleUpdate()
  {
    rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
    if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0)
    {
      this->stopRobot();
      RCLCPP_WARN(
          this->logger_,
          "Exceeded time allowance before reaching the DriveOnHeading goal - Exiting DriveOnHeading");
      return Status::FAILED;
    }

    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(
            current_pose, *this->tf_, this->global_frame_, this->robot_base_frame_,
            this->transform_tolerance_))
    {
      RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
      return Status::FAILED;
    }

    double diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
    double diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
    double distance = hypot(diff_x, diff_y);

    feedback_->distance_traveled = distance;
    this->action_server_->publish_feedback(feedback_);

    if (distance >= std::fabs(command_x_))
    {
      this->stopRobot();
      return Status::SUCCEEDED;
    }

    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel->linear.y = twist_y_;
    cmd_vel->linear.x = twist_x_;

    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = current_pose.pose.position.x;
    pose2d.y = current_pose.pose.position.y;
    pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

    if (!isCollisionFree(distance, cmd_vel.get(), pose2d))
    {
      this->stopRobot();
      RCLCPP_WARN(this->logger_, "Collision Ahead - Exiting DriveOnHeading");
      return Status::FAILED;
    }

    this->vel_pub_->publish(std::move(cmd_vel));

    return Status::RUNNING;
  }

} // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::BackUpTwzFree, nav2_core::Behavior)