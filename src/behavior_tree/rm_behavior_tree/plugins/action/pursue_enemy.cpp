#include "rm_behavior_tree/plugins/action/pursue_enemy.hpp"
#include "rm_behavior_tree/map_accessor.hpp"
#include "rm_behavior_tree/visibility_checker.hpp"
#include "rm_behavior_tree/key_point_loader.hpp"

#include <cmath>
#include <algorithm>
#include <limits>
#include <chrono>
#include <mutex>
#include <optional>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

using namespace std::chrono_literals;

namespace rm_behavior_tree
{
static std::atomic<int> g_pursue_onstart_count(0);

PursueEnemyAction::PursueEnemyAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, config), ros_node_(params.nh)
{
  pursue_radius_ = 2.0;
  update_interval_ = 0.5;
  action_name_ = "navigate_to_pose";
  target_frame_ = "map";
  robot_base_frame_ = "gimbal_yaw";

  pursue_distance_threshold_ = 0.5;
  max_pursuit_points_ = 10;
  pursuit_points_sent_ = 0;
  has_received_armor_ = false;
  enemy_timeout_ = 10.0;

  goal_sent_ = false;
  navigation_complete_ = false;
  arrived_at_enemy_ = false;
  last_enemy_pos_ = geometry_msgs::msg::Point();
  last_enemy_pos_set_ = false;
  last_sent_enemy_pos_ = geometry_msgs::msg::Point();
  last_sent_enemy_pos_set_ = false;
  last_result_code_ = rclcpp_action::ResultCode::UNKNOWN;
  last_update_time_ = std::chrono::steady_clock::now();
  nav_action_client_ = nullptr;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros_node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  wall_check_clearance_ = 0.0;
  map_topic_ = "/map";
  map_sub_ = nullptr;
  latest_map_ = nullptr;
}

BT::NodeStatus PursueEnemyAction::onStart()
{
  ++g_pursue_onstart_count;
  try {
    if (!ros_node_) {
      RCLCPP_ERROR(rclcpp::get_logger("pursue_enemy"), "PursueEnemy: ROS node is null");
      return BT::NodeStatus::FAILURE;
    }

    if (auto radius_res = getInput<double>("pursue_radius")) {
      pursue_radius_ = radius_res.value();
    }
    if (auto threshold_res = getInput<double>("pursue_distance_threshold")) {
      pursue_distance_threshold_ = threshold_res.value();
    }
    if (auto max_points_res = getInput<int>("max_pursuit_points")) {
      max_pursuit_points_ = max_points_res.value();
    }
    if (auto interval_res = getInput<double>("update_interval")) {
      update_interval_ = interval_res.value();
    }
    if (auto timeout_res = getInput<double>("enemy_timeout")) {
      enemy_timeout_ = timeout_res.value();
    }
    if (auto action_res = getInput<std::string>("action_name")) {
      action_name_ = action_res.value();
    }
    if (auto frame_res = getInput<std::string>("target_frame")) {
      target_frame_ = frame_res.value();
    }
    if (auto base_res = getInput<std::string>("robot_base_frame")) {
      robot_base_frame_ = base_res.value();
    }
    if (auto map_topic_res = getInput<std::string>("map_topic")) {
      map_topic_ = map_topic_res.value();
    }
    if (auto clearance_res = getInput<double>("wall_check_clearance")) {
      wall_check_clearance_ = clearance_res.value();
    }

    auto armors = getInput<auto_aim_interfaces::msg::Armors>("armors");
    if (!armors || armors->armors.empty()) {
      RCLCPP_WARN(ros_node_->get_logger(), "PursueEnemy: No enemies detected");
      return BT::NodeStatus::FAILURE;
    }

    has_received_armor_ = true;
    enemy_timeout_ = 10.0;
    last_armor_stamp_ = ros_node_->now();
    first_armor_stamp_ = last_armor_stamp_;
    pursuit_points_sent_ = 0;

    enemy_pos_history_.clear();

    auto robot_location = getInput<geometry_msgs::msg::TransformStamped>("current_location");
    if (!robot_location) {
      RCLCPP_WARN(ros_node_->get_logger(), "PursueEnemy: Missing required input [current_location]");
      return BT::NodeStatus::FAILURE;
    }

    if (wall_check_clearance_ > 0.0) {
      initMapSubscription();
      auto enemy_pos = getEnemyPosition(armors.value());
      geometry_msgs::msg::Point robot_pos;
      robot_pos.x = robot_location.value().transform.translation.x;
      robot_pos.y = robot_location.value().transform.translation.y;
      robot_pos.z = robot_location.value().transform.translation.z;

      if (hasObstacleBetweenRobotAndEnemy(robot_pos, enemy_pos)) {
        RCLCPP_WARN(ros_node_->get_logger(),
                    "PursueEnemy: Wall detected between robot and enemy, skipping pursuit");
        return BT::NodeStatus::FAILURE;
      }
    }

    if (!nav_action_client_) {
      nav_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        ros_node_, action_name_);
    }

    current_goal_ = calculatePursuePose(armors.value(), robot_location.value());

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = current_goal_;
    goal_msg.pose.header.frame_id = target_frame_;
    goal_msg.pose.header.stamp = ros_node_->now();

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](std::shared_ptr<GoalHandleNav> handle) {
      goal_handle_ = handle;
    };

    send_goal_options.result_callback = [this](const GoalHandleNav::WrappedResult & result) {
      last_result_code_ = result.code;
      navigation_complete_ = true;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        arrived_at_enemy_ = true;
      }
    };

    auto future_goal_handle = nav_action_client_->async_send_goal(goal_msg, send_goal_options);

    auto enemy_pos_start = getEnemyPosition(armors.value());
    last_enemy_pos_ = enemy_pos_start;
    last_enemy_pos_set_ = true;
    last_sent_enemy_pos_ = enemy_pos_start;
    last_sent_enemy_pos_set_ = true;
    pursuit_points_sent_ = 1;

    goal_sent_ = true;
    navigation_complete_ = false;
    arrived_at_enemy_ = false;
    last_update_time_ = std::chrono::steady_clock::now();
    last_result_code_ = rclcpp_action::ResultCode::UNKNOWN;

    return BT::NodeStatus::RUNNING;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(ros_node_->get_logger(), "PursueEnemy onStart EXCEPTION: %s", e.what());
    return BT::NodeStatus::FAILURE;
  } catch (...) {
    RCLCPP_ERROR(ros_node_->get_logger(), "PursueEnemy onStart UNKNOWN EXCEPTION");
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus PursueEnemyAction::onRunning()
{
  auto armors = getInput<auto_aim_interfaces::msg::Armors>("armors");
  auto robot_location = getInput<geometry_msgs::msg::TransformStamped>("current_location");

  if (!robot_location) {
    RCLCPP_WARN(ros_node_->get_logger(), "PursueEnemy: Missing current_location, stopping");
    if (goal_handle_.has_value() && goal_sent_) {
      nav_action_client_->async_cancel_goal(goal_handle_.value());
    }
    return BT::NodeStatus::FAILURE;
  }

  bool enemy_lost = false;
  if (!armors || armors->armors.empty()) {
    enemy_lost = true;
  } else {
    const auto & a = armors->armors[0];
    bool pos_zero = (std::abs(a.pose.position.x) < 0.001 &&
                      std::abs(a.pose.position.y) < 0.001 &&
                      std::abs(a.pose.position.z) < 0.001);
    if (pos_zero) {
      enemy_lost = true;
    } else if (has_received_armor_) {
      auto time_since_last_armor = (ros_node_->now() - last_armor_stamp_).seconds();
      if (time_since_last_armor > enemy_timeout_) {
        enemy_lost = true;
      }
    }
  }

  if (enemy_lost) {
  RCLCPP_WARN_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 5000,
      "PursueEnemy: Enemy lost, canceling goal and returning FAILURE");
    if (goal_handle_.has_value() && goal_sent_) {
      nav_action_client_->async_cancel_goal(goal_handle_.value());
    }
    return BT::NodeStatus::FAILURE;
  }

  last_armor_stamp_ = ros_node_->now();
  auto current_time = std::chrono::steady_clock::now();

  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    current_time - last_update_time_).count();

  // 需要时才计算敌人位置和滤波（用于墙检查或发目标）
  if (wall_check_clearance_ > 0.0 || elapsed_ms >= update_interval_ * 1000) {
    geometry_msgs::msg::Point enemy_pos = getEnemyPosition(armors.value());

    // 滑动窗口滤波：入队，取均值
    enemy_pos_history_.push_back(enemy_pos);
    if (enemy_pos_history_.size() > kEnemyHistorySize) {
      enemy_pos_history_.pop_front();
    }
    geometry_msgs::msg::Point filtered_enemy_pos = getFilteredEnemyPos();

    RCLCPP_INFO_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 5000,
        "PursueEnemy: raw=(%.3f,%.3f) filtered=(%.3f,%.3f) queue=%zu",
        enemy_pos.x, enemy_pos.y,
        filtered_enemy_pos.x, filtered_enemy_pos.y,
        enemy_pos_history_.size());

    // 墙检查（仅在启用时）
    if (wall_check_clearance_ > 0.0) {
      geometry_msgs::msg::Point robot_pos;
      robot_pos.x = robot_location.value().transform.translation.x;
      robot_pos.y = robot_location.value().transform.translation.y;
      robot_pos.z = robot_location.value().transform.translation.z;

      if (hasObstacleBetweenRobotAndEnemy(robot_pos, enemy_pos)) {
        if (goal_handle_.has_value() && goal_sent_) {
          nav_action_client_->async_cancel_goal(goal_handle_.value());
        }
        return BT::NodeStatus::FAILURE;
      }
    }

    // 未到间隔，只更新滤波历史，不发目标
    if (elapsed_ms < update_interval_ * 1000) {
      return BT::NodeStatus::RUNNING;
    }

    if (pursuit_points_sent_ >= max_pursuit_points_) {
      if (goal_handle_.has_value() && goal_sent_) {
        nav_action_client_->async_cancel_goal(goal_handle_.value());
      }
      return BT::NodeStatus::SUCCESS;
    }

    // 计算追击目标点
    auto new_goal = calculatePursuePoseWithFilteredPos(filtered_enemy_pos, robot_location.value());

    // 只在目标点变化时打印
    bool goal_changed = (current_goal_.pose.position.x != new_goal.pose.position.x ||
                         current_goal_.pose.position.y != new_goal.pose.position.y);
    if (goal_changed) {
      RCLCPP_INFO(ros_node_->get_logger(),
          "PursueEnemy: Sending goal #%d: (%.3f, %.3f)",
          pursuit_points_sent_ + 1,
          new_goal.pose.position.x, new_goal.pose.position.y);
    }

    if (goal_handle_.has_value() && goal_sent_) {
      nav_action_client_->async_cancel_goal(goal_handle_.value());
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = new_goal;
    goal_msg.pose.header.frame_id = target_frame_;
    goal_msg.pose.header.stamp = ros_node_->now();

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](std::shared_ptr<GoalHandleNav> handle) {
      goal_handle_ = handle;
    };
    send_goal_options.result_callback = [this](const GoalHandleNav::WrappedResult & result) {
      last_result_code_ = result.code;
      navigation_complete_ = true;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        arrived_at_enemy_ = true;
      }
    };

    nav_action_client_->async_send_goal(goal_msg, send_goal_options);
    goal_sent_ = true;
    current_goal_ = new_goal;
    last_update_time_ = current_time;
    pursuit_points_sent_++;
  }

  return BT::NodeStatus::RUNNING;
}

void PursueEnemyAction::onHalted()
{
  if (goal_handle_.has_value() && goal_sent_) {
    nav_action_client_->async_cancel_goal(goal_handle_.value());
  }
  goal_sent_ = false;
  navigation_complete_ = false;
  arrived_at_enemy_ = false;
  last_result_code_ = rclcpp_action::ResultCode::UNKNOWN;
}

geometry_msgs::msg::Point PursueEnemyAction::getEnemyPosition(
  const auto_aim_interfaces::msg::Armors & armors)
{
  if (armors.armors.empty()) {
    geometry_msgs::msg::Point empty;
    return empty;
  }

  auto robot_location = getInput<geometry_msgs::msg::TransformStamped>("current_location");
  if (!robot_location) {
    return armors.armors[0].pose.position;
  }

  geometry_msgs::msg::Point robot_pos;
  robot_pos.x = robot_location->transform.translation.x;
  robot_pos.y = robot_location->transform.translation.y;
  robot_pos.z = robot_location->transform.translation.z;

  double min_distance = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point closest_enemy;

  for (const auto & armor : armors.armors) {
    // SubTargetPos 已经把 armor 转换到 map/odom 了（target_frame）
    // robot_location 也是 map/odom 帧，直接用，不需要再转换
    geometry_msgs::msg::Point enemy_pos_map;
    enemy_pos_map.x = armor.pose.position.x;
    enemy_pos_map.y = armor.pose.position.y;
    enemy_pos_map.z = armor.pose.position.z;

    double dx = enemy_pos_map.x - robot_pos.x;
    double dy = enemy_pos_map.y - robot_pos.y;
    double dz = enemy_pos_map.z - robot_pos.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distance < min_distance) {
      min_distance = distance;
      closest_enemy = enemy_pos_map;
    }
  }

  return closest_enemy;
}

geometry_msgs::msg::PoseStamped PursueEnemyAction::generatePursuePose(
  const geometry_msgs::msg::Point & enemy_pos,
  const geometry_msgs::msg::Point & robot_pos,
  double radius)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = target_frame_;
  pose.header.stamp = ros_node_->now();

  double dx = robot_pos.x - enemy_pos.x;
  double dy = robot_pos.y - enemy_pos.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  if (distance < 1e-6) {
    pose.pose.position.x = enemy_pos.x + radius;
    pose.pose.position.y = enemy_pos.y;
    pose.pose.position.z = robot_pos.z;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    pose.pose.orientation = tf2::toMsg(q);
    return pose;
  }

  double ratio = radius / distance;
  pose.pose.position.x = enemy_pos.x + dx * ratio;
  pose.pose.position.y = enemy_pos.y + dy * ratio;
  pose.pose.position.z = robot_pos.z;

  double yaw = std::atan2(dy, dx);
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  pose.pose.orientation = tf2::toMsg(q);

  return pose;
}

geometry_msgs::msg::PoseStamped PursueEnemyAction::calculatePursuePose(
  const auto_aim_interfaces::msg::Armors & armors,
  const geometry_msgs::msg::TransformStamped & robot_location)
{
  geometry_msgs::msg::Point enemy_pos = getEnemyPosition(armors);

  geometry_msgs::msg::Point robot_pos;
  robot_pos.x = robot_location.transform.translation.x;
  robot_pos.y = robot_location.transform.translation.y;
  robot_pos.z = robot_location.transform.translation.z;

  geometry_msgs::msg::PoseStamped goal_camera_init = generatePursuePose(enemy_pos, robot_pos, pursue_radius_);
  goal_camera_init.header.frame_id = robot_location.header.frame_id;

  if (target_frame_ == robot_location.header.frame_id) {
    return goal_camera_init;
  }

  geometry_msgs::msg::PoseStamped goal_target;
  try {
    if (tf_buffer_) {
      auto tf_cam_to_tgt = tf_buffer_->lookupTransform(
        target_frame_,
        robot_location.header.frame_id,
        rclcpp::Time(0),
        tf2::durationFromSec(0.5));
      tf2::doTransform(goal_camera_init, goal_target, tf_cam_to_tgt);
    } else {
      goal_target = goal_camera_init;
    }
  } catch (const tf2::TransformException & ex) {
    goal_target = goal_camera_init;
  }

  return goal_target;
}

geometry_msgs::msg::PoseStamped PursueEnemyAction::calculatePursuePoseWithFilteredPos(
  const geometry_msgs::msg::Point & filtered_enemy_pos,
  const geometry_msgs::msg::TransformStamped & robot_location)
{
  geometry_msgs::msg::Point robot_pos;
  robot_pos.x = robot_location.transform.translation.x;
  robot_pos.y = robot_location.transform.translation.y;
  robot_pos.z = robot_location.transform.translation.z;

  geometry_msgs::msg::PoseStamped goal_camera_init = generatePursuePose(filtered_enemy_pos, robot_pos, pursue_radius_);
  goal_camera_init.header.frame_id = robot_location.header.frame_id;

  if (target_frame_ == robot_location.header.frame_id) {
    return goal_camera_init;
  }

  geometry_msgs::msg::PoseStamped goal_target;
  try {
    if (tf_buffer_) {
      auto tf_cam_to_tgt = tf_buffer_->lookupTransform(
        target_frame_,
        robot_location.header.frame_id,
        rclcpp::Time(0),
        tf2::durationFromSec(0.5));
      tf2::doTransform(goal_camera_init, goal_target, tf_cam_to_tgt);
    } else {
      goal_target = goal_camera_init;
    }
  } catch (const tf2::TransformException & ex) {
    goal_target = goal_camera_init;
  }

  return goal_target;
}

void PursueEnemyAction::initMapSubscription()
{
  if (wall_check_clearance_ <= 0.0) {
    return;
  }

  if (map_sub_) {
    return;
  }

  RCLCPP_DEBUG(ros_node_->get_logger(), "PursueEnemy: Subscribing to map topic: %s", map_topic_.c_str());

  map_sub_ = ros_node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_,
    rclcpp::QoS(5).best_effort(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(map_mutex_);
      latest_map_ = msg;
      RCLCPP_INFO(ros_node_->get_logger(), "PursueEnemy: Map received: %dx%d, resolution=%.3f",
          msg->info.width, msg->info.height, msg->info.resolution);
    });
}

bool PursueEnemyAction::hasObstacleBetweenRobotAndEnemy(
  const geometry_msgs::msg::Point & robot_pos,
  const geometry_msgs::msg::Point & enemy_pos)
{
  if (wall_check_clearance_ <= 0.0) {
    return false;
  }

  RCLCPP_DEBUG(ros_node_->get_logger(),
      "PursueEnemy: hasObstacle CHECK: Robot(%.2f,%.2f) Enemy(%.2f,%.2f) wall_check_clearance=%.2f",
      robot_pos.x, robot_pos.y, enemy_pos.x, enemy_pos.y, wall_check_clearance_);

  std::lock_guard<std::mutex> lock(map_mutex_);
  if (!latest_map_) {
    RCLCPP_WARN(ros_node_->get_logger(),
        "PursueEnemy: No map yet on topic '%s', skipping wall check", map_topic_.c_str());
    return false;
  }

  MapAccessor map_accessor(*latest_map_);

  double resolution = map_accessor.getResolution();
  if (resolution <= 0) {
    resolution = 0.05;
  }
  int min_clearance_cells = static_cast<int>(wall_check_clearance_ / resolution);

  KeyPoint robot_keypoint;
  robot_keypoint.x = robot_pos.x;
  robot_keypoint.y = robot_pos.y;

  KeyPoint enemy_keypoint;
  enemy_keypoint.x = enemy_pos.x;
  enemy_keypoint.y = enemy_pos.y;

  // 调试：检查起点和终点是否在地图范围内
  int mx0, my0, mx1, my1;
  bool robot_in_map = map_accessor.worldToMap(robot_pos.x, robot_pos.y, mx0, my0);
  bool enemy_in_map = map_accessor.worldToMap(enemy_pos.x, enemy_pos.y, mx1, my1);

  // 调试：查看机器人位置和敌人位置的栅格值（每次都打印）
  if (robot_in_map && enemy_in_map) {
    int robot_cell = latest_map_->data[my0 * latest_map_->info.width + mx0];
    int enemy_cell = latest_map_->data[my1 * latest_map_->info.width + mx1];
    RCLCPP_DEBUG(ros_node_->get_logger(),
        "PursueEnemy: Map check - Robot(%.2f,%.2f)[%d,%d]=%d Enemy(%.2f,%.2f)[%d,%d]=%d clearance=%d",
        robot_pos.x, robot_pos.y, mx0, my0, robot_cell,
        enemy_pos.x, enemy_pos.y, mx1, my1, enemy_cell,
        min_clearance_cells);
  }

  if (!robot_in_map || !enemy_in_map) {
    return false;  // 坐标不在地图范围内时放行
  }

  bool visible = VisibilityChecker::isVisible(robot_keypoint, enemy_keypoint, map_accessor, min_clearance_cells);

  if (!visible) {
    RCLCPP_WARN(ros_node_->get_logger(),
        "PursueEnemy: [WALL DETECTED] Robot(%.2f,%.2f) Enemy(%.2f,%.2f) - skipping pursuit",
        robot_pos.x, robot_pos.y, enemy_pos.x, enemy_pos.y);
  }

  return !visible;
}

geometry_msgs::msg::Point PursueEnemyAction::getFilteredEnemyPos()
{
  if (enemy_pos_history_.empty()) {
    geometry_msgs::msg::Point empty;
    return empty;
  }
  double sum_x = 0, sum_y = 0, sum_z = 0;
  for (const auto & p : enemy_pos_history_) {
    sum_x += p.x;
    sum_y += p.y;
    sum_z += p.z;
  }
  geometry_msgs::msg::Point filtered;
  filtered.x = sum_x / enemy_pos_history_.size();
  filtered.y = sum_y / enemy_pos_history_.size();
  filtered.z = sum_z / enemy_pos_history_.size();
  return filtered;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::PursueEnemyAction, "PursueEnemy");
