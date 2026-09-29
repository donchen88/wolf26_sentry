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

#ifndef FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
#define FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "example_interfaces/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "roborts_msgs/msg/chassis_cmd.hpp"

namespace fake_vel_transform
{
class FakeVelTransform : public rclcpp::Node
{
public:
  explicit FakeVelTransform(const rclcpp::NodeOptions & options);

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  void cmdSpinCallback(example_interfaces::msg::Float32::SharedPtr msg);

  void freezeAngleCallback(const std_msgs::msg::Bool::SharedPtr msg);

  void autoAimCallback(const geometry_msgs::msg::Point::SharedPtr msg);

  geometry_msgs::msg::Twist applyVelocityFilter(const geometry_msgs::msg::Twist & twist);

  static double clampWithStep(double current_value, double target_value, double max_step);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr cmd_spin_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr freeze_angle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr auto_aim_target_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_pub_;

  // Broadcast tf from robot_base to robot_base_fake
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 用于发送chassis_cmd消息的发布对象
  rclcpp::Publisher<roborts_msgs::msg::ChassisCmd>::SharedPtr pubChassis;

  std::string robot_base_frame_;
  std::string fake_robot_base_frame_;
  std::string odom_topic_;
  std::string cmd_spin_topic_;
  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  float spin_speed_;

  bool enable_velocity_filter_;
  double filter_time_constant_;
  double linear_speed_deadband_;  
  double angular_speed_deadband_;
  double max_linear_accel_;
  double max_angular_accel_;

  geometry_msgs::msg::Twist filtered_cmd_vel_;
  rclcpp::Time last_filter_stamp_;
  bool filter_initialized_;

  // 速度突变检测与日志
  bool enable_velocity_jump_check_;
  double velocity_jump_linear_threshold_;
  double velocity_jump_angular_threshold_;
  double velocity_direction_jump_threshold_;
  double direction_valid_speed_threshold_;
  bool has_last_cmd_vel_;
  geometry_msgs::msg::Twist last_raw_cmd_vel_;
  rclcpp::Time last_cmd_vel_stamp_;

  std::mutex state_mutex_;

  double current_robot_base_angle_;

  bool angle_frozen_;
  double frozen_angle_;
  bool has_frozen_angle_;

  bool has_auto_aim_target_;
  rclcpp::Time last_auto_aim_stamp_;

  // 自瞄模式下禁用旋转相关
  bool auto_aim_disable_rotation_;

  // 速度自适应相关
  double gimbal_angle_change_threshold_;  // 角度变化阈值 (rad/s)
  double speed_reduction_factor_;         // 减速系数 (0.0-1.0)
  double recovery_time_;                  // 恢复时间 (s)
  double current_speed_scale_;            // 当前速度缩放
  rclcpp::Time gimbal_stable_time_;      // 云台稳定时刻
  double last_gimbal_angle_;             // 上一时刻云台角度
  bool gimbal_unstable_;                 // 云台是否处于不稳定状态
};

}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
