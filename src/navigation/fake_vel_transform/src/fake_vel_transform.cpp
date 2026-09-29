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

#include "fake_vel_transform/fake_vel_transform.hpp"

#include <algorithm>
#include <cmath>

#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{
FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  this->declare_parameter<std::string>("robot_base_frame", "gimbal_link");
  this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_link_fake");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");
  this->declare_parameter<float>("init_spin_speed", 0.0);
  this->declare_parameter<bool>("enable_velocity_filter", true);
  this->declare_parameter<double>("filter_time_constant", 0.03);
  this->declare_parameter<double>("linear_speed_deadband", 0.06);
  this->declare_parameter<double>("angular_speed_deadband", 0.05);
  this->declare_parameter<double>("max_linear_accel", 5.0);
  this->declare_parameter<double>("max_angular_accel", 6.0);
  this->declare_parameter<bool>("enable_velocity_jump_check", true);
  this->declare_parameter<double>("velocity_jump_linear_threshold", 1.0);
  this->declare_parameter<double>("velocity_jump_angular_threshold", 1.0);
  this->declare_parameter<double>("velocity_direction_jump_threshold", 0.8);
  this->declare_parameter<double>("direction_valid_speed_threshold", 0.05);
  this->declare_parameter<bool>("auto_aim_disable_rotation", true);
  this->declare_parameter<double>("gimbal_angle_change_threshold", 5.0);
  this->declare_parameter<double>("speed_reduction_factor", 1.0);
  this->declare_parameter<double>("recovery_time", 0.5);

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("init_spin_speed", spin_speed_);
  this->get_parameter("enable_velocity_filter", enable_velocity_filter_);
  this->get_parameter("filter_time_constant", filter_time_constant_);
  this->get_parameter("linear_speed_deadband", linear_speed_deadband_);
  this->get_parameter("angular_speed_deadband", angular_speed_deadband_);
  this->get_parameter("max_linear_accel", max_linear_accel_);
  this->get_parameter("max_angular_accel", max_angular_accel_);
  this->get_parameter("enable_velocity_jump_check", enable_velocity_jump_check_);
  this->get_parameter("velocity_jump_linear_threshold", velocity_jump_linear_threshold_);
  this->get_parameter("velocity_jump_angular_threshold", velocity_jump_angular_threshold_);
  this->get_parameter("velocity_direction_jump_threshold", velocity_direction_jump_threshold_);
  this->get_parameter("direction_valid_speed_threshold", direction_valid_speed_threshold_);
  this->get_parameter("auto_aim_disable_rotation", auto_aim_disable_rotation_);
  this->get_parameter("gimbal_angle_change_threshold", gimbal_angle_change_threshold_);
  this->get_parameter("speed_reduction_factor", speed_reduction_factor_);
  this->get_parameter("recovery_time", recovery_time_);

  has_last_cmd_vel_ = false;

  angle_frozen_ = false;
  frozen_angle_ = 0.0;
  has_frozen_angle_ = false;

  has_auto_aim_target_ = false;
  last_auto_aim_stamp_ = this->get_clock()->now();
  auto_aim_disable_rotation_ = true;

  gimbal_angle_change_threshold_ = 0.5;
  speed_reduction_factor_ = 0.3;
  recovery_time_ = 0.5;
  current_speed_scale_ = 1.0;
  gimbal_stable_time_ = this->get_clock()->now();
  last_gimbal_angle_ = 0.0;
  gimbal_unstable_ = false;

  filter_initialized_ = false;
  current_robot_base_angle_ = 0.0;

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
    cmd_spin_topic_, 1, std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 1,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&FakeVelTransform::odomCallback, this, std::placeholders::_1));
  freeze_angle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "freeze_angle", 1,
    std::bind(&FakeVelTransform::freezeAngleCallback, this, std::placeholders::_1));
  auto_aim_target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
    "auto_aim_target_pos", 5,
    std::bind(&FakeVelTransform::autoAimCallback, this, std::placeholders::_1));
  // 消息类型为roborts_msgs::msg::ChassisCmd，队列大小为5，发送处理后的速度数据
  pubChassis = create_publisher<roborts_msgs::msg::ChassisCmd>("/chassis_cmd", 5);
}

void FakeVelTransform::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double new_angle;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    new_angle = tf2::getYaw(msg->pose.pose.orientation);

    // 计算云台角度变化率
    auto now = this->get_clock()->now();
    double dt = (now - gimbal_stable_time_).seconds();
    if (dt > 0.001) {
      double angle_change_rate = std::abs(new_angle - last_gimbal_angle_) / dt;

      if (angle_change_rate > gimbal_angle_change_threshold_) {
        // 云台变化太快，进入不稳定状态，减速
        if (!gimbal_unstable_) {
          RCLCPP_WARN(this->get_logger(),
            "[Gimbal] Angle change too fast (%.2f rad/s), reducing speed to %.0f%%",
            angle_change_rate, speed_reduction_factor_ * 100);
        }
        gimbal_unstable_ = true;
        gimbal_stable_time_ = now;
      } else if (gimbal_unstable_) {
        // 云台稳定了，等待恢复时间
        double stable_duration = (now - gimbal_stable_time_).seconds();
        if (stable_duration > recovery_time_) {
          RCLCPP_INFO(this->get_logger(), "[Gimbal] Angle stable, restoring speed");
          gimbal_unstable_ = false;
        }
      }

      // 更新速度缩放
      current_speed_scale_ = gimbal_unstable_ ? speed_reduction_factor_ : 1.0;
    }

    last_gimbal_angle_ = new_angle;
    current_robot_base_angle_ = new_angle;
  }

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = msg->header.stamp;
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  q.setRPY(0, 0, -current_robot_base_angle_);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

void FakeVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  spin_speed_ = msg->data;
}

void FakeVelTransform::freezeAngleCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (msg->data) {
    // 冻结当前角度
    angle_frozen_ = true;
    frozen_angle_ = current_robot_base_angle_;
    has_frozen_angle_ = true;
    RCLCPP_INFO(this->get_logger(), "Angle frozen at %.3f rad", frozen_angle_);
  } else {
    // 解除冻结
    angle_frozen_ = false;
    has_frozen_angle_ = false;
    RCLCPP_INFO(this->get_logger(), "Angle unfrozen");
  }
}

void FakeVelTransform::autoAimCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  bool has_target = (msg->x != 0.0 || msg->y != 0.0 || msg->z != 0.0);

  if (has_target) {
    if (!has_auto_aim_target_) {
      // 首次检测到目标，冻结当前底盘角度
      angle_frozen_ = true;
      frozen_angle_ = current_robot_base_angle_;
      has_frozen_angle_ = true;
      RCLCPP_INFO(this->get_logger(), "[AutoAim] Target detected, freeze angle at %.3f rad, rotation DISABLED", frozen_angle_);
    }
    has_auto_aim_target_ = true;
    last_auto_aim_stamp_ = this->get_clock()->now();
  } else {
    if (has_auto_aim_target_) {
      // 目标丢失超过500ms，解除冻结
      auto now = this->get_clock()->now();
      if ((now - last_auto_aim_stamp_).seconds() > 0.5) {
        angle_frozen_ = false;
        has_frozen_angle_ = false;
        RCLCPP_INFO(this->get_logger(), "[AutoAim] Target lost, unfreeze angle");
        has_auto_aim_target_ = false;
      }
    }
  }
}

// Transform the velocity from `robot_base_frame` to `fake_robot_base_frame`
void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  double angle_diff = 0.0;
  double spin_speed = 0.0;
  bool frozen = false;
  bool disable_rotation = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (angle_frozen_ && has_frozen_angle_) {
      angle_diff = frozen_angle_;
      frozen = true;
    } else {
      angle_diff = current_robot_base_angle_;
    }
    spin_speed = spin_speed_;
    disable_rotation = (has_auto_aim_target_ && auto_aim_disable_rotation_);
  }

  geometry_msgs::msg::Twist aft_tf_vel;
  // 自瞄模式下禁用旋转
  aft_tf_vel.angular.z = disable_rotation ? 0.0 : (msg->angular.z + spin_speed);
  aft_tf_vel.linear.x =
    (msg->linear.x * std::cos(angle_diff) + msg->linear.y * std::sin(angle_diff));
  aft_tf_vel.linear.y =
    (-msg->linear.x * std::sin(angle_diff) + msg->linear.y * std::cos(angle_diff));

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (enable_velocity_jump_check_ && has_last_cmd_vel_) {
      const auto now = this->get_clock()->now();
      const double dt = (now - last_cmd_vel_stamp_).seconds();

      const double current_linear_speed =
        std::hypot(aft_tf_vel.linear.x, aft_tf_vel.linear.y);
      const double prev_linear_speed = std::hypot(
        last_raw_cmd_vel_.linear.x, last_raw_cmd_vel_.linear.y);

      const double delta_linear =
        std::hypot(aft_tf_vel.linear.x - last_raw_cmd_vel_.linear.x,
                   aft_tf_vel.linear.y - last_raw_cmd_vel_.linear.y);
      const double delta_angular =
        aft_tf_vel.angular.z - last_raw_cmd_vel_.angular.z;

      bool trigger_jump =
        (delta_linear > velocity_jump_linear_threshold_) ||
        (std::fabs(delta_angular) > velocity_jump_angular_threshold_);

      if (!trigger_jump &&
        current_linear_speed > direction_valid_speed_threshold_ &&
        prev_linear_speed > direction_valid_speed_threshold_)
      {
        const double current_dir = std::atan2(aft_tf_vel.linear.y, aft_tf_vel.linear.x);
        const double prev_dir =
          std::atan2(last_raw_cmd_vel_.linear.y, last_raw_cmd_vel_.linear.x);
        double dir_delta = current_dir - prev_dir;
        while (dir_delta > M_PI) {
          dir_delta -= 2.0 * M_PI;
        }
        while (dir_delta < -M_PI) {
          dir_delta += 2.0 * M_PI;
        }
        trigger_jump = std::fabs(dir_delta) > velocity_direction_jump_threshold_;

        if (trigger_jump) {
          RCLCPP_WARN(
            this->get_logger(),
            "速度突变检测: dir_jump=%.3f rad, prev_dir=%.3f, cur_dir=%.3f, prev_speed=%.3f, cur_speed=%.3f, dt=%.3f",
            std::fabs(dir_delta), prev_dir, current_dir, prev_linear_speed, current_linear_speed, dt);
        }
      }

      if (trigger_jump) {
        RCLCPP_WARN(
          this->get_logger(),
          "速度突变检测: delta_lin=%.3f, delta_ang=%.3f, prev=(x=%.3f,y=%.3f,w=%.3f), cur=(x=%.3f,y=%.3f,w=%.3f), dt=%.3f",
          delta_linear, std::fabs(delta_angular),
          last_raw_cmd_vel_.linear.x, last_raw_cmd_vel_.linear.y, last_raw_cmd_vel_.angular.z,
          aft_tf_vel.linear.x, aft_tf_vel.linear.y, aft_tf_vel.angular.z,
          dt);
      }
    }

    last_raw_cmd_vel_ = aft_tf_vel;
    last_cmd_vel_stamp_ = this->get_clock()->now();
    has_last_cmd_vel_ = true;
  }

  aft_tf_vel = applyVelocityFilter(aft_tf_vel);

  if (frozen) {
    RCLCPP_DEBUG(this->get_logger(),
      "Frozen velocity transform: raw(x=%.3f,y=%.3f) -> tf(x=%.3f,y=%.3f), angle=%.3f",
      msg->linear.x, msg->linear.y, aft_tf_vel.linear.x, aft_tf_vel.linear.y, frozen_angle_);
  }

  cmd_vel_chassis_pub_->publish(aft_tf_vel);
  //RCLCPP_INFO(this->get_logger(),"x轴速度%lf,Y轴速度%lf", aft_tf_vel.linear.x,aft_tf_vel.linear.y);
  //温馨提醒   由于队内双头哨兵比较特殊，x轴与y轴的方向是相反的，所以将x轴与y轴的速度对调

  double speed_scale;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    speed_scale = current_speed_scale_;
  }

  roborts_msgs::msg::ChassisCmd chassis_cmd;
  chassis_cmd.lx = aft_tf_vel.linear.y * speed_scale;
  chassis_cmd.ly = aft_tf_vel.linear.x * speed_scale;

  pubChassis->publish(std::move(chassis_cmd));

}

double FakeVelTransform::clampWithStep(
  double current_value, double target_value, double max_step)
{
  if (max_step <= 0.0) {
    return target_value;
  }

  const double delta = target_value - current_value;
  if (delta > max_step) {
    return current_value + max_step;
  }
  if (delta < -max_step) {
    return current_value - max_step;
  }
  return target_value;
}

geometry_msgs::msg::Twist FakeVelTransform::applyVelocityFilter(
  const geometry_msgs::msg::Twist & twist)
{
  if (!enable_velocity_filter_) {
    return twist;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);

  const rclcpp::Time now = this->get_clock()->now();
  if (!filter_initialized_) {
    filtered_cmd_vel_ = twist;
    last_filter_stamp_ = now;
    filter_initialized_ = true;
  }

  double dt = (now - last_filter_stamp_).seconds();
  if (dt <= 1e-4 || dt > 0.5) {
    dt = 0.02;
  }
  last_filter_stamp_ = now;

  double alpha = 1.0;
  if (filter_time_constant_ > 1e-4) {
    alpha = dt / (filter_time_constant_ + dt);
    alpha = std::min(1.0, std::max(0.0, alpha));
  }

  const double lp_x =
    filtered_cmd_vel_.linear.x + alpha * (twist.linear.x - filtered_cmd_vel_.linear.x);
  const double lp_y =
    filtered_cmd_vel_.linear.y + alpha * (twist.linear.y - filtered_cmd_vel_.linear.y);
  const double lp_w =
    filtered_cmd_vel_.angular.z + alpha * (twist.angular.z - filtered_cmd_vel_.angular.z);

  const double max_linear_step = std::max(0.0, max_linear_accel_) * dt;
  const double max_angular_step = std::max(0.0, max_angular_accel_) * dt;

  filtered_cmd_vel_.linear.x =
    clampWithStep(filtered_cmd_vel_.linear.x, lp_x, max_linear_step);
  filtered_cmd_vel_.linear.y =
    clampWithStep(filtered_cmd_vel_.linear.y, lp_y, max_linear_step);
  filtered_cmd_vel_.angular.z =
    clampWithStep(filtered_cmd_vel_.angular.z, lp_w, max_angular_step);

  const double linear_speed =
    std::hypot(filtered_cmd_vel_.linear.x, filtered_cmd_vel_.linear.y);
  if (linear_speed < std::abs(linear_speed_deadband_)) {
    filtered_cmd_vel_.linear.x = 0.0;
    filtered_cmd_vel_.linear.y = 0.0;
  }
  if (std::abs(filtered_cmd_vel_.angular.z) < std::abs(angular_speed_deadband_)) {
    filtered_cmd_vel_.angular.z = 0.0;
  }

  return filtered_cmd_vel_;
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
