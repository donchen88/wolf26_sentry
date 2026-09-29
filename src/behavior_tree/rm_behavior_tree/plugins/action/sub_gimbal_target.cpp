#include "rm_behavior_tree/plugins/action/sub_gimbal_target.hpp"
#include <sstream>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace rm_behavior_tree
{

SubGimbalTargetAction::SubGimbalTargetAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::SyncActionNode(name, config), node_(params.nh)
{
  gimbal_frame_ = "gimbal_field";
  target_frame_ = "map";
  gimbal_origin_x_ = 0.0;
  gimbal_origin_y_ = 0.0;
  gimbal_origin_z_ = 0.0;
  gimbal_yaw_ = 0.0;
  has_valid_target_ = false;
  subscription_created_ = false;
  tf_published_ = false;
  last_gimbal_x_ = 0.0;
  last_gimbal_y_ = 0.0;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
}

void SubGimbalTargetAction::publishStaticTf()
{
  geometry_msgs::msg::TransformStamped static_tf;
  static_tf.header.stamp = node_->now();
  static_tf.header.frame_id = target_frame_;
  static_tf.child_frame_id = gimbal_frame_;

  static_tf.transform.translation.x = gimbal_origin_x_;
  static_tf.transform.translation.y = gimbal_origin_y_;
  static_tf.transform.translation.z = gimbal_origin_z_;

  double roll = 0.0, pitch = 0.0;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, gimbal_yaw_);
  static_tf.transform.rotation = tf2::toMsg(q);

  static_tf_broadcaster_->sendTransform(static_tf);

  RCLCPP_INFO(node_->get_logger(),
              "SubGimbalTarget: Published static TF %s -> %s at (%.3f, %.3f, %.3f) yaw=%.3f",
              target_frame_.c_str(), gimbal_frame_.c_str(),
              gimbal_origin_x_, gimbal_origin_y_, gimbal_origin_z_, gimbal_yaw_);
}

bool SubGimbalTargetAction::transformPointToMap(
    double gimbal_x, double gimbal_y,
    geometry_msgs::msg::PoseStamped & map_pose)
{
  // 构建gimbal_frame下的PoseStamped
  geometry_msgs::msg::PoseStamped gimbal_pose;
  gimbal_pose.header.stamp = node_->now();
  gimbal_pose.header.frame_id = gimbal_frame_;
  gimbal_pose.pose.position.x = gimbal_x;
  gimbal_pose.pose.position.y = gimbal_y;
  gimbal_pose.pose.position.z = 0.0;
  gimbal_pose.pose.orientation.w = 1.0;  // 单位四元数（无旋转）

  try {
    // 使用TF2将点从gimbal_frame转换到map_frame
    map_pose = tf_buffer_->transform(gimbal_pose, target_frame_, tf2::durationFromSec(0.5));

    RCLCPP_DEBUG(node_->get_logger(),
                "SubGimbalTarget: Transformed (%.2f, %.2f) from %s to %s: (%.2f, %.2f)",
                gimbal_x, gimbal_y, gimbal_frame_.c_str(), target_frame_.c_str(),
                map_pose.pose.position.x, map_pose.pose.position.y);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(),
                "SubGimbalTarget: Failed to transform from %s to %s: %s",
                gimbal_frame_.c_str(), target_frame_.c_str(), ex.what());
    return false;
  }
}

std::string SubGimbalTargetAction::transformPointToMapString(double gimbal_x, double gimbal_y)
{
  geometry_msgs::msg::PoseStamped map_pose;

  if (!transformPointToMap(gimbal_x, gimbal_y, map_pose)) {
    RCLCPP_WARN(node_->get_logger(),
                "SubGimbalTarget: Transform failed for point (%.2f, %.2f)",
                gimbal_x, gimbal_y);
    return "";
  }

  // 格式: x;y;z;qx;qy;qz;qw
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6)
      << map_pose.pose.position.x << ";"
      << map_pose.pose.position.y << ";"
      << map_pose.pose.position.z << ";"
      << map_pose.pose.orientation.x << ";"
      << map_pose.pose.orientation.y << ";"
      << map_pose.pose.orientation.z << ";"
      << map_pose.pose.orientation.w;

  RCLCPP_INFO(node_->get_logger(),
              "SubGimbalTarget: Transformed (%.2f, %.2f) -> map: (%.2f, %.2f)",
              gimbal_x, gimbal_y,
              map_pose.pose.position.x, map_pose.pose.position.y);

  return oss.str();
}

void SubGimbalTargetAction::gimbalTargetCallback(const robot_msgs::msg::GimbalTarget::SharedPtr msg)
{
  double x = static_cast<double>(msg->x);
  double y = static_cast<double>(msg->y);

  // 过滤无效坐标点 (0,0) 表示下位机未收到有效数据
  if (std::abs(x) < 0.001 && std::abs(y) < 0.001) {
    RCLCPP_WARN(node_->get_logger(),
                "SubGimbalTarget: Received invalid target (0, 0) from gimbal, clearing");
    has_valid_target_ = false;
    return;
  }

  // 存储原始gimbal坐标
  last_gimbal_x_ = x;
  last_gimbal_y_ = y;
  has_valid_target_ = true;

  RCLCPP_INFO(node_->get_logger(),
              "SubGimbalTarget: Received gimbal target (%.2f, %.2f) in frame %s",
              x, y, gimbal_frame_.c_str());
}

BT::NodeStatus SubGimbalTargetAction::tick()
{
  // 获取输入参数
  std::string topic_name = "gimbal_target_point";
  if (auto topic_res = getInput<std::string>("topic_name")) {
    topic_name = topic_res.value();
  }

  if (auto frame_res = getInput<std::string>("gimbal_frame")) {
    gimbal_frame_ = frame_res.value();
  }

  if (auto x_res = getInput<double>("gimbal_origin_x")) {
    gimbal_origin_x_ = x_res.value();
  }
  if (auto y_res = getInput<double>("gimbal_origin_y")) {
    gimbal_origin_y_ = y_res.value();
  }
  if (auto z_res = getInput<double>("gimbal_origin_z")) {
    gimbal_origin_z_ = z_res.value();
  }
  if (auto yaw_res = getInput<double>("gimbal_yaw")) {
    gimbal_yaw_ = yaw_res.value();
  }

  // 发布静态TF（只发布一次）
  if (!tf_published_) {
    publishStaticTf();
    tf_published_ = true;
  }

  // 如果订阅者不存在，创建它
  if (!sub_) {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();

    sub_ = node_->create_subscription<robot_msgs::msg::GimbalTarget>(
      topic_name, qos,
      std::bind(&SubGimbalTargetAction::gimbalTargetCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "SubGimbalTarget: Subscribed to topic [%s]", topic_name.c_str());
    subscription_created_ = true;
  }

  // 有有效目标就转换输出，SubGimbalTarget 只负责坐标转换
  // 超时控制由 IsGimbalTargetValid 节点负责
  std::string target_pose_str = "";
  if (has_valid_target_) {
    target_pose_str = transformPointToMapString(last_gimbal_x_, last_gimbal_y_);
    // 如果转换失败，标记为无效
    if (target_pose_str.empty()) {
      has_valid_target_ = false;
    }
  }

  // 输出结果
  setOutput("target_pose", target_pose_str);
  setOutput("target_valid", has_valid_target_);
  setOutput("target_x", has_valid_target_ ? last_gimbal_x_ : 0.0);
  setOutput("target_y", has_valid_target_ ? last_gimbal_y_ : 0.0);

  RCLCPP_INFO(node_->get_logger(), "SubGimbalTarget: OUTPUT target_valid=%d, target_pose='%s', target_x=%.2f, target_y=%.2f",
              has_valid_target_, target_pose_str.c_str(), last_gimbal_x_, last_gimbal_y_);

  // 只有订阅者创建成功就返回 SUCCESS（只要订阅成功就开始工作）
  if (subscription_created_) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubGimbalTargetAction, "SubGimbalTarget");
