#include "rm_behavior_tree/plugins/action/sub_target_pos.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <sstream>
#include <algorithm>
#include <cmath>

namespace rm_behavior_tree
{

SubTargetPosAction::SubTargetPosAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::SyncActionNode(name, config), node_(params.nh)
{
  target_frame_ = "map";
  source_frame_ = "chassis";

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  last_armors_.armors.clear();
  last_armors_.header.frame_id = target_frame_;
  last_armors_.header.stamp = node_->now();
  last_msg_time_ = node_->now();

  subscription_created_ = false;
}

bool SubTargetPosAction::parseTargetPos(
  const std::string & data, double & x, double & y, double & z, double & w)
{
  std::istringstream iss(data);
  std::string token;
  std::vector<double> values;

  while (std::getline(iss, token, ',')) {
    try {
      values.push_back(std::stod(token));
    } catch (const std::exception &) {
      return false;
    }
  }

  if (values.size() < 4) {
    return false;
  }

  x = values[0];
  y = values[1];
  z = values[2];
  w = values[3];
  return true;
}

void SubTargetPosAction::targetPosCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_DEBUG(node_->get_logger(), "SubTargetPos CB: Received raw msg: '%s'", msg->data.c_str());

  double x, y, z, w;

  if (!parseTargetPos(msg->data, x, y, z, w)) {
    RCLCPP_WARN(node_->get_logger(), "SubTargetPos CB: Failed to parse target pos");
    return;
  }

  if (std::abs(x) < 1e-6 && std::abs(y) < 1e-6) {
    // 显式清空 raw_armor_ 表示敌人消失
    raw_armor_.pose.position.x = 0.0;
    raw_armor_.pose.position.y = 0.0;
    raw_armor_.pose.position.z = 0.0;
    last_armors_.armors.clear();
    last_armors_.header.frame_id = target_frame_;
    last_armors_.header.stamp = node_->now();
    last_msg_time_ = node_->now();
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "SubTargetPos CB: Detected zero armor, clearing data");
    return;
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "SubTargetPos CB: raw=(%.3f, %.3f, %.3f)", x, y, z);

  raw_armor_.pose.position.x = x;
  raw_armor_.pose.position.y = y;
  raw_armor_.pose.position.z = z;
  raw_armor_.pose.orientation.x = 0.0;
  raw_armor_.pose.orientation.y = 0.0;
  raw_armor_.pose.orientation.z = 0.0;
  raw_armor_.pose.orientation.w = 1.0;

  last_msg_time_ = node_->now();
}

bool SubTargetPosAction::transformArmorToTargetFrame(
  const geometry_msgs::msg::Pose & in_pose,
  geometry_msgs::msg::Pose & out_pose,
  const rclcpp::Time & stamp)
{
  if (!tf_buffer_) {
    out_pose = geometry_msgs::msg::Pose();
    return false;
  }

  const std::string & actual_src = source_frame_;
  const std::string & actual_tgt = target_frame_;

  if (actual_src == actual_tgt) {
    out_pose = in_pose;
    return true;
  }

  try {
    // 用当前时间查 TF（消息时间戳可能是旧时间，导致 extrapolation 失败）
    geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
      actual_tgt, actual_src, rclcpp::Time(0), tf2::durationFromSec(0.5));

    geometry_msgs::msg::PoseStamped pose_in, pose_out;
    pose_in.header.stamp = rclcpp::Time(0);  // 用当前时间保持一致
    pose_in.header.frame_id = actual_src;
    pose_in.pose = in_pose;

    tf2::doTransform(pose_in, pose_out, tf_stamped);
    out_pose = pose_out.pose;

    return true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(node_->get_logger(),
                "SubTargetPos: [TF FAIL] %s->%s failed: %s  → returning empty",
                actual_src.c_str(), actual_tgt.c_str(), e.what());
    out_pose = geometry_msgs::msg::Pose();
    return false;
  }
}

BT::NodeStatus SubTargetPosAction::tick()
{
  static int tick_count = 0;
  if (tick_count++ % 100 == 0) {
    RCLCPP_INFO(node_->get_logger(), "SubTargetPos tick #%d: raw=(%.3f,%.3f,%.3f) sub_ready=%d",
        tick_count,
        raw_armor_.pose.position.x, raw_armor_.pose.position.y, raw_armor_.pose.position.z,
        sub_ != nullptr);
  }

  std::string topic_name = "auto_aim_target_pos";
  if (auto topic_res = getInput<std::string>("topic_name")) {
    topic_name = topic_res.value();
  }

  if (auto target_res = getInput<std::string>("target_frame")) {
    target_frame_ = target_res.value();
  }

  if (auto source_res = getInput<std::string>("source_frame")) {
    source_frame_ = source_res.value();
  }

  if (!sub_) {
    rclcpp::QoS qos(10);
    qos.reliable();

    RCLCPP_INFO(node_->get_logger(), "SubTargetPos: Creating subscription to '%s'", topic_name.c_str());
    sub_ = node_->create_subscription<std_msgs::msg::String>(
      topic_name, qos,
      std::bind(&SubTargetPosAction::targetPosCallback, this, std::placeholders::_1));

    subscription_created_ = true;
    RCLCPP_INFO(node_->get_logger(), "SubTargetPos: Subscription created successfully");
  }

  if (subscription_created_ && !last_armors_.armors.empty()) {
    auto dt = (node_->now() - last_msg_time_).seconds();
    if (dt > 10.0) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
          "SubTargetPos: No message for %.1f seconds, clearing armor data", dt);
      last_armors_.armors.clear();
      last_valid_armor_ = auto_aim_interfaces::msg::Armor();
    }
  }

  last_armors_.armors.clear();
  last_armors_.header.frame_id = target_frame_;
  last_armors_.header.stamp = node_->now();

  auto t0 = std::chrono::steady_clock::now();
  auto ms_now = [this, &t0]() {
    return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
  };

  if (!raw_armor_.pose.position.x && !raw_armor_.pose.position.y && !raw_armor_.pose.position.z) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "SubTargetPos: raw_armor is (0,0,0), enemy lost");
    // 敌人消失时清空有效数据，输出空数组让行为树能检测到敌人丢失
    last_valid_armor_ = auto_aim_interfaces::msg::Armor();
    last_armors_.armors.clear();
    setOutput("armors", last_armors_);
    return subscription_created_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::Pose transformed_pose;
  bool transform_ok = transformArmorToTargetFrame(
    raw_armor_.pose, transformed_pose, last_msg_time_);

  if (!transform_ok) {
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "SubTargetPos: [%05.1fms] TF FAIL %s->%s raw=(%.3f,%.3f,%.3f), keeping last valid",
        ms_now(), source_frame_.c_str(), target_frame_.c_str(),
        raw_armor_.pose.position.x, raw_armor_.pose.position.y, raw_armor_.pose.position.z);
    // TF失败时保留上一帧有效数据（不输出0）
    if (last_valid_armor_.pose.position.x || last_valid_armor_.pose.position.y || last_valid_armor_.pose.position.z) {
      transformed_pose.position = last_valid_armor_.pose.position;
      transformed_pose.orientation = last_valid_armor_.pose.orientation;
      transform_ok = true;
    }
  } else {
    // 成功时更新last_valid_armor_
    last_valid_armor_.pose.position = transformed_pose.position;
    last_valid_armor_.pose.orientation = transformed_pose.orientation;
    last_valid_armor_.number = "0";
    last_valid_armor_.type = "unknown";
    last_valid_armor_.distance_to_image_center = 0.0f;
  }

  last_armors_.header.frame_id = target_frame_;

  auto_aim_interfaces::msg::Armor armor;
  armor.pose.position = transformed_pose.position;
  armor.pose.orientation = transformed_pose.orientation;
  armor.number = "0";
  armor.type = "unknown";
  armor.distance_to_image_center = 0.0f;

  last_armors_.armors.push_back(armor);

  setOutput("armors", last_armors_);
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "SubTargetPos: [%05.1fms] done -> (%.3f,%.3f,%.3f)",
      ms_now(),
      transformed_pose.position.x, transformed_pose.position.y, transformed_pose.position.z);
  return subscription_created_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubTargetPosAction, "SubTargetPos");
