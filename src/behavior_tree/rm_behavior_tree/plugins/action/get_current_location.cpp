#include "rm_behavior_tree/plugins/action/get_current_location.hpp"

#include <rclcpp/logging.hpp>

namespace rm_behavior_tree
{
static std::atomic<int> g_getloc_tick(0);

GetCurrentLocationAction::GetCurrentLocationAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::SyncActionNode(name, config), node_(params.nh), logger_(params.nh->get_logger())
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  if (!tf_listener_) {
    throw std::runtime_error("Failed to create tf2_ros::TransformListener");
  }
}

BT::NodeStatus GetCurrentLocationAction::tick()
{
  int my_tick = ++g_getloc_tick;

  std::string target_frame = "map";
  std::string source_frame = "gimbal_yaw";

  if (auto target_res = getInput<std::string>("target_frame")) {
    target_frame = target_res.value();
  }
  if (auto source_res = getInput<std::string>("source_frame")) {
    source_frame = source_res.value();
  }

  try {
    // 用 rclcpp::Time(0) 查最新可用 TF，有一定容差
    geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
      target_frame,
      source_frame,
      rclcpp::Time(0),
      std::chrono::milliseconds(100));

    setOutput("current_location", t);
    return BT::NodeStatus::SUCCESS;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      logger_,
      *node_->get_clock(),
      3000,
      "GetCurrentLocation tick#%d: FAILURE - TF %s->%s failed: %s",
      my_tick,
      source_frame.c_str(),
      target_frame.c_str(),
      e.what());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::GetCurrentLocationAction, "GetCurrentLocation");