#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__GET_CURRENT_LOCATION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__GET_CURRENT_LOCATION_HPP_

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <rclcpp/rclcpp.hpp>

namespace rm_behavior_tree
{

class GetCurrentLocationAction : public BT::SyncActionNode
{
public:
  GetCurrentLocationAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::TransformStamped>("current_location"),
      BT::InputPort<std::string>("target_frame", "map", "Target frame (usually 'map')"),
      BT::InputPort<std::string>("source_frame", "gimbal_yaw", "Source frame (usually 'gimbal_yaw')")};
  }

private:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Logger logger_{rclcpp::get_logger("rm_behavor_tree")};
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__GET_CURRENT_LOCATION_HPP_