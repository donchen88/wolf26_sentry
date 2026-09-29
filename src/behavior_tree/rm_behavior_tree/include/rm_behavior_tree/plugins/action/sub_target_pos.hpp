#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_TARGET_POS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_TARGET_POS_HPP_

#include "auto_aim_interfaces/msg/armors.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace rm_behavior_tree
{

/**
 * @brief 订阅目标位置话题，解析字符串格式的位置信息并转换为Armors消息
 * @param[in] topic_name 话题名称，默认为"auto_aim_target_pos"
 * @param[in] target_frame 目标坐标系名称（armors.header.frame_id），默认为"map"
 * @param[in] source_frame 视觉数据发布时的源坐标系，默认为"camera_init"
 * @param[out] armors 输出的Armors消息
 */
class SubTargetPosAction : public BT::SyncActionNode
{
public:
  SubTargetPosAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name", "auto_aim_target_pos", "Topic name for target position"),
      BT::InputPort<std::string>("target_frame", "map", "Target frame to transform armors to (e.g., map)"),
      BT::InputPort<std::string>("source_frame", "camera_init", "Source frame of visual data (camera/body frame)"),
      BT::OutputPort<auto_aim_interfaces::msg::Armors>("armors", "Converted armors message")};
  }

  BT::NodeStatus tick() override;

private:
  bool parseTargetPos(const std::string & data, double & x, double & y, double & z, double & w);
  void targetPosCallback(const std_msgs::msg::String::SharedPtr msg);
  bool transformArmorToTargetFrame(const geometry_msgs::msg::Pose & in_pose,
                                   geometry_msgs::msg::Pose & out_pose,
                                   const rclcpp::Time & stamp);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  auto_aim_interfaces::msg::Armors last_armors_;
  auto_aim_interfaces::msg::Armor raw_armor_;  // 原始 source_frame 坐标系下的数据
  auto_aim_interfaces::msg::Armor last_valid_armor_;  // TF失败时保留上一帧有效数据
  std::string target_frame_;
  std::string source_frame_;
  bool subscription_created_;
  rclcpp::Time last_msg_time_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_TARGET_POS_HPP_
