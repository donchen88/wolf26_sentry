#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_CHASSIS_MODE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_CHASSIS_MODE_HPP_

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "robot_msgs/msg/chassis_mode.hpp"

namespace rm_behavior_tree
{

// 发布底盘模式（复用 AutoAimMode.msg 的 mode 字段）
class PubChassisModeAction : public BT::RosTopicPubNode<robot_msgs::msg::ChassisMode>
{
public:
  PubChassisModeAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  bool setMessage(robot_msgs::msg::ChassisMode & msg) override;

  static BT::PortsList providedPorts();
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_CHASSIS_MODE_HPP_


