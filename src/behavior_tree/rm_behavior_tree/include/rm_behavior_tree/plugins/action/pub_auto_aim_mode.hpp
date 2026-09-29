#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_AUTO_AIM_MODE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_AUTO_AIM_MODE_HPP_

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "robot_msgs/msg/auto_aim_mode.hpp"

namespace rm_behavior_tree
{

class PubAutoAimModeAction : public BT::RosTopicPubNode<robot_msgs::msg::AutoAimMode>
{
public:
  PubAutoAimModeAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  bool setMessage(robot_msgs::msg::AutoAimMode & msg) override;

  static BT::PortsList providedPorts();
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_AUTO_AIM_MODE_HPP_

