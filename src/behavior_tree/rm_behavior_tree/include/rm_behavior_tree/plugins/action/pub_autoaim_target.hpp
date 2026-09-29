#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_AUTOAIM_TARGET_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_AUTOAIM_TARGET_HPP_

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "sp_msgs/msg/autoaim_target_msg.hpp"

namespace rm_behavior_tree
{

class PubAutoaimTargetAction : public BT::RosTopicPubNode<sp_msgs::msg::AutoaimTargetMsg>
{
public:
  PubAutoaimTargetAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  bool setMessage(sp_msgs::msg::AutoaimTargetMsg & msg) override;

  static BT::PortsList providedPorts();
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_AUTOAIM_TARGET_HPP_

