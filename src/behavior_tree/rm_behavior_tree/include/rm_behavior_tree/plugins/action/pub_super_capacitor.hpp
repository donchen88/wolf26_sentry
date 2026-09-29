#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_SUPER_CAPACITOR_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_SUPER_CAPACITOR_HPP_

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "robot_msgs/msg/super_capacitor.hpp"

namespace rm_behavior_tree
{

// 发布超电状态
class PubSuperCapacitorAction : public BT::RosTopicPubNode<robot_msgs::msg::SuperCapacitor>
{
public:
  PubSuperCapacitorAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  bool setMessage(robot_msgs::msg::SuperCapacitor & msg) override;

  static BT::PortsList providedPorts();
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_SUPER_CAPACITOR_HPP_
