#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_ENEMY_STATUS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_ENEMY_STATUS_HPP_

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "sp_msgs/msg/enemy_status_msg.hpp"

namespace rm_behavior_tree
{

class PubEnemyStatusAction : public BT::RosTopicPubNode<sp_msgs::msg::EnemyStatusMsg>
{
public:
  PubEnemyStatusAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  bool setMessage(sp_msgs::msg::EnemyStatusMsg & msg) override;

  static BT::PortsList providedPorts();
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_ENEMY_STATUS_HPP_

