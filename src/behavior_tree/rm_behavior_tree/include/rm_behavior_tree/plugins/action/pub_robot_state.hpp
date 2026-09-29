#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_ROBOT_STATE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_ROBOT_STATE_HPP_

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "robot_msgs/msg/robot_state.hpp"

namespace rm_behavior_tree
{

class PubRobotStateAction : public BT::RosTopicPubNode<robot_msgs::msg::RobotState>
{
public:
  PubRobotStateAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  bool setMessage(robot_msgs::msg::RobotState & msg) override;

  static BT::PortsList providedPorts();
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_ROBOT_STATE_HPP_

