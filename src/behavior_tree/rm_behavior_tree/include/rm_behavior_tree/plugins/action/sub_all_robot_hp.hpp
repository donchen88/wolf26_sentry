#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ALL_ROBOT_HP_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ALL_ROBOT_HP_HPP_

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_decision_interfaces/msg/all_robot_hp.hpp"
#include "robot_msgs/msg/all_robot_hp.hpp"

namespace rm_behavior_tree
{
class SubAllRobotHPAction : public BT::RosTopicSubNode<robot_msgs::msg::AllRobotHP>
{
public:
  SubAllRobotHPAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::OutputPort<robot_msgs::msg::AllRobotHP>("robot_hp")};
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<robot_msgs::msg::AllRobotHP> & last_msg) override;
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ALL_ROBOT_HP_HPP_