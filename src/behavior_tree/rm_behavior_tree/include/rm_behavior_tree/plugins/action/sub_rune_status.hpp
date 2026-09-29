#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_RUNE_STATUS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_RUNE_STATUS_HPP_

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "robot_msgs/msg/rune_status.hpp"

namespace rm_behavior_tree
{
/**
 * @brief BT Action节点：订阅 /rune_status，把电控下行的 RuneStatus 消息写入 blackboard。
 *
 * 输入端口：
 *  - topic_name: 要订阅的话题名（默认 /rune_status）
 *
 * 输出端口：
 *  - rune_status_msg: robot_msgs::msg::RuneStatus 整条消息
 */
class SubRuneStatusAction : public BT::RosTopicSubNode<robot_msgs::msg::RuneStatus>
{
public:
  SubRuneStatusAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::OutputPort<robot_msgs::msg::RuneStatus>("rune_status")};
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<robot_msgs::msg::RuneStatus> & last_msg) override;
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_RUNE_STATUS_HPP_