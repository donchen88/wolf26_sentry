#include "rm_behavior_tree/plugins/action/sub_decision_num.hpp"

namespace rm_behavior_tree
{

SubDecisionNumAction::SubDecisionNumAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_decision_interfaces::msg::DecisionNum>(name, conf, params)
{
}

BT::NodeStatus SubDecisionNumAction::onTick(
  const std::shared_ptr<rm_decision_interfaces::msg::DecisionNum> & last_msg)
{
  if (last_msg)  // empty if no new message received, since the last tick
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new message, decision_num: %s", name().c_str(),
      std::to_string(last_msg->decision_num).c_str());
    setOutput("decision_num", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubDecisionNumAction, "SubDecisionNum");