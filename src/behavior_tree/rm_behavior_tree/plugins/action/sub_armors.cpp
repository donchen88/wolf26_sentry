#include "rm_behavior_tree/plugins/action/sub_armors.hpp"
//订阅敌方装甲板识别节点的输出话题 /detector/armors，把检测到的装甲板信息传递给行为树

namespace rm_behavior_tree
{

SubArmorsAction::SubArmorsAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<auto_aim_interfaces::msg::Armors>(name, conf, params)
{
}

BT::NodeStatus SubArmorsAction::onTick(
  const std::shared_ptr<auto_aim_interfaces::msg::Armors> & last_msg)
{
  if (last_msg) {
    setOutput("armors", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubArmorsAction, "SubArmors");