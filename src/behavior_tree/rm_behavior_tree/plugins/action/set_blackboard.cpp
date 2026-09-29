#include "rm_behavior_tree/plugins/action/set_blackboard.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rm_behavior_tree
{
}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::SetBlackboardAction>("RMSetBlackboard");
}


