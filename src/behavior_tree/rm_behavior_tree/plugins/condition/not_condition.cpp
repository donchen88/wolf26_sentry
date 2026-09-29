#include "rm_behavior_tree/plugins/condition/not_condition.hpp"

namespace rm_behavior_tree
{
}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::NotCondition>("Not");
}


