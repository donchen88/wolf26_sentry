#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__PRINT_MESSAGE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__PRINT_MESSAGE_HPP_

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include <iostream>

namespace rm_behavior_tree
{
class PrintMessageAction : public BT::SyncActionNode
{
public:
    PrintMessageAction(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("message_to_print")};
    }

    BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__PRINT_MESSAGE_HPP_

