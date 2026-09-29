#include "behaviortree_cpp/action_node.h"
#include "rm_behavior_tree/plugins/action/print_message.hpp"
#include <iostream>

namespace rm_behavior_tree
{
PrintMessageAction::PrintMessageAction(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config)
{
}

BT::NodeStatus PrintMessageAction::tick() 
{
    std::string message;
    if (!getInput<std::string>("message_to_print", message))
    {
        std::cerr << "Missing required input [message_to_print]" << '\n';
        return BT::NodeStatus::FAILURE;
    }
    std::cout << message << '\n';
    RCLCPP_INFO(rclcpp::get_logger("print_message"), "%s", message.c_str());
    return BT::NodeStatus::SUCCESS;
}
}  // namespace rm_behavior_tree


#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::PrintMessageAction>("PrintMessage");
}