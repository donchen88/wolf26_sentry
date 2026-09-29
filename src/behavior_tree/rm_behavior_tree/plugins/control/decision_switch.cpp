#include "rm_behavior_tree/plugins/control/decision_switch.hpp"
#include "behaviortree_cpp/behavior_tree.h"

namespace rm_behavior_tree
{
    DecisionSwitch::DecisionSwitch(const std::string & name, const BT::NodeConfig & config)
    : BT::ControlNode(name, config)
    {
    }

    BT::NodeStatus DecisionSwitch::tick()
    {
        auto msg = getInput<rm_decision_interfaces::msg::DecisionNum>("decision_num");

        if(!msg)
        {
            std::cout << "missing required input [decision_num]" << '\n';
            return BT::NodeStatus::FAILURE;
        }

        switch (msg->decision_num)
        {
            case 1:
                if(children_nodes_[0]->executeTick() == BT::NodeStatus::SUCCESS)
                {
                    return BT::NodeStatus::SUCCESS;
                }
                break;
            case 2:
                if(children_nodes_[1]->executeTick() == BT::NodeStatus::SUCCESS)
                {
                    return BT::NodeStatus::SUCCESS;
                }
                break;
            // 可以添加更多分支
            default:
                return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::FAILURE;
    }
}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::DecisionSwitch>("DecisionSwitch");
}