#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONTROL__DECISION_SWITCH_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONTROL__DECISION_SWITCH_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "rm_decision_interfaces/msg/decision_num.hpp"
#include "behaviortree_cpp/control_node.h"

namespace rm_behavior_tree
{
class DecisionSwitch : public BT::ControlNode
{
public:
    DecisionSwitch(const std::string & name, const BT::NodeConfig & config);

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<rm_decision_interfaces::msg::DecisionNum>("decision_num")};
    }

    BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONTROL__DECISION_SWITCH_HPP_