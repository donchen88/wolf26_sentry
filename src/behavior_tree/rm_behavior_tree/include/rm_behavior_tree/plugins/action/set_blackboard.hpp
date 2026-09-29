#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_BLACKBOARD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_BLACKBOARD_HPP_

#include "behaviortree_cpp/action_node.h"
#include <rclcpp/rclcpp.hpp>

namespace rm_behavior_tree
{

/**
 * @brief BT Action节点：设置Blackboard中的值
 * 将指定的值写入blackboard
 *
 * 输入端口：
 * - key: 要设置的blackboard键名
 * - value: 要设置的值 (bool类型)
 */
class SetBlackboardAction : public BT::SyncActionNode
{
public:
    SetBlackboardAction(const std::string & name, const BT::NodeConfig & config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus tick() override
    {
        auto key = getInput<std::string>("key");
        if (!key)
        {
            RCLCPP_ERROR(rclcpp::get_logger("set_blackboard"), "Missing required input [key]");
            return BT::NodeStatus::FAILURE;
        }

        auto value = getInput<bool>("value");
        if (!value)
        {
            RCLCPP_ERROR(rclcpp::get_logger("set_blackboard"), "Missing required input [value]");
            return BT::NodeStatus::FAILURE;
        }

        config().blackboard->set<bool>(key.value(), value.value());
        RCLCPP_INFO(rclcpp::get_logger("set_blackboard"),
                    "[RMSetBlackboard] key='%s' set to %s -> SUCCESS",
                    key.value().c_str(),
                    (value.value() ? "true" : "false"));
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("key"),
            BT::InputPort<bool>("value")
        };
    }
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_BLACKBOARD_HPP_
