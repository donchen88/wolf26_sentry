#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__GET_BLACKBOARD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__GET_BLACKBOARD_HPP_

#include "behaviortree_cpp/condition_node.h"
#include <rclcpp/rclcpp.hpp>

namespace rm_behavior_tree
{

/**
 * @brief BT Condition节点：检查Blackboard中的值
 * 检查指定的blackboard键是否存在且为true
 *
 * 输入端口：
 * - key: 要检查的blackboard键名
 */
class GetBlackboardCondition : public BT::ConditionNode
{
public:
    GetBlackboardCondition(const std::string & name, const BT::NodeConfig & config)
    : BT::ConditionNode(name, config)
    {}

    BT::NodeStatus tick() override
    {
        auto key = getInput<std::string>("key");
        if (!key)
        {
            RCLCPP_ERROR(rclcpp::get_logger("get_blackboard"), "Missing required input [key]");
            return BT::NodeStatus::FAILURE;
        }

        try
        {
            auto value = config().blackboard->get<bool>(key.value());
            RCLCPP_INFO(rclcpp::get_logger("get_blackboard"),
                        "[GetBlackboard] key='%s' value=%s -> %s",
                        key.value().c_str(),
                        (value ? "true" : "false"),
                        (value ? "SUCCESS" : "FAILURE"));
            return value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
        catch (const std::exception &)
        {
            // key 不存在时返回 FAILURE，不抛异常
            RCLCPP_INFO(rclcpp::get_logger("get_blackboard"),
                        "[GetBlackboard] key='%s' not found -> FAILURE",
                        key.value().c_str());
            return BT::NodeStatus::FAILURE;
        }
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("key")
        };
    }
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__GET_BLACKBOARD_HPP_
