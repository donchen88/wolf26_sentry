#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__NOT_CONDITION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__NOT_CONDITION_HPP_

#include "behaviortree_cpp/decorator_node.h"

namespace rm_behavior_tree
{

/**
 * @brief BT Decorator节点：对子节点结果取反
 * NOT 装饰器 - 反转子节点的状态
 * SUCCESS -> FAILURE
 * FAILURE -> SUCCESS
 * RUNNING -> RUNNING
 */
class NotCondition : public BT::DecoratorNode
{
public:
    NotCondition(const std::string & name, const BT::NodeConfig & config)
    : BT::DecoratorNode(name, config)
    {}

    BT::NodeStatus tick() override
    {
        BT::NodeStatus child_status = child()->executeTick();

        switch (child_status)
        {
            case BT::NodeStatus::SUCCESS:
                return BT::NodeStatus::FAILURE;
            case BT::NodeStatus::FAILURE:
                return BT::NodeStatus::SUCCESS;
            case BT::NodeStatus::RUNNING:
                return BT::NodeStatus::RUNNING;
            default:
                return child_status;
        }
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__NOT_CONDITION_HPP_
