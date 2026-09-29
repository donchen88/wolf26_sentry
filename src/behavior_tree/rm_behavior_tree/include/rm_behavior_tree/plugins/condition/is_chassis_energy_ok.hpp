#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_CHASSIS_ENERGY_OK_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_CHASSIS_ENERGY_OK_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "robot_msgs/msg/chassis_energy.hpp"
#include <rclcpp/rclcpp.hpp>

namespace rm_behavior_tree
{

/**
 * @brief BT Condition节点：判断底盘能量是否不足
 *
 * 规则：
 * - 下位机发 0: 正常底盘能量 -> 返回 FAILURE
 * - 下位机发 1: 底盘能量不足 -> 返回 SUCCESS
 *
 * 输入端口：
 * - message: robot_msgs::msg::ChassisEnergy 类型，可为空，优先使用内部订阅消息
 */
class IsChassisEnergyOKCondition : public BT::ConditionNode
{
public:
    IsChassisEnergyOKCondition(const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<robot_msgs::msg::ChassisEnergy>("message")
        };
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<robot_msgs::msg::ChassisEnergy>::SharedPtr sub_;
    robot_msgs::msg::ChassisEnergy last_msg_;
    bool has_msg_{false};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_CHASSIS_ENERGY_OK_HPP_