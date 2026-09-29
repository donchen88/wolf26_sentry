#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GAME_TIME_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GAME_TIME_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "robot_msgs/msg/game_status.hpp"
#include <rclcpp/rclcpp.hpp>

namespace rm_behavior_tree
{

/**
 * @brief BT Condition节点：判断比赛阶段和剩余时间是否符合预期
 *
 * 输入端口：
 * - message: robot_msgs::msg::GameStatus 类型，可为空，优先使用内部订阅消息
 * - game_progress: 期望比赛阶段
 * - lower_remain_time: 剩余时间下限
 * - higher_remain_time: 剩余时间上限
 */
class IsGameTimeCondition : public BT::ConditionNode
{
public:
    // 构造函数，传入 ROS2 节点参数
    IsGameTimeCondition(const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

    BT::NodeStatus tick() override;

    // Groot2 需要这个
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<robot_msgs::msg::GameStatus>("message"),
            BT::InputPort<int>("game_progress"),
            BT::InputPort<int>("lower_remain_time"),
            BT::InputPort<int>("higher_remain_time")
        };
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<robot_msgs::msg::GameStatus>::SharedPtr sub_;
    robot_msgs::msg::GameStatus last_msg_;
    bool has_msg_{false};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GAME_TIME_HPP_
