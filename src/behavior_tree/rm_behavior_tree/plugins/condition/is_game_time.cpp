#include "rm_behavior_tree/plugins/condition/is_game_time.hpp"

namespace rm_behavior_tree
{

IsGameTimeCondition::IsGameTimeCondition(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params)
    : BT::ConditionNode(name, config),
      node_(params.nh)
{
    // 订阅 /game_status
    sub_ = node_->create_subscription<robot_msgs::msg::GameStatus>(
        "/game_status", 10,
        [this](const robot_msgs::msg::GameStatus::SharedPtr msg)
        {
            last_msg_ = *msg;
            has_msg_ = true;
        });
}

BT::NodeStatus IsGameTimeCondition::tick()
{
    int game_progress, lower_remain_time, higher_remain_time;
    getInput("game_progress", game_progress);
    getInput("lower_remain_time", lower_remain_time);
    getInput("higher_remain_time", higher_remain_time);

    // XML message 优先
    auto msg = getInput<robot_msgs::msg::GameStatus>("message");
    if (!msg && !has_msg_)
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "IsGameTime: waiting for /game_status ...");
        return BT::NodeStatus::RUNNING;  // 启动阶段返回 RUNNING，避免误判
    }

    const auto & game_msg = msg ? *msg : last_msg_;

    if (game_msg.game_progress == game_progress &&
        game_msg.stage_remain_time >= lower_remain_time &&
        game_msg.stage_remain_time <= higher_remain_time)
    {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::IsGameTimeCondition, "IsGameTime");
