#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_TIME_TO_BULLET_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_TIME_TO_BULLET_HPP_

#include"behaviortree_cpp/condition_node.h"
#include"robot_msgs/msg/game_status.hpp"
#include <vector>
#include <algorithm>  // std::find
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{
    /**
 * @brief condition节点，用于判断比赛阶段与剩余时间是否符合预期
  * {0, "未开始比赛"}, {1, "准备阶段"}, {2, "十五秒裁判系统自检阶段"},
  * {3, "五秒倒计时"}, {4, "比赛开始"}, {5, "比赛结算中"}
 * @param[in] message 比赛状态话题id
 * @param[in] game_progress 期望的比赛阶段
 * @param[in] time_to_bullet 期望的比赛时间点
 */
class IsTimeToBulletCondition : public BT::SimpleConditionNode
{
public:
    IsTimeToBulletCondition(const std::string & name, const BT::NodeConfig & config);

    BT::NodeStatus checkGameStart();

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<robot_msgs::msg::GameStatus>("message"),
            BT::InputPort<int>("game_progress"),BT::InputPort<std::string>("time_to_bullet")
        };
    }
};
} //namespace rm_behavior_tree

#endif  //RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_TIME_TO_BULLET_HPP_