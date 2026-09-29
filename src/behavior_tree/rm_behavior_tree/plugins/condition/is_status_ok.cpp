#include "rm_behavior_tree/plugins/condition/is_status_ok.hpp"
#include <iostream>

namespace rm_behavior_tree
{

IsStatusOKAction::IsStatusOKAction(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::ConditionNode(name, config)
{
  (void)params;
}

BT::NodeStatus IsStatusOKAction::tick()
{
  robot_msgs::msg::RobotStatus msg;

  // 1️⃣ 必须从黑板拿到 robot_status
  if (!getInput("message", msg))
  {
    // 启动阶段：SubRobotStatus 还没写入
    // 返回 SUCCESS，让 Inverter 翻转为 FAILURE，第一分支立即失败
    // ReactiveFallback 切换到第二分支（主要逻辑）
    return BT::NodeStatus::SUCCESS;
  }

  // 调试打印收到的原始字段，帮助定位反序列化/字节序问题
  std::cout << "[IsStatusOK] robot_id=" << static_cast<uint32_t>(msg.robot_id)
            << " current_hp=" << msg.current_hp
            << " shooter_barrel_heat_limit=" << msg.shooter_barrel_heat_limit
            << " shooter_heat=" << msg.shooter_heat
            << " team_color=" << msg.team_color
            << " is_attacked=" << msg.is_attacked
            << std::endl;

  int hp_threshold = 0;
  int heat_threshold = 0;
  int barrel_heat_limit_threshold = 0;

  getInput("hp_threshold", hp_threshold);
  getInput("heat_threshold", heat_threshold);
  getInput("barrel_heat_limit_threshold", barrel_heat_limit_threshold);

  // 2️⃣ 判断逻辑（与你当前工程语义一致）
  bool hp_ok = msg.current_hp >= hp_threshold;

  // shooter_heat = 剩余弹量（越大越好）
  bool ammo_ok = msg.shooter_heat >= heat_threshold;

  // 枪口热量上限不能超
  bool barrel_ok =
    msg.shooter_barrel_heat_limit <= barrel_heat_limit_threshold;

  if (hp_ok && ammo_ok && barrel_ok)
  {
    std::cout << "[IsStatusOK] returning SUCCESS" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    std::cout << "[IsStatusOK] returning FAILURE" << std::endl;
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::IsStatusOKAction, "IsStatusOK");
