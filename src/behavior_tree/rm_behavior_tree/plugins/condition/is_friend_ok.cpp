#include "rm_behavior_tree/plugins/condition/is_friend_ok.hpp"

namespace rm_behavior_tree
{

IsFriendOKAction::IsFriendOKAction(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsFriendOKAction::checkFriendStatus, this), config)
{
}

BT::NodeStatus IsFriendOKAction::checkFriendStatus()
{
  int friend_average_hp = 0;
  int enemy_average_hp = 0;
  auto msg = getInput<rm_decision_interfaces::msg::AllRobotHP>("message");
  auto friend_color = getInput<std::string>("friend_color");

  if (!msg) {
    std::cerr << "Missing required input [message]" << '\n';
    return BT::NodeStatus::FAILURE;
  }

  if (friend_color != "red" && friend_color != "blue") {
    std::cerr << "Unknown color: " << friend_color->c_str() << '\n';
    return BT::NodeStatus::FAILURE;
  }

  // clang-format off
  // 由于3V3中无法较早确定对手上场机器人编号，故此处均设为 1（英雄）, 3（步兵）, 4（步兵）, 7（哨兵）
  if (friend_color == "red") {
    friend_average_hp = (msg->red_1_robot_hp + msg->red_3_robot_hp + msg->red_4_robot_hp + msg->red_7_robot_hp) / 4;
    enemy_average_hp = (msg->blue_1_robot_hp + msg->blue_3_robot_hp + msg->blue_4_robot_hp + msg->blue_7_robot_hp) / 4;
  } else {
    friend_average_hp = (msg->blue_1_robot_hp + msg->blue_3_robot_hp + msg->blue_4_robot_hp + msg->blue_7_robot_hp) / 4;
    enemy_average_hp = (msg->red_1_robot_hp + msg->red_3_robot_hp + msg->red_4_robot_hp + msg->red_7_robot_hp) / 4;
  }
  // clang-format on

  if (friend_average_hp > enemy_average_hp) {
    // std::cout << "我方血量优势" << '\n';
    return BT::NodeStatus::SUCCESS;
  } else {
    // std::cout << "我方血量劣势" << '\n';
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsFriendOKAction>("IsFriendOK");
}
