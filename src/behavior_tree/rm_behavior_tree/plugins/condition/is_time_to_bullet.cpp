#include "rm_behavior_tree/plugins/condition/is_time_to_bullet.hpp"

namespace rm_behavior_tree{

IsTimeToBulletCondition::IsTimeToBulletCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsTimeToBulletCondition::checkGameStart,this),config)
{
}

BT::NodeStatus IsTimeToBulletCondition::checkGameStart(){
  int game_progress;
  std::string time_to_bullet;
  auto msg =getInput<robot_msgs::msg::GameStatus>("message");
  getInput("game_progress",game_progress);
  getInput("time_to_bullet",time_to_bullet);

  std::vector<int> time_points;
  std::istringstream iss(time_to_bullet);
  int time;
  while(iss >> time){
    time_points.push_back(time);
  }

  if(!msg){
          RCLCPP_INFO(rclcpp::get_logger("print_message"), "没有");
    return BT::NodeStatus::FAILURE;
  }

      RCLCPP_INFO(rclcpp::get_logger("print_message"), "%d %d", msg->stage_remain_time,msg->game_progress);

  if(
    msg->game_progress == game_progress && time_points.end() !=std::find(time_points.begin(),
    time_points.end(),msg->stage_remain_time)
  ){
    return BT::NodeStatus::SUCCESS;
  }else{
    return BT::NodeStatus::FAILURE;
  }
}
}//namespace rm_behavior_tree

#include"behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsTimeToBulletCondition>("IsTimeToBullet");
}