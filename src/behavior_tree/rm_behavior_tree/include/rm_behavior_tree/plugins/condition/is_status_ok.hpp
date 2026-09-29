#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <robot_msgs/msg/robot_status.hpp>

namespace rm_behavior_tree
{

class IsStatusOKAction : public BT::ConditionNode
{
public:
  IsStatusOKAction(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      // 来自 SubRobotStatus 的黑板数据
      BT::InputPort<robot_msgs::msg::RobotStatus>("message"),

      // 阈值参数
      BT::InputPort<int>("hp_threshold"),
      BT::InputPort<int>("heat_threshold"),  // 剩余弹量阈值
      BT::InputPort<int>("barrel_heat_limit_threshold")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace rm_behavior_tree
