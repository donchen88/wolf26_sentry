#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_RFID_DETECTED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_RFID_DETECTED_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "robot_msgs/msg/rfid_status.hpp"

namespace rm_behavior_tree
{

/**
 * @brief condition节点，用于判断机器人是否处于某个 RFID 增益区域中
 * @param[in] key_port RfidStatus 消息端口
 * @param[in] base_gain_point 是否检测己方基地增益点
 * @param[in] friendly_fortress_gain_point 是否检测己方堡垒增益点
 * @param[in] center_gain_point 是否检测中心增益点
 */
class IsRfidDetectedCondition : public BT::SimpleConditionNode
{
public:
  IsRfidDetectedCondition(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus checkRfidStatus();

  static BT::PortsList providedPorts();
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_RFID_DETECTED_HPP_

