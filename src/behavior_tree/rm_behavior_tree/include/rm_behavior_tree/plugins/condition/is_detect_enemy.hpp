#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_DETECT_ENEMY_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_DETECT_ENEMY_HPP_

#include <auto_aim_interfaces/msg/detail/armors__struct.hpp>
#include "behaviortree_cpp/condition_node.h"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

/**
 * @brief condition节点，用于判断视野内是否存在有效敌人
 * @param[in] message 识别模块的检测结果序列
 * @param[in] max_range 最大检测范围（米），可选参数，默认值为-1表示不限制范围
 * @param[in] debounce_ms 去抖动时间（毫秒），检测到敌人后需保持 debounce_ms 才确认，默认 200ms
 */
class IsDetectEnemyAction : public BT::SimpleConditionNode
{
public:
  IsDetectEnemyAction(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus detectEnemyStatus();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<auto_aim_interfaces::msg::Armors>("message"),
      BT::InputPort<double>("max_range", -1.0, "Maximum detection range in meters (distance from robot to enemy), -1 means no limit"),
      BT::InputPort<geometry_msgs::msg::TransformStamped>("current_location", "", "Robot current location for distance calculation (optional)"),
      BT::InputPort<int>("debounce_ms", 200, "Debounce duration in ms - enemy must be stable for this long before confirming detection")};
  }

private:
  bool isEnemyDetected() const;

  rclcpp::Time detecting_start_time_;
  rclcpp::Time lost_start_time_;
  bool was_enemy_detected_ = false;
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_FRIEND_OK_HPP_