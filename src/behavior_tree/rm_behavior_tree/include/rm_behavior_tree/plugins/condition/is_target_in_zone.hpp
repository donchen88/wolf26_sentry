#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_TARGET_IN_ZONE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_TARGET_IN_ZONE_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "auto_aim_interfaces/msg/armors.hpp"

namespace rm_behavior_tree
{

/**
 * @brief 条件节点，用于判断目标是否在指定的四边形区域内
 * @param[in] armors 目标装甲板位置数据（SubTargetPos输出的armors，已经是map坐标系）
 * @param[in] x1, y1 角点1坐标 (左下角)
 * @param[in] x2, y2 角点2坐标 (右下角)
 * @param[in] x3, y3 角点3坐标 (右上角)
 * @param[in] x4, y4 角点4坐标 (左上角)
 * @note 角点顺序：顺时针或逆时针均可，支持任意凸四边形。
 *       默认矩形顺序：左下 -> 右下 -> 右上 -> 左上
 * @note 如果目标坐标在四边形区域内返回SUCCESS，否则返回FAILURE
 */
class IsTargetInZoneAction : public BT::SimpleConditionNode
{
public:
  IsTargetInZoneAction(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus checkTargetInZoneStatus();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<auto_aim_interfaces::msg::Armors>("armors", "Target armors from SubTargetPos (map frame)"),
      BT::InputPort<double>("x1", 0.0, "Corner 1 x coordinate"),
      BT::InputPort<double>("y1", 0.0, "Corner 1 y coordinate"),
      BT::InputPort<double>("x2", 0.0, "Corner 2 x coordinate"),
      BT::InputPort<double>("y2", 0.0, "Corner 2 y coordinate"),
      BT::InputPort<double>("x3", 0.0, "Corner 3 x coordinate"),
      BT::InputPort<double>("y3", 0.0, "Corner 3 y coordinate"),
      BT::InputPort<double>("x4", 0.0, "Corner 4 x coordinate"),
      BT::InputPort<double>("y4", 0.0, "Corner 4 y coordinate")};
  }

private:
  bool pointInQuadrilateral(double px, double py,
                            double x1, double y1,
                            double x2, double y2,
                            double x3, double y3,
                            double x4, double y4);
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_TARGET_IN_ZONE_HPP_
