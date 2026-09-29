#include "rm_behavior_tree/plugins/condition/is_target_in_zone.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <atomic>

namespace rm_behavior_tree
{
static std::atomic<int> g_is_target_in_zone_tick(0);

IsTargetInZoneAction::IsTargetInZoneAction(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsTargetInZoneAction::checkTargetInZoneStatus, this), config)
{
}

bool IsTargetInZoneAction::pointInQuadrilateral(
    double px, double py,
    double x1, double y1,
    double x2, double y2,
    double x3, double y3,
    double x4, double y4)
{
  auto cross = [](double ax, double ay, double bx, double by) {
    return ax * by - ay * bx;
  };

  double d1 = cross(x2 - x1, y2 - y1, px - x1, py - y1);
  double d2 = cross(x3 - x2, y3 - y2, px - x2, py - y2);
  double d3 = cross(x4 - x3, y4 - y3, px - x3, py - y3);
  double d4 = cross(x1 - x4, y1 - y4, px - x4, py - y4);

  bool has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0) || (d4 < 0);
  bool has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0) || (d4 > 0);

  return !(has_neg && has_pos);
}

BT::NodeStatus IsTargetInZoneAction::checkTargetInZoneStatus()
{
  auto armors = getInput<auto_aim_interfaces::msg::Armors>("armors");

  if (!armors) {
    RCLCPP_INFO(rclcpp::get_logger("is_target_in_zone"), "IsTargetInZone: No armors input, returning FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  if (armors->armors.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("is_target_in_zone"), "IsTargetInZone: Armors empty, returning FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  double x1, y1, x2, y2, x3, y3, x4, y4;
  if (!getInput("x1", x1) || !getInput("y1", y1) ||
      !getInput("x2", x2) || !getInput("y2", y2) ||
      !getInput("x3", x3) || !getInput("y3", y3) ||
      !getInput("x4", x4) || !getInput("y4", y4)) {
    RCLCPP_WARN(rclcpp::get_logger("is_target_in_zone"), "IsTargetInZone: Missing zone corners, returning FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  for (const auto & armor : armors->armors) {
    double ax = armor.pose.position.x;
    double ay = armor.pose.position.y;

    if (pointInQuadrilateral(ax, ay, x1, y1, x2, y2, x3, y3, x4, y4)) {
      RCLCPP_INFO(rclcpp::get_logger("is_target_in_zone"),
          "IsTargetInZone: Target (%.2f, %.2f) in zone [(%.2f,%.2f), (%.2f,%.2f), (%.2f,%.2f), (%.2f,%.2f)], returning SUCCESS",
          ax, ay, x1, y1, x2, y2, x3, y3, x4, y4);
      return BT::NodeStatus::SUCCESS;
    }
  }

  double ax = armors->armors[0].pose.position.x;
  double ay = armors->armors[0].pose.position.y;
  RCLCPP_INFO(rclcpp::get_logger("is_target_in_zone"),
      "IsTargetInZone: Target (%.2f, %.2f) NOT in zone [(%.2f,%.2f), (%.2f,%.2f), (%.2f,%.2f), (%.2f,%.2f)], returning FAILURE",
      ax, ay, x1, y1, x2, y2, x3, y3, x4, y4);
  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsTargetInZoneAction>("IsTargetInZone");
}
