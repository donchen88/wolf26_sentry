#ifndef RM_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
#define RM_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_

#include "behaviortree_cpp/basic_types.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace BT
{

/**
 * @brief Parse XML string to geometry_msgs::msg::PoseStamped
 * @param key XML string
 * @return geometry_msgs::msg::PoseStamped
 */
template <>
inline geometry_msgs::msg::PoseStamped convertFromString(const StringView key)
{
  // three real numbers separated by semicolons
  auto parts = splitString(key, ';');
  if (parts.size() != 7) {
    throw RuntimeError("invalid input)");
  } else {
    geometry_msgs::msg::PoseStamped msg;
    msg.pose.position.x = convertFromString<double>(parts[0]);
    msg.pose.position.y = convertFromString<double>(parts[1]);
    msg.pose.position.z = convertFromString<double>(parts[2]);
    msg.pose.orientation.x = convertFromString<double>(parts[3]);
    msg.pose.orientation.y = convertFromString<double>(parts[4]);
    msg.pose.orientation.z = convertFromString<double>(parts[5]);
    msg.pose.orientation.w = convertFromString<double>(parts[6]);
    return msg;
  }
}

}  // namespace BT

#endif  // RM_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
