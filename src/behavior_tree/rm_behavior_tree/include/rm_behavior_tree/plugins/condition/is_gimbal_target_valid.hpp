#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GIMBAL_TARGET_VALID_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GIMBAL_TARGET_VALID_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace rm_behavior_tree
{

/**
 * @brief 条件节点，用于判断云台手目标点是否有效且未超时
 *
 * 超时机制：
 *   - 首次收到一个非 0 目标点时启动计时（记录 first_seen_time_ 和 first_seen_x/y_）
 *   - 后续如果收到不同的点（target_x/y 与 first_seen 不同），重置计时（开始新一轮）
 *   - 如果目标点一直没变（电控一直发同一个点），超过 target_timeout_ms 后返回 FAILURE
 *   - 即使电控仍持续发送同一个点，也会被识别为超时，不会一直 SUCCESS
 *
 * @param[in] message 黑板变量 gimbal_target_valid 的值（是否有有效目标）
 * @param[in] target_x 黑板变量 gimbal_target_x 的值
 * @param[in] target_y 黑板变量 gimbal_target_y 的值
 * @param[in] target_timeout_ms 目标点超时时间(毫秒)
 *
 * 使用示例：
 * <IsGimbalTargetValid message="{gimbal_target_valid}"
 *                      target_x="{gimbal_target_x}"
 *                      target_y="{gimbal_target_y}"
 *                      target_timeout_ms="5000"/>
 */
class IsGimbalTargetValidCondition : public BT::SimpleConditionNode
{
public:
  IsGimbalTargetValidCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::SimpleConditionNode(name, std::bind(&IsGimbalTargetValidCondition::checkValid, this), config),
    node_(rclcpp::Node::make_shared("is_gimbal_target_valid_" + name)),
    has_first_target_(false)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("message", false, "Gimbal target valid flag from blackboard"),
      BT::InputPort<double>("target_x", 0.0, "Gimbal target x coordinate from blackboard"),
      BT::InputPort<double>("target_y", 0.0, "Gimbal target y coordinate from blackboard"),
      BT::InputPort<int>("target_timeout_ms", 5000, "Target timeout in milliseconds")};
  }

private:
  BT::NodeStatus checkValid()
  {
    auto msg = getInput<bool>("message");
    auto target_x = getInput<double>("target_x");
    auto target_y = getInput<double>("target_y");
    int timeout_ms = 5000;
    if (auto timeout_res = getInput<int>("target_timeout_ms")) {
      timeout_ms = timeout_res.value();
    }

    // 没有有效目标 → FAILURE
    if (!msg || !msg.value()) {
      has_first_target_ = false;
      return BT::NodeStatus::FAILURE;
    }

    if (!target_x || !target_y) {
      return BT::NodeStatus::FAILURE;
    }

    const double current_x = target_x.value();
    const double current_y = target_y.value();
    const double epsilon = 0.001;

    auto now = node_->now();

    // 首次收到目标点：记录起点
    if (!has_first_target_) {
      first_seen_time_ = now;
      first_seen_x_ = current_x;
      first_seen_y_ = current_y;
      has_first_target_ = true;
      return BT::NodeStatus::SUCCESS;
    }

    // 检查目标点是否变化
    bool target_changed =
        (std::abs(current_x - first_seen_x_) > epsilon) ||
        (std::abs(current_y - first_seen_y_) > epsilon);

    if (target_changed) {
      // 收到了新点，重置计时
      first_seen_time_ = now;
      first_seen_x_ = current_x;
      first_seen_y_ = current_y;
      RCLCPP_INFO(node_->get_logger(),
                  "[IsGimbalTargetValid] New target (%.2f, %.2f), timer reset",
                  current_x, current_y);
      return BT::NodeStatus::SUCCESS;
    }

    // 目标没变（同一点持续接收）→ 检查从第一次见到这个点到现在是否超时
    auto elapsed_ms = (now - first_seen_time_).seconds() * 1000.0;
    if (elapsed_ms > timeout_ms) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                  "[IsGimbalTargetValid] Target TIMEOUT: same point (%.2f, %.2f) for %.0f ms (threshold %d ms)",
                  current_x, current_y, elapsed_ms, timeout_ms);
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Time first_seen_time_;   // 首次见到当前目标点的时间
  double first_seen_x_;            // 当前目标点的 x
  double first_seen_y_;            // 当前目标点的 y
  bool has_first_target_;          // 是否已经记录了首次目标点
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GIMBAL_TARGET_VALID_HPP_
