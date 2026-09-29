#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_DURATION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_DURATION_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{

/**
 * @brief A non-blocking duration wait node.
 * 
 * Each tick checks if the specified duration has elapsed.
 * Returns RUNNING while waiting, SUCCESS when duration is reached.
 * Can be halted by parent nodes (e.g., ReactiveFallback) for preemption.
 */
class WaitDuration : public BT::StatefulActionNode
{
public:
  /**
   * @brief A constructor for rm_behavior_tree::WaitDuration
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  WaitDuration(const std::string & name, const BT::NodeConfig & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<unsigned>("duration_msec", 1000, "Wait duration in milliseconds")};
  }

  /**
   * @brief Callback for when the node starts
   * @return BT::NodeStatus SUCCESS immediately if duration is 0, RUNNING otherwise
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief Callback for each tick while the node is running
   * @return RUNNING if still waiting, SUCCESS if duration reached
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief Callback when the node is halted
   */
  void onHalted() override;

private:
  std::chrono::time_point<std::chrono::steady_clock> start_time_;
  unsigned duration_msec_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_DURATION_HPP_
