#ifndef RM_BEHAVIOR_TREE__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp/decorator_node.h"

namespace rm_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that ticks its child at a specified rate
 */
class RateController : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for rm_behavior_tree::RateController
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  RateController(const std::string & name, const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts() { return {BT::InputPort<double>("hz", 10.0, "Rate")}; }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
  double period_;
  bool first_time_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_
