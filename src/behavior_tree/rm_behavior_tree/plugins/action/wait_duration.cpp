#include "rm_behavior_tree/plugins/action/wait_duration.hpp"

#include <chrono>
#include <string>

namespace rm_behavior_tree
{

WaitDuration::WaitDuration(const std::string & name, const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf)
{
}

BT::NodeStatus WaitDuration::onStart()
{
  // Get duration from input port (default 1000ms)
  duration_msec_ = 1000;
  getInput("duration_msec", duration_msec_);

  if (duration_msec_ == 0) {
    // No waiting needed, return SUCCESS immediately
    return BT::NodeStatus::SUCCESS;
  }

  // Record start time
  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitDuration::onRunning()
{
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_);

  if (elapsed.count() >= static_cast<long>(duration_msec_)) {
    // Duration reached, return SUCCESS
    return BT::NodeStatus::SUCCESS;
  }

  // Still waiting, return RUNNING
  return BT::NodeStatus::RUNNING;
}

void WaitDuration::onHalted()
{
  // Reset state - node will return FAILURE when halted
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::WaitDuration>("WaitDuration");
}
