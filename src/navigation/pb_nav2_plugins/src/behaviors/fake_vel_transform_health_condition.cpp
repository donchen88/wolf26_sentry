// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "nav2_behavior_tree/bt_condition_node.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace pb_nav2_behaviors
{

class FakeVelTransformHealthy : public nav2_behavior_tree::BtConditionNode
{
public:
  FakeVelTransformHealthy(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtConditionNode(xml_tag_name, conf),
    subscription_initialized_(false)
  {
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
    if (!node_) {
      throw std::runtime_error{"Failed to get node from blackboard"};
    }
  }

  FakeVelTransformHealthy(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : FakeVelTransformHealthy(xml_tag_name, "", conf)
  {
  }

  ~FakeVelTransformHealthy() override
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    fault_sub_.reset();
  }

  BT::NodeStatus tick() override
  {
    initializeSubscription();

    std::string topic;
    double stale_time = 1.0;
    getInput("fault_topic", topic);
    getInput("fault_stale_time", stale_time);

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_msg_) {
      return BT::NodeStatus::SUCCESS;
    }

    if (stale_time > 0.0) {
      const double elapsed = (node_->now() - last_msg_stamp_).seconds();
      if (elapsed > stale_time) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "fake_vel_transform health topic stale for %.3f s (>% .3f s), treat as fault",
          elapsed, stale_time);
        return BT::NodeStatus::FAILURE;
      }
    }

    if (last_fault_) {
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("fault_topic", "cmd_vel_fault", "cmd_vel fault topic"),
      BT::InputPort<double>(
        "fault_stale_time", 1.0,
        "Stale timeout for fault status in seconds, <=0 disables this check")
    };
  }

private:
  void initializeSubscription()
  {
    std::string topic = "cmd_vel_fault";
    getInput("fault_topic", topic);

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (subscription_initialized_ && topic == fault_topic_) {
      return;
    }

    fault_topic_ = topic;

    auto qos = rclcpp::QoS(1).best_effort();
    fault_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      fault_topic_, qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->state_mutex_);
        this->last_fault_ = msg->data;
        this->has_msg_ = true;
        this->last_msg_stamp_ = this->node_->now();
      });

    subscription_initialized_ = true;
  }

  std::mutex state_mutex_;

  std::string fault_topic_;
  bool subscription_initialized_;
  bool last_fault_;
  bool has_msg_;
  rclcpp::Time last_msg_stamp_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fault_sub_;
};

}  // namespace pb_nav2_behaviors

BT_REGISTER_NODES(factory)
{
  factory.registerBuilder<pb_nav2_behaviors::FakeVelTransformHealthy>(
    "FakeVelTransformHealthy",
    [](const std::string & name, const BT::NodeConfiguration & conf) {
      return std::make_unique<pb_nav2_behaviors::FakeVelTransformHealthy>(name, conf);
    });
}
