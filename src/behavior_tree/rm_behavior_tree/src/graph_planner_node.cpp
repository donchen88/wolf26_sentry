#include "rm_behavior_tree/graph_builder.hpp"
#include "rm_behavior_tree/key_point_loader.hpp"
#include "rm_behavior_tree/map_accessor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace rm_behavior_tree
{

class GraphPlannerNode : public rclcpp::Node
{
public:
  GraphPlannerNode()
  : Node("graph_planner_node")
  {
    this->declare_parameter<std::string>("keypoint_yaml", "");
    this->declare_parameter<std::string>("map_topic", "/map");
    this->declare_parameter<double>("max_edge_length", 0.0);
    this->declare_parameter<int>("min_clearance_cells", 0);
    this->declare_parameter<int>("occupancy_threshold", 50);

    yaml_path_ = this->get_parameter("keypoint_yaml").as_string();
    map_topic_ = this->get_parameter("map_topic").as_string();

    options_.max_edge_length = this->get_parameter("max_edge_length").as_double();
    options_.min_clearance_cells = this->get_parameter("min_clearance_cells").as_int();
    options_.occupancy_threshold = this->get_parameter("occupancy_threshold").as_int();

    if (yaml_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "keypoint_yaml parameter required");
      return;
    }

    if (!kp_loader_.loadFromFile(yaml_path_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load keypoints from %s", yaml_path_.c_str());
      return;
    }

    // Publishers for responses
    path_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("graph_planner/response", 10);
    cost_pub_ = this->create_publisher<std_msgs::msg::Float64>("graph_planner/response_cost", 10);

    // Subscriber for requests
    req_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "graph_planner/request",
      10,
      std::bind(&GraphPlannerNode::onRequest, this, std::placeholders::_1));

    // Subscribe to map (transient_local) to build graph
    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.transient_local().reliable();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_,
      map_qos,
      std::bind(&GraphPlannerNode::mapCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "GraphPlannerNode created, waiting for map on %s", map_topic_.c_str());
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (graph_built_) return;
    RCLCPP_INFO(this->get_logger(), "Received map, building graph...");
    MapAccessor map_accessor(*msg);
    auto keypoints = kp_loader_.getKeyPoints();
    graph_ = GraphBuilder::buildGraph(keypoints, map_accessor, options_);
    graph_built_ = true;
    RCLCPP_INFO(this->get_logger(), "Graph built: nodes=%zu edges=%zu", graph_.getNodeCount(), graph_.getEdgeCount()/2);
  }

  void onRequest(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (!graph_built_) {
      RCLCPP_WARN(this->get_logger(), "Graph not built yet, cannot answer request");
      return;
    }

    if (msg->data.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "Request must contain at least two ints: start_id, goal_id");
      return;
    }

    int start_id = static_cast<int>(msg->data[0]);
    int goal_id = static_cast<int>(msg->data[1]);

    int start_idx = findIndexById(start_id);
    int goal_idx = findIndexById(goal_id);
    if (start_idx < 0 || goal_idx < 0) {
      RCLCPP_WARN(this->get_logger(), "Start or goal ID not found: %d -> %d", start_id, goal_id);
      return;
    }

    auto res = GraphBuilder::AStarSearch(graph_, start_idx, goal_idx);
    std_msgs::msg::Int32MultiArray out;
    if (!res.first.empty()) {
      // convert node indices to keypoint IDs
      for (int idx : res.first) {
        out.data.push_back(static_cast<int32_t>(graph_.nodes[idx].id));
      }
      std_msgs::msg::Float64 cost_msg;
      cost_msg.data = res.second;
      cost_pub_->publish(cost_msg);
      path_pub_->publish(out);
      RCLCPP_INFO(this->get_logger(), "Answered path request %d -> %d cost=%.3f", start_id, goal_id, res.second);
    } else {
      RCLCPP_WARN(this->get_logger(), "No path for %d -> %d", start_id, goal_id);
      std_msgs::msg::Float64 cost_msg;
      cost_msg.data = std::numeric_limits<double>::infinity();
      cost_pub_->publish(cost_msg);
      path_pub_->publish(out);
    }
  }

  int findIndexById(int id)
  {
    for (size_t i = 0; i < graph_.nodes.size(); ++i) {
      if (graph_.nodes[i].id == id) return static_cast<int>(i);
    }
    return -1;
  }

  KeyPointLoader kp_loader_;
  Graph graph_;
  GraphBuildOptions options_;
  bool graph_built_ = false;

  // params
  std::string yaml_path_;
  std::string map_topic_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr req_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cost_pub_;
};

}  // namespace rm_behavior_tree

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rm_behavior_tree::GraphPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


