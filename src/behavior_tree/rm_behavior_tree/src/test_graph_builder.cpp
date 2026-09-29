#include "rm_behavior_tree/key_point_loader.hpp"
#include "rm_behavior_tree/map_accessor.hpp"
#include "rm_behavior_tree/graph_builder.hpp"
#include "rm_behavior_tree/graph_visualizer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <memory>
#include <cstdint>
#include <iostream>

using rm_behavior_tree::KeyPointLoader;
using rm_behavior_tree::MapAccessor;
using rm_behavior_tree::GraphBuilder;
using rm_behavior_tree::Graph;
using rm_behavior_tree::GraphVisualizer;
using rm_behavior_tree::GraphBuildOptions;

class GraphBuilderTestNode : public rclcpp::Node
{
public:
  GraphBuilderTestNode()
  : Node("graph_builder_test")
  {
    // 声明参数
    this->declare_parameter<std::string>("keypoint_yaml", "");
    this->declare_parameter<std::string>("map_topic", "/map");
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<double>("max_edge_length", 0.0);          // 米，<=0 不限制
    this->declare_parameter<int>("min_clearance_cells", 1);           // 栅格数
    this->declare_parameter<int>("occupancy_threshold", 50);          // 0-100
    this->declare_parameter<std::vector<int64_t>>("blacklist_edges", {}); // 形如 [id1,id2,id3,id4,...]

    // 获取参数
    std::string yaml_path = this->get_parameter("keypoint_yaml").as_string();
    std::string map_topic = this->get_parameter("map_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();

    if (yaml_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Please provide keypoint_yaml parameter!");
      return;
    }

    // 加载关键点
    RCLCPP_INFO(this->get_logger(), "Loading keypoints from: %s", yaml_path.c_str());
    if (!keypoint_loader_.loadFromFile(yaml_path)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load keypoints!");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu keypoints", keypoint_loader_.size());

    // 读取构图参数
    options_.max_edge_length = this->get_parameter("max_edge_length").as_double();
    options_.min_clearance_cells = this->get_parameter("min_clearance_cells").as_int();
    options_.occupancy_threshold = this->get_parameter("occupancy_threshold").as_int();

    // 解析黑名单（偶数个整数，按 [id1,id2,id3,id4,...] 配对）
    auto bl = this->get_parameter("blacklist_edges").as_integer_array();
    if (bl.size() % 2 != 0) {
      RCLCPP_WARN(this->get_logger(), "blacklist_edges size is odd, last id will be ignored");
      bl.pop_back();
    }
    for (size_t i = 0; i + 1 < bl.size(); i += 2) {
      options_.blacklist_edges.emplace_back(static_cast<int>(bl[i]), static_cast<int>(bl[i + 1]));
    }
    RCLCPP_INFO(this->get_logger(),
      "Graph options: max_edge_length=%.2f m, min_clearance_cells=%d, occupancy_threshold=%d, blacklist_edges=%zu",
      options_.max_edge_length, options_.min_clearance_cells, options_.occupancy_threshold,
      options_.blacklist_edges.size());

    // 创建可视化器
    visualizer_ = std::make_unique<GraphVisualizer>(this, frame_id_);

    // 订阅地图
    // map_server 发布的 /map 使用的是 transient_local QoS（latched），
    // 订阅时也要设置成 transient_local 才能收到已有的地图
    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.transient_local().reliable();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic,
      map_qos,
      std::bind(&GraphBuilderTestNode::mapCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waiting for map on topic: %s", map_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Graph will be built automatically when map is received");
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (graph_built_) {
      return;  // 只构建一次
    }

    RCLCPP_INFO(this->get_logger(), "Received map! Building graph...");
    RCLCPP_INFO(this->get_logger(), "Map size: %dx%d, resolution: %.3f",
                msg->info.width, msg->info.height, msg->info.resolution);

    // 创建地图访问器
    MapAccessor map_accessor(*msg);

    if (!map_accessor.isValid()) {
      RCLCPP_ERROR(this->get_logger(), "Invalid map!");
      return;
    }

    // 获取关键点
    const auto & keypoints = keypoint_loader_.getKeyPoints();

    // 构建图
    auto start_time = std::chrono::steady_clock::now();
    graph_ = GraphBuilder::buildGraph(keypoints, map_accessor, options_);
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time - start_time).count();

    RCLCPP_INFO(this->get_logger(), "Graph built in %ld ms", duration);
    RCLCPP_INFO(this->get_logger(), "Nodes: %zu, Edges: %zu",
                graph_.getNodeCount(), graph_.getEdgeCount() / 2);  // 除以2因为是无向图

    // 示例：用 A* 查询从关键点ID 1 到 ID 19 的路径，并打印（如果存在）
    auto find_index_by_id = [&](int id)->int {
      for (size_t i = 0; i < graph_.nodes.size(); ++i) {
        if (graph_.nodes[i].id == id) return static_cast<int>(i);
      }
      return -1;
    };

    int start_id = 9;
    int goal_id = 3;
    int start_idx = find_index_by_id(start_id);
    int goal_idx = find_index_by_id(goal_id);
    if (start_idx >= 0 && goal_idx >= 0) {
      auto astar_res = GraphBuilder::AStarSearch(graph_, start_idx, goal_idx);
      if (!astar_res.first.empty()) {
        RCLCPP_INFO(this->get_logger(), "A* path from id %d to %d cost=%.3f:", start_id, goal_id, astar_res.second);
        std::string path_s;
        for (int idx : astar_res.first) {
          const auto & kp = graph_.nodes[idx];
          RCLCPP_INFO(this->get_logger(), "  node idx=%d id=%d (%.3f, %.3f)", idx, kp.id, kp.x, kp.y);
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "A* found no path from id %d to %d", start_id, goal_id);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Start or goal id not found in keypoints (start=%d idx=%d, goal=%d idx=%d)",
        start_id, start_idx, goal_id, goal_idx);
    }

    // 可视化图
    visualizer_->visualizeGraph(graph_);
    RCLCPP_INFO(this->get_logger(), "Graph visualization published!");

    // 打印一些统计信息
    printGraphStats();

    graph_built_ = true;
  }

  void printGraphStats()
  {
    RCLCPP_INFO(this->get_logger(), "\n=== Graph Statistics ===");
    RCLCPP_INFO(this->get_logger(), "Total nodes: %zu", this->graph_.getNodeCount());
    RCLCPP_INFO(this->get_logger(), "Total edges: %zu (undirected)", this->graph_.getEdgeCount() / 2);

    // 统计每个节点的连接数
    std::vector<int> connection_counts(this->graph_.getNodeCount(), 0);
    for (const auto & edge : this->graph_.edges) {
      connection_counts[edge.from]++;
    }

    // 找出连接最多和最少的节点
    int max_connections = 0, min_connections = 1000;
    for (size_t i = 0; i < connection_counts.size(); ++i) {
      if (connection_counts[i] > max_connections) {
        max_connections = connection_counts[i];
      }
      if (connection_counts[i] < min_connections) {
        min_connections = connection_counts[i];
      }
    }

    RCLCPP_INFO(this->get_logger(), "Max connections per node: %d", max_connections);
    RCLCPP_INFO(this->get_logger(), "Min connections per node: %d", min_connections);
    RCLCPP_INFO(this->get_logger(), "========================\n");
  }

  KeyPointLoader keypoint_loader_;
  Graph graph_;
  std::unique_ptr<GraphVisualizer> visualizer_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  std::string frame_id_;
  bool graph_built_ = false;
  GraphBuildOptions options_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GraphBuilderTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

