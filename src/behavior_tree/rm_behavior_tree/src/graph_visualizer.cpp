#include "rm_behavior_tree/graph_visualizer.hpp"

namespace rm_behavior_tree
{

GraphVisualizer::GraphVisualizer(rclcpp::Node * node, const std::string & frame_id)
: node_(node), frame_id_(frame_id), marker_id_counter_(0)
{
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliable().transient_local();  // 让新订阅者也能收到最近一次发布

  marker_array_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "graph_visualization", qos);
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
    "graph_markers", qos);

  // 周期性发布 timer（1 Hz），确保即使 rviz 之后订阅也能收到最新 MarkerArray
  publish_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(1000),
    [this]() {
      std::lock_guard<std::mutex> lock(marker_mutex_);
      if (!last_marker_array_.markers.empty()) {
        marker_array_pub_->publish(last_marker_array_);
      }
    });
}

void GraphVisualizer::visualizeGraph(const Graph & graph)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // 1. 可视化关键点（球体）
  for (size_t i = 0; i < graph.nodes.size(); ++i) {
    auto node_marker = createNodeMarker(graph.nodes[i], static_cast<int>(i));
    marker_array.markers.push_back(node_marker);
  }

  // 2. 可视化边（线段）
  auto edge_marker = createEdgeMarker(graph);
  marker_array.markers.push_back(edge_marker);

  // 3. 发布标记
  {
    std::lock_guard<std::mutex> lock(marker_mutex_);
    last_marker_array_ = marker_array;  // 保存最新的用于周期发布
  }
  marker_array_pub_->publish(marker_array);
}

void GraphVisualizer::clearMarkers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  
  // 创建删除标记
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  delete_marker.header.frame_id = frame_id_;
  delete_marker.header.stamp = node_->now();
  
  marker_array.markers.push_back(delete_marker);
  marker_array_pub_->publish(marker_array);
}

visualization_msgs::msg::Marker GraphVisualizer::createNodeMarker(
  const KeyPoint & point,
  int id,
  const std::string & ns)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = node_->now();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // 位置
  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.position.z = 0.1;  // 稍微抬高，避免贴地
  marker.pose.orientation.w = 1.0;

  // 大小
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  // 颜色（绿色）
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  // 文本标签（显示 ID）
  marker.text = std::to_string(point.id);

  marker.lifetime = rclcpp::Duration(0, 0);  // 永久显示

  return marker;
}

visualization_msgs::msg::Marker GraphVisualizer::createEdgeMarker(
  const Graph & graph,
  const std::string & ns)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = node_->now();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // 位置（不需要，因为使用点列表）
  marker.pose.orientation.w = 1.0;

  // 线宽
  marker.scale.x = 0.05;  // 线宽

  // 颜色（蓝色）
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 0.8;

  // 添加所有边的点对
  for (const auto & edge : graph.edges) {
    // 只添加一次边（避免重复，因为是无向图）
    if (edge.from < edge.to) {
      geometry_msgs::msg::Point p1, p2;
      p1.x = graph.nodes[edge.from].x;
      p1.y = graph.nodes[edge.from].y;
      p1.z = 0.1;

      p2.x = graph.nodes[edge.to].x;
      p2.y = graph.nodes[edge.to].y;
      p2.z = 0.1;

      marker.points.push_back(p1);
      marker.points.push_back(p2);
    }
  }

  marker.lifetime = rclcpp::Duration(0, 0);  // 永久显示

  return marker;
}

}  // namespace rm_behavior_tree

