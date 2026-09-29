#ifndef RM_BEHAVIOR_TREE__GRAPH_VISUALIZER_HPP_
#define RM_BEHAVIOR_TREE__GRAPH_VISUALIZER_HPP_

#include "rm_behavior_tree/graph_builder.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <mutex>

namespace rm_behavior_tree
{

/**
 * @brief 图可视化器
 *        将图结构发布为 rviz2 可显示的 Marker
 */
class GraphVisualizer
{
public:
  /**
   * @brief 构造函数
   * @param node ROS2 节点（用于创建发布器）
   * @param frame_id 坐标系（通常是 "map"）
   */
  GraphVisualizer(rclcpp::Node * node, const std::string & frame_id = "map");

  /**
   * @brief 可视化图结构
   * @param graph 要可视化的图
   */
  void visualizeGraph(const Graph & graph);

  /**
   * @brief 清除所有可视化标记
   */
  void clearMarkers();

private:
  /**
   * @brief 创建关键点标记（球体）
   */
  visualization_msgs::msg::Marker createNodeMarker(
    const KeyPoint & point,
    int id,
    const std::string & ns = "keypoints");

  /**
   * @brief 创建边标记（线段）
   */
  visualization_msgs::msg::Marker createEdgeMarker(
    const Graph & graph,
    const std::string & ns = "edges");

  rclcpp::Node * node_;
  std::string frame_id_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  int marker_id_counter_;
  // 保存最后一次发布的 MarkerArray，周期性重复发布以保证新订阅者能看到
  visualization_msgs::msg::MarkerArray last_marker_array_;
  std::mutex marker_mutex_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__GRAPH_VISUALIZER_HPP_

