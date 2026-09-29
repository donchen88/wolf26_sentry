#ifndef RM_BEHAVIOR_TREE__GRAPH_BUILDER_HPP_
#define RM_BEHAVIOR_TREE__GRAPH_BUILDER_HPP_

#include "rm_behavior_tree/key_point_loader.hpp"
#include "rm_behavior_tree/map_accessor.hpp"
#include "rm_behavior_tree/visibility_checker.hpp"
#include <vector>
#include <cmath>

namespace rm_behavior_tree
{

/**
 * @brief 图的边结构
 */
struct Edge
{
  int from;      // 起点索引（在 nodes 中的索引）
  int to;        // 终点索引
  double cost;   // 边的代价（距离）

  Edge(int f, int t, double c)
  : from(f), to(t), cost(c)
  {}
};

/**
 * @brief 图结构
 */
struct Graph
{
  std::vector<KeyPoint> nodes;  // 节点（关键点）
  std::vector<Edge> edges;      // 边（连通关系）

  /**
   * @brief 获取节点的邻居节点索引
   * @param node_idx 节点索引
   * @return 邻居节点索引列表
   */
  std::vector<int> getNeighbors(int node_idx) const;

  /**
   * @brief 获取两个节点之间的边代价
   * @param from_idx 起点索引
   * @param to_idx 终点索引
   * @return 边的代价，如果不存在边则返回 -1
   */
  double getEdgeCost(int from_idx, int to_idx) const;

  /**
   * @brief 获取边的数量
   */
  size_t getEdgeCount() const { return edges.size(); }

  /**
   * @brief 获取节点的数量
   */
  size_t getNodeCount() const { return nodes.size(); }
};

/**
 * @brief 图构建选项
 */
struct GraphBuildOptions
{
  double max_edge_length = 0.0;          // 最大连边距离（米），<=0 表示不限制
  int min_clearance_cells = 0;           // Raycast 安全间隔（栅格数）
  int occupancy_threshold = 50;          // 占用阈值
  std::vector<std::pair<int, int>> blacklist_edges;  // 黑名单边（按关键点ID）
};

/**
 * @brief 图构建器
 */
class GraphBuilder
{
public:
  /**
   * @brief 从关键点和地图构建图
   * @param keypoints 关键点列表
   * @param map 地图访问器
   * @return 构建好的图
   */
  static Graph buildGraph(
    const std::vector<KeyPoint> & keypoints,
    MapAccessor map,
    const GraphBuildOptions & options = {});

private:
  /**
   * @brief 计算两点之间的欧氏距离
   */
  static double calculateDistance(const KeyPoint & a, const KeyPoint & b);

  static bool isBlacklisted(
    int id_a, int id_b,
    const std::vector<std::pair<int, int>> & blacklist);
  
public:
  /**
   * @brief A* search from start index to goal index on the graph
   * @param graph input graph
   * @param start_idx index of start node in graph.nodes
   * @param goal_idx index of goal node in graph.nodes
   * @return pair of (path as list of node indices, total cost). If no path, path empty and cost = infinity.
   */
  static std::pair<std::vector<int>, double> AStarSearch(
    const Graph & graph,
    int start_idx,
    int goal_idx);
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__GRAPH_BUILDER_HPP_

