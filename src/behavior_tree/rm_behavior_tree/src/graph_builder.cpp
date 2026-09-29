#include "rm_behavior_tree/graph_builder.hpp"
#include <cmath>
#include <unordered_map>
#include <queue>
#include <limits>
#include <functional>

namespace rm_behavior_tree
{

std::vector<int> Graph::getNeighbors(int node_idx) const
{
  std::vector<int> neighbors;
  for (const auto & edge : edges) {
    if (edge.from == node_idx) {
      neighbors.push_back(edge.to);
    }
  }
  return neighbors;
}

double Graph::getEdgeCost(int from_idx, int to_idx) const
{
  for (const auto & edge : edges) {
    if (edge.from == from_idx && edge.to == to_idx) {
      return edge.cost;
    }
  }
  return -1.0;  // 边不存在
}

double GraphBuilder::calculateDistance(const KeyPoint & a, const KeyPoint & b)
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

bool GraphBuilder::isBlacklisted(
  int id_a, int id_b,
  const std::vector<std::pair<int, int>> & blacklist)
{
  for (const auto & p : blacklist) {
    if ((p.first == id_a && p.second == id_b) ||
        (p.first == id_b && p.second == id_a)) {
      return true;
    }
  }
  return false;
}

Graph GraphBuilder::buildGraph(
  const std::vector<KeyPoint> & keypoints,
  MapAccessor map,
  const GraphBuildOptions & options)
{
  Graph graph;
  graph.nodes = keypoints;

  // 应用占用阈值
  map.setOccupiedThreshold(options.occupancy_threshold);

  // 建立 id -> index 映射，便于黑名单检查
  std::unordered_map<int, int> id_to_index;
  for (size_t i = 0; i < keypoints.size(); ++i) {
    id_to_index[keypoints[i].id] = static_cast<int>(i);
  }

  // 两两检查关键点之间的连通性
  for (size_t i = 0; i < keypoints.size(); ++i) {
    for (size_t j = i + 1; j < keypoints.size(); ++j) {
      // 黑名单过滤
      if (isBlacklisted(keypoints[i].id, keypoints[j].id, options.blacklist_edges)) {
        continue;
      }

      // 距离过滤
      double dist = calculateDistance(keypoints[i], keypoints[j]);
      if (options.max_edge_length > 0.0 && dist > options.max_edge_length) {
        continue;
      }

      // 检查两点是否可见（直线可达）
      if (VisibilityChecker::isVisible(
            keypoints[i], keypoints[j], map, options.min_clearance_cells)) {

        // 添加双向边（无向图）
        graph.edges.emplace_back(i, j, dist);
        graph.edges.emplace_back(j, i, dist);
      }
    }
  }

  return graph;
}

std::pair<std::vector<int>, double> GraphBuilder::AStarSearch(
  const Graph & graph,
  int start_idx,
  int goal_idx)
{
  std::pair<std::vector<int>, double> result;
  result.second = std::numeric_limits<double>::infinity();

  if (start_idx < 0 || goal_idx < 0 ||
      start_idx >= static_cast<int>(graph.getNodeCount()) ||
      goal_idx >= static_cast<int>(graph.getNodeCount())) {
    return result;
  }

  const int n = static_cast<int>(graph.getNodeCount());
  const double INF = std::numeric_limits<double>::infinity();

  std::vector<double> g_score(n, INF);
  std::vector<double> f_score(n, INF);
  std::vector<int> came_from(n, -1);

  auto heuristic = [&](int idx) {
    return calculateDistance(graph.nodes[idx], graph.nodes[goal_idx]);
  };

  using PQItem = std::pair<double, int>; // f_score, idx
  std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> open_set;

  g_score[start_idx] = 0.0;
  f_score[start_idx] = heuristic(start_idx);
  open_set.emplace(f_score[start_idx], start_idx);

  std::vector<char> in_closed(n, 0);

  while (!open_set.empty()) {
    auto [f_cur, current] = open_set.top();
    open_set.pop();

    if (in_closed[current]) continue;
    in_closed[current] = 1;

    if (current == goal_idx) {
      // reconstruct path
      std::vector<int> path;
      int cur = current;
      while (cur != -1) {
        path.push_back(cur);
        cur = came_from[cur];
      }
      std::reverse(path.begin(), path.end());
      result.first = path;
      result.second = g_score[goal_idx];
      return result;
    }

    // explore neighbors
    auto neighbors = graph.getNeighbors(current);
    for (int nb : neighbors) {
      double edge_cost = graph.getEdgeCost(current, nb);
      if (edge_cost < 0) continue;
      double tentative_g = g_score[current] + edge_cost;
      if (tentative_g < g_score[nb]) {
        g_score[nb] = tentative_g;
        came_from[nb] = current;
        f_score[nb] = tentative_g + heuristic(nb);
        open_set.emplace(f_score[nb], nb);
      }
    }
  }

  // no path
  return result;
}

}  // namespace rm_behavior_tree

