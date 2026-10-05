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

#include "nav2_plugins/layers/slowdown_zone_layer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_plugins
{

void SlowdownZoneLayer::onInitialize()
{
  auto node = node_.lock();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("map_topic", rclcpp::ParameterValue(std::string("/slowdown_map")));
  declareParameter("slowdown_cost", rclcpp::ParameterValue(80));
  declareParameter("zone_threshold", rclcpp::ParameterValue(50));
  declareParameter("forbidden_cost", rclcpp::ParameterValue(80));
  declareParameter("inflation_dist", rclcpp::ParameterValue(0.0));
  declareParameter("inflation_min_cost", rclcpp::ParameterValue(10));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".map_topic", map_topic_);

  int slowdown_cost = 80;
  int forbidden_cost = 80;
  int inflation_min_cost = 10;
  node->get_parameter(name_ + ".slowdown_cost", slowdown_cost);
  node->get_parameter(name_ + ".zone_threshold", zone_threshold_);
  node->get_parameter(name_ + ".forbidden_cost", forbidden_cost);
  node->get_parameter(name_ + ".inflation_dist", inflation_dist_);
  node->get_parameter(name_ + ".inflation_min_cost", inflation_min_cost);
  slowdown_cost_ = static_cast<unsigned char>(std::clamp(slowdown_cost, 0, 252));
  forbidden_cost_ = static_cast<unsigned char>(std::clamp(forbidden_cost, 0, 252));
  inflation_min_cost_ = static_cast<unsigned char>(std::clamp(inflation_min_cost, 0, 252));

  slowdown_map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&SlowdownZoneLayer::slowdownMapCallback, this, std::placeholders::_1));

  current_ = true;
}

void SlowdownZoneLayer::slowdownMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    slowdown_map_ = msg;
  }
  precomputeDistField();
}

bool SlowdownZoneLayer::isSlowdownCell(int8_t value) const { return value >= zone_threshold_; }

bool SlowdownZoneLayer::isForbiddenCell(int8_t value) const { return value >= forbidden_threshold_; }

void SlowdownZoneLayer::precomputeDistField()
{
  nav_msgs::msg::OccupancyGrid::SharedPtr local_map;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!slowdown_map_) {
      std::lock_guard<std::mutex> df_lock(dist_field_mutex_);
      dist_field_valid_ = false;
      return;
    }
    local_map = slowdown_map_;
  }

  const auto & info = local_map->info;
  const int width = static_cast<int>(info.width);
  const int height = static_cast<int>(info.height);
  const auto & data = local_map->data;
  const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);

  const int max_search_cells = inflation_dist_ > 0.0
    ? static_cast<int>(std::ceil(inflation_dist_ / info.resolution)) + 1
    : 0;

  // Both distance fields initialized to infinity
  std::vector<float> dist(n, std::numeric_limits<float>::infinity());
  std::vector<float> forb_dist(n, std::numeric_limits<float>::infinity());
  std::vector<int8_t> visited(n, 0);
  std::queue<std::tuple<int, int, float>> q;

  // BFS seeds: normal slowdown cells and forbidden (255) cells separately
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
      if (idx >= data.size()) continue;
      if (isForbiddenCell(data[idx])) {
        forb_dist[idx] = 0.0f;
        dist[idx] = 0.0f;   // forbidden is also a slowdown cell
        visited[idx] = 1;
        q.emplace(x, y, 0.0f);
      } else if (isSlowdownCell(data[idx])) {
        dist[idx] = 0.0f;
        visited[idx] = 1;
        q.emplace(x, y, 0.0f);
      }
    }
  }

  const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
  const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};

  while (!q.empty()) {
    auto [cx, cy, d] = q.front();
    q.pop();

    if (d >= max_search_cells && max_search_cells > 0) {
      continue;
    }

    for (int dir = 0; dir < 8; ++dir) {
      const int nx = cx + dx[dir];
      const int ny = cy + dy[dir];
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
        continue;
      }
      const size_t nidx = static_cast<size_t>(ny) * static_cast<size_t>(width) + static_cast<size_t>(nx);
      const float step = (dir % 2 == 0) ? 1.41421356f : 1.0f;
      const float nd = d + step;

      bool pushed = false;
      if (nd < dist[nidx]) {
        dist[nidx] = nd;
        visited[nidx] = 1;
        pushed = true;
      }
      if (nd < forb_dist[nidx]) {
        forb_dist[nidx] = nd;
        visited[nidx] = 1;
        pushed = true;
      }
      if (pushed && max_search_cells > 0 && nd < max_search_cells) {
        q.emplace(nx, ny, nd);
      }
    }
  }

  {
    std::lock_guard<std::mutex> df_lock(dist_field_mutex_);
    dist_field_width_ = width;
    dist_field_height_ = height;
    dist_field_ = std::move(dist);
    forbidden_dist_field_ = std::move(forb_dist);
    dist_field_valid_ = true;
  }
}

void SlowdownZoneLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (!enabled_ || !slowdown_map_) {
    return;
  }

  const auto & info = slowdown_map_->info;
  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;
  const double res = info.resolution;

  double extra = inflation_dist_;
  *min_x = std::min(*min_x, origin_x - extra);
  *min_y = std::min(*min_y, origin_y - extra);
  *max_x = std::max(*max_x, origin_x + info.width * res + extra);
  *max_y = std::max(*max_y, origin_y + info.height * res + extra);
}

void SlowdownZoneLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (!enabled_ || !slowdown_map_) {
    return;
  }

  const auto & info = slowdown_map_->info;
  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;
  const double resolution = info.resolution;
  const auto & slowmap_data = slowdown_map_->data;

  std::lock_guard<std::mutex> df_lock(dist_field_mutex_);
  if (!dist_field_valid_ || dist_field_width_ != static_cast<int>(info.width) ||
    dist_field_height_ != static_cast<int>(info.height))
  {
    return;
  }

  const float inf = std::numeric_limits<float>::infinity();
  const float range_cells = inflation_dist_ > 0.0
    ? static_cast<float>(inflation_dist_ / resolution)
    : 0.0f;
  const bool do_inflation = range_cells > 0.0f;

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);

      const int sx = static_cast<int>((wx - origin_x) / resolution);
      const int sy = static_cast<int>((wy - origin_y) / resolution);

      unsigned char final_cost = 0;

      if (sx >= 0 && sy >= 0 && sx < static_cast<int>(info.width) &&
          sy < static_cast<int>(info.height))
      {
        const size_t idx = static_cast<size_t>(sy) * info.width + static_cast<size_t>(sx);
        if (idx < slowmap_data.size()) {
          const int8_t value = slowmap_data[idx];
          if (isForbiddenCell(value)) {
            continue;
          } else if (isSlowdownCell(value)) {
            final_cost = slowdown_cost_;
          } else if (do_inflation) {
            unsigned char forb_inflation_cost = 0;
            unsigned char normal_inflation_cost = 0;
            if (idx < forbidden_dist_field_.size()) {
              const float fd = forbidden_dist_field_[idx];
              if (fd < inf && fd <= range_cells) {
                const double t = 1.0 - static_cast<double>(fd) / range_cells;
                forb_inflation_cost = static_cast<unsigned char>(
                  std::round(inflation_min_cost_ + (forbidden_cost_ - inflation_min_cost_) * t));
              }
            }
            if (idx < dist_field_.size()) {
              const float d = dist_field_[idx];
              if (d < inf && d <= range_cells) {
                const double t = 1.0 - static_cast<double>(d) / range_cells;
                normal_inflation_cost = static_cast<unsigned char>(
                  std::round(inflation_min_cost_ + (slowdown_cost_ - inflation_min_cost_) * t));
              }
            }
            final_cost = std::max(forb_inflation_cost, normal_inflation_cost);
          }
        }
      }

      if (final_cost > 0) {
        const unsigned char old_cost = master_grid.getCost(i, j);
        if (old_cost < nav2_costmap_2d::LETHAL_OBSTACLE) {
          master_grid.setCost(i, j, std::max(old_cost, final_cost));
        }
      }
    }
  }
}

void SlowdownZoneLayer::reset() { current_ = true; }

}  // namespace nav2_plugins

PLUGINLIB_EXPORT_CLASS(nav2_plugins::SlowdownZoneLayer, nav2_costmap_2d::Layer)
