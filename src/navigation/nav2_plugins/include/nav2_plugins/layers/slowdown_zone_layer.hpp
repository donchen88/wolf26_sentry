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

#ifndef NAV2_PLUGINS__LAYERS__SLOWDOWN_ZONE_LAYER_HPP_
#define NAV2_PLUGINS__LAYERS__SLOWDOWN_ZONE_LAYER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_plugins
{

class SlowdownZoneLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  SlowdownZoneLayer() = default;
  ~SlowdownZoneLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override { return false; }

private:
  void slowdownMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  bool isSlowdownCell(int8_t value) const;
  bool isForbiddenCell(int8_t value) const;
  void precomputeDistField();

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr slowdown_map_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr slowdown_map_;
  mutable std::mutex map_mutex_;

  std::string map_topic_;
  unsigned char slowdown_cost_{80};
  int zone_threshold_{50};
  int forbidden_threshold_{100};
  unsigned char forbidden_cost_{80};
  double inflation_dist_{0.0};
  unsigned char inflation_min_cost_{10};
  std::vector<float> dist_field_;     // dist to nearest slowdown cell (threshold)
  int dist_field_width_{0};
  int dist_field_height_{0};
  bool dist_field_valid_{false};
  std::vector<float> forbidden_dist_field_;  // dist to nearest forbidden cell (255)
  std::mutex dist_field_mutex_;
};

}  // namespace nav2_plugins

#endif  // NAV2_PLUGINS__LAYERS__SLOWDOWN_ZONE_LAYER_HPP_
