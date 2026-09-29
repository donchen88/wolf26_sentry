#include "rm_behavior_tree/map_accessor.hpp"
#include <cmath>

namespace rm_behavior_tree
{

MapAccessor::MapAccessor(const nav_msgs::msg::OccupancyGrid & map)
: map_(map), occupied_threshold_(50)
{
  resolution_ = map_.info.resolution;
  origin_x_ = map_.info.origin.position.x;
  origin_y_ = map_.info.origin.position.y;
  width_ = static_cast<int>(map_.info.width);
  height_ = static_cast<int>(map_.info.height);
}

bool MapAccessor::worldToMap(double wx, double wy, int & mx, int & my) const
{
  mx = static_cast<int>((wx - origin_x_) / resolution_);
  my = static_cast<int>((wy - origin_y_) / resolution_);

  if (mx < 0 || my < 0 || mx >= width_ || my >= height_) {
    return false;
  }

  return true;
}

void MapAccessor::mapToWorld(int mx, int my, double & wx, double & wy) const
{
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

bool MapAccessor::isOccupied(int mx, int my) const
{
  if (mx < 0 || my < 0 || mx >= width_ || my >= height_) {
    return true;  // 越界视为障碍物
  }

  int idx = my * width_ + mx;
  if (idx < 0 || idx >= static_cast<int>(map_.data.size())) {
    return true;  // 索引越界视为障碍物
  }

  int value = map_.data[idx];
  // 占用值 > 50 视为障碍物，-1(未知)和0(空闲)视为可通行
  return value > occupied_threshold_;
}

bool MapAccessor::isOccupiedWorld(double wx, double wy) const
{
  int mx, my;
  if (!worldToMap(wx, wy, mx, my)) {
    return true;  // 越界视为障碍物
  }
  int value = map_.data[my * width_ + mx];
  return value > occupied_threshold_;
}

}  // namespace rm_behavior_tree

