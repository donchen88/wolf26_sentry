#include "rm_behavior_tree/visibility_checker.hpp"
#include <cmath>
#include <algorithm>

namespace rm_behavior_tree
{

bool VisibilityChecker::isVisible(
  const KeyPoint & a,
  const KeyPoint & b,
  const MapAccessor & map,
  int min_clearance_cells)
{
  // 将世界坐标转换为栅格坐标
  int x0, y0, x1, y1;
  if (!map.worldToMap(a.x, a.y, x0, y0)) {
    return false;  // 起点不在地图范围内
  }
  if (!map.worldToMap(b.x, b.y, x1, y1)) {
    return false;  // 终点不在地图范围内
  }

  // 检查起点和终点本身是否被占用
  if (map.isOccupied(x0, y0) || map.isOccupied(x1, y1)) {
    return false;
  }

  // 使用 Bresenham 算法检查路径
  return bresenhamLine(x0, y0, x1, y1, map, min_clearance_cells);
}

bool VisibilityChecker::bresenhamLine(
  int x0, int y0,
  int x1, int y1,
  const MapAccessor & map,
  int min_clearance_cells)
{
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int x = x0;
  int y = y0;

  int wall_count = 0;
  int total_points = 0;

  while (true) {
    // 检查当前点以及邻域是否被占用
    for (int ddx = -min_clearance_cells; ddx <= min_clearance_cells; ++ddx) {
      for (int ddy = -min_clearance_cells; ddy <= min_clearance_cells; ++ddy) {
        if (map.isOccupied(x + ddx, y + ddy)) {
          wall_count++;
        }
      }
    }
    total_points++;

    // 到达终点
    if (x == x1 && y == y1) {
      break;
    }

    // Bresenham 算法核心
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }

  // 如果路径上有超过20%的点被占用，认为有墙
  double wall_ratio = static_cast<double>(wall_count) / total_points;
  return wall_ratio < 0.2;
}

}  // namespace rm_behavior_tree

