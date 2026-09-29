#ifndef RM_BEHAVIOR_TREE__VISIBILITY_CHECKER_HPP_
#define RM_BEHAVIOR_TREE__VISIBILITY_CHECKER_HPP_

#include "rm_behavior_tree/key_point_loader.hpp"
#include "rm_behavior_tree/map_accessor.hpp"
#include <cmath>

namespace rm_behavior_tree
{

/**
 * @brief 可见性检查器
 *        使用 Bresenham 算法进行 Raycast，判断两点之间是否直线可达
 *        
 *        注意：
 *        - 输入的 KeyPoint 坐标是世界坐标（map坐标系），可以直接用于导航
 *        - 内部会将世界坐标转换为栅格坐标，用于访问地图数据
 *        - 这是内部实现细节，不影响关键点的使用
 */
class VisibilityChecker
{
public:
  /**
   * @brief 检查两个关键点是否可见（直线可达）
   * @param a 起点关键点
   * @param b 终点关键点
   * @param map 地图访问器
   * @param min_clearance_cells 最小安全间隔（以栅格为单位），检查线段附近邻域
   * @return true 表示两点之间无障碍物，可以直线到达
   */
  static bool isVisible(
    const KeyPoint & a,
    const KeyPoint & b,
    const MapAccessor & map,
    int min_clearance_cells = 0);

private:
  /**
   * @brief Bresenham 直线算法
   *        从 (x0, y0) 到 (x1, y1) 画线，检查路径上的每个点
   */
  static bool bresenhamLine(
    int x0, int y0,
    int x1, int y1,
    const MapAccessor & map,
    int min_clearance_cells);
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__VISIBILITY_CHECKER_HPP_

