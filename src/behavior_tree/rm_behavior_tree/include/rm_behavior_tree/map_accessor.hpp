#ifndef RM_BEHAVIOR_TREE__MAP_ACCESSOR_HPP_
#define RM_BEHAVIOR_TREE__MAP_ACCESSOR_HPP_

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace rm_behavior_tree
{

/**
 * @brief 地图访问器
 *        提供世界坐标到栅格坐标的转换，以及障碍物检查功能
 *        
 *        注意：
 *        - 关键点的 x, y 坐标已经是世界坐标（map坐标系），可以直接用于导航
 *        - 坐标转换仅用于内部实现：访问地图栅格数据、Raycast 检查
 *        - 最终发给导航系统的坐标不需要转换，直接使用关键点的 x, y
 */
class MapAccessor
{
public:
  /**
   * @brief 构造函数
   * @param map 占用栅格地图
   */
  explicit MapAccessor(const nav_msgs::msg::OccupancyGrid & map);

  /**
   * @brief 世界坐标转栅格坐标
   * @param wx 世界坐标 X
   * @param wy 世界坐标 Y
   * @param mx 输出：栅格坐标 X
   * @param my 输出：栅格坐标 Y
   * @return 是否转换成功（坐标在地图范围内）
   */
  bool worldToMap(double wx, double wy, int & mx, int & my) const;

  /**
   * @brief 栅格坐标转世界坐标
   * @param mx 栅格坐标 X
   * @param my 栅格坐标 Y
   * @param wx 输出：世界坐标 X
   * @param wy 输出：世界坐标 Y
   */
  void mapToWorld(int mx, int my, double & wx, double & wy) const;

  /**
   * @brief 判断栅格是否被占用（障碍物）
   * @param mx 栅格坐标 X
   * @param my 栅格坐标 Y
   * @return true 表示被占用（障碍物），false 表示可通行
   */
  bool isOccupied(int mx, int my) const;

  /**
   * @brief 判断世界坐标点是否被占用
   * @param wx 世界坐标 X
   * @param wy 世界坐标 Y
   * @return true 表示被占用，false 表示可通行
   */
  bool isOccupiedWorld(double wx, double wy) const;

  /**
   * @brief 设置占用阈值（0-100，默认50）
   */
  void setOccupiedThreshold(int threshold) { occupied_threshold_ = threshold; }

  /**
   * @brief 获取占用阈值
   */
  int getOccupiedThreshold() const { return occupied_threshold_; }

  /**
   * @brief 检查地图是否有效
   */
  bool isValid() const { return map_.data.size() > 0; }

  /**
   * @brief 获取地图分辨率
   */
  double getResolution() const { return resolution_; }

  /**
   * @brief 获取地图宽度（栅格数）
   */
  int getWidth() const { return width_; }

  /**
   * @brief 获取地图高度（栅格数）
   */
  int getHeight() const { return height_; }

private:
  nav_msgs::msg::OccupancyGrid map_;
  double resolution_;
  double origin_x_, origin_y_;
  int width_, height_;
  int occupied_threshold_;  // 占用阈值，默认 50
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__MAP_ACCESSOR_HPP_

