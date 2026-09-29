#ifndef RM_BEHAVIOR_TREE__KEY_POINT_LOADER_HPP_
#define RM_BEHAVIOR_TREE__KEY_POINT_LOADER_HPP_

#include <string>
#include <vector>
#include <geometry_msgs/msg/point.hpp>

namespace rm_behavior_tree
{

/**
 * @brief 关键点数据结构
 *        
 *        注意：x, y 坐标是世界坐标（map坐标系），可以直接用于导航
 *        例如：可以直接发送给 navigate_to_pose，无需坐标转换
 */
struct KeyPoint
{
  int id;                    // 关键点ID
  double x;                  // X坐标（世界坐标，map坐标系，单位：米）
  double y;                  // Y坐标（世界坐标，map坐标系，单位：米）
  std::string description;   // 描述（中文注释）
  
  // 转换为 geometry_msgs::msg::Point
  geometry_msgs::msg::Point toPoint() const
  {
    geometry_msgs::msg::Point pt;
    pt.x = x;
    pt.y = y;
    pt.z = 0.0;
    return pt;
  }
};

/**
 * @brief 关键点加载器
 *        从 YAML 配置文件中加载关键点数据
 */
class KeyPointLoader
{
public:
  /**
   * @brief 从 YAML 文件加载关键点
   * @param yaml_path YAML 文件路径
   * @return 是否加载成功
   */
  bool loadFromFile(const std::string & yaml_path);

  /**
   * @brief 获取所有关键点
   */
  const std::vector<KeyPoint> & getKeyPoints() const { return keypoints_; }

  /**
   * @brief 根据ID获取关键点
   * @param id 关键点ID
   * @return 关键点指针，如果不存在返回 nullptr
   */
  const KeyPoint * getKeyPointById(int id) const;

  /**
   * @brief 获取关键点数量
   */
  size_t size() const { return keypoints_.size(); }

private:
  std::vector<KeyPoint> keypoints_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__KEY_POINT_LOADER_HPP_

