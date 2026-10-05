#ifndef NAV2_PLUGINS__LAYERS__ESCAPE_BIAS_LAYER_HPP_
#define NAV2_PLUGINS__LAYERS__ESCAPE_BIAS_LAYER_HPP_

#include <vector>
#include <memory>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_plugins
{

class EscapeBiasLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  EscapeBiasLayer();
  ~EscapeBiasLayer() override;

  // Lifecycle / initialization
  void onInitialize() override;

  // Costmap update hooks
  void updateBounds(
    double robot_x,
    double robot_y,
    double robot_yaw,
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j) override;

  // ❗ 必须实现 Layer 的纯虚函数
  void reset() override;
  bool isClearable() override;

private:
  // ---- parameters ----
  int sector_count_;
  double sample_radius_min_;
  double sample_radius_max_;
  double bias_cost_;

  // ---- runtime state ----
  double robot_x_;
  double robot_y_;
  double robot_yaw_;
  bool robot_pose_set_;
};

}  // namespace nav2_plugins

#endif  // NAV2_PLUGINS__LAYERS__ESCAPE_BIAS_LAYER_HPP_
