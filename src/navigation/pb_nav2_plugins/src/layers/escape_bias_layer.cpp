// pb_nav2_plugins/src/layers/escape_bias_layer.cpp
#include "pb_nav2_plugins/layers/escape_bias_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
#include <algorithm>

PLUGINLIB_EXPORT_CLASS(escape_bias_layer::EscapeBiasLayer, nav2_costmap_2d::Layer)

using namespace escape_bias_layer;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;

EscapeBiasLayer::EscapeBiasLayer()
: sector_count_(16),
  sample_radius_min_(0.3),
  sample_radius_max_(0.8),
  bias_cost_(80.0),
  robot_x_(0.0),
  robot_y_(0.0),
  robot_yaw_(0.0),
  robot_pose_set_(false)
{}

EscapeBiasLayer::~EscapeBiasLayer() {}

void EscapeBiasLayer::onInitialize()
{
  // 调用基类的初始化（设置 node_/name_/logger_/等）
  nav2_costmap_2d::CostmapLayer::onInitialize();

  declareParameter("sector_count", rclcpp::ParameterValue(16));
  declareParameter("sample_radius_min", rclcpp::ParameterValue(0.3));
  declareParameter("sample_radius_max", rclcpp::ParameterValue(0.8));
  declareParameter("bias_cost", rclcpp::ParameterValue(80.0));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + ".sector_count", sector_count_);
  node->get_parameter(name_ + ".sample_radius_min", sample_radius_min_);
  node->get_parameter(name_ + ".sample_radius_max", sample_radius_max_);
  node->get_parameter(name_ + ".bias_cost", bias_cost_);

  // 确保与主 costmap 大小匹配（如果需要）
  matchSize();

  robot_pose_set_ = false;
}

void EscapeBiasLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  // record robot pose for use in updateCosts
  robot_x_ = robot_x;
  robot_y_ = robot_y;
  robot_yaw_ = robot_yaw;
  robot_pose_set_ = true;

  // ensure costmap is updated at least in robot vicinity
  useExtraBounds(min_x, min_y, max_x, max_y);
}

void EscapeBiasLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !robot_pose_set_) {
    return;
  }

  const double angle_step = 2.0 * M_PI / static_cast<double>(sector_count_);

  // ============================================================
  // Step 1: 收集每个扇区的 cost 样本和梯度
  // ============================================================
  const int samples_per_sector = 8;  // 每个扇区采 8 个角度
  const int radial_samples = 4;       // 每个角度采 4 个距离

  std::vector<double> sector_scores(sector_count_, 0.0);
  std::vector<int> sector_counts(sector_count_, 0);
  std::vector<double> sector_gradients(sector_count_, -1000.0);

  for (int s = 0; s < sector_count_; ++s) {
    double angle_start = s * angle_step;
    double inner_cost_sum = 0.0, outer_cost_sum = 0.0;
    int inner_count = 0, outer_count = 0;

    for (int sa = 0; sa < samples_per_sector; ++sa) {
      // 采样角度：均匀分布在扇区内
      double angle = angle_start + (sa + 0.5) * angle_step / samples_per_sector;

      for (int r = 0; r < radial_samples; ++r) {
        // 采样距离：从 sample_radius_min_ 到 sample_radius_max_
        double frac = (r + 1) / static_cast<double>(radial_samples + 1);
        double dist = sample_radius_min_ + frac * (sample_radius_max_ - sample_radius_min_);

        double wx = robot_x_ + dist * cos(angle);
        double wy = robot_y_ + dist * sin(angle);

        unsigned int mx, my;
        if (!master_grid.worldToMap(wx, wy, mx, my)) {
          continue;
        }
        unsigned char cost = master_grid.getCost(mx, my);
        sector_scores[s] += static_cast<double>(cost);
        sector_counts[s] += 1;

        // 收集内圈和外圈的 cost 用于计算梯度
        if (r < radial_samples / 2) {
          inner_cost_sum += cost;
          inner_count++;
        } else {
          outer_cost_sum += cost;
          outer_count++;
        }
      }
    }

    // 计算平均 cost
    if (sector_counts[s] > 0) {
      sector_scores[s] /= sector_counts[s];
    } else {
      sector_scores[s] = 255.0;
    }

    // 计算梯度：内圈到外圈的 cost 变化
    if (inner_count > 0 && outer_count > 0) {
      double inner_avg = inner_cost_sum / inner_count;
      double outer_avg = outer_cost_sum / outer_count;
      sector_gradients[s] = inner_avg - outer_avg;  // 正值 = 往外 cost 降低
    }
  }

  // ============================================================
  // Step 2: 结合平均 cost 和梯度，选择最佳逃生方向
  // ============================================================
  int escape_sector = 0;
  double best_combined_score = sector_scores[0] * 2.0 - sector_gradients[0];

  for (int s = 1; s < sector_count_; ++s) {
    // 综合评分 = 平均 cost * 权重 + (-梯度)
    // 梯度越大（正值），减去的越多，分数越低 = 越好
    double combined = sector_scores[s] * 2.0 - sector_gradients[s];
    if (combined < best_combined_score) {
      best_combined_score = combined;
      escape_sector = s;
    }
  }

  // ============================================================
  // Step 3: 应用 bias：非 escape_sector 区域加 cost
  // ============================================================
  double bias_radius = sample_radius_max_;
  int grid_min_x = std::max(min_i, 0);
  int grid_min_y = std::max(min_j, 0);
  int grid_max_x = std::min(max_i, static_cast<int>(master_grid.getSizeInCellsX()) - 1);
  int grid_max_y = std::min(max_j, static_cast<int>(master_grid.getSizeInCellsY()) - 1);

  for (int j = grid_min_y; j <= grid_max_y; ++j) {
    for (int i = grid_min_x; i <= grid_max_x; ++i) {
      double wx, wy;
      master_grid.mapToWorld(static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);
      double dx = wx - robot_x_;
      double dy = wy - robot_y_;
      double dist = hypot(dx, dy);
      if (dist > bias_radius) continue;

      double angle = atan2(dy, dx);
      if (angle < 0) angle += 2.0 * M_PI;
      int sector = static_cast<int>(angle / angle_step) % sector_count_;

      if (sector != escape_sector) {
        unsigned int mx = static_cast<unsigned int>(i);
        unsigned int my = static_cast<unsigned int>(j);

        unsigned char old_cost = master_grid.getCost(mx, my);

        // 不修改 unknown / lethal 的 cell
        if (old_cost == NO_INFORMATION || old_cost == LETHAL_OBSTACLE) {
          continue;
        }

        int new_cost = static_cast<int>(old_cost) + static_cast<int>(bias_cost_);
        if (new_cost > 254) new_cost = 254;

        master_grid.setCost(mx, my, static_cast<unsigned char>(new_cost));
      }
    }
  }
}

void EscapeBiasLayer::reset()
{
  // 如果没有内部维护独立 map，只需重置 pose 状态
  robot_pose_set_ = false;
}

bool EscapeBiasLayer::isClearable()
{
  // 这是一个叠加型 layer，不清除单独区域
  return false;
}
