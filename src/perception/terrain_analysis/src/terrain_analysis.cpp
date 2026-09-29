// Copyright 2025 Lihan Chen
// Copyright 2024 Hongbiao Zhu
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
//
// Original work based on sensor_scan_generation package by Hongbiao Zhu.

#include <iomanip>
#include "terrain_analysis/terrain_analysis.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace terrain_analysis
{

TerrainAnalysisNode::TerrainAnalysisNode(const rclcpp::NodeOptions & options)
: Node("terrain_analysis", options)
{
  declare_parameter<std::string>("sensor_frame", sensor_frame_);
  declare_parameter<double>("scan_voxel_size", scan_voxel_size_);
  declare_parameter<double>("decay_time", decay_time_);
  declare_parameter<double>("no_decay_dis", no_decay_dis_);
  declare_parameter<double>("clearing_dis", clearing_dis_);
  declare_parameter<bool>("use_sorting", use_sorting_);
  declare_parameter<double>("quantile_z", quantile_z_);
  declare_parameter<bool>("consider_drop", consider_drop_);
  declare_parameter<bool>("limit_ground_lift", limit_ground_lift_);
  declare_parameter<double>("max_ground_lift", max_ground_lift_);
  declare_parameter<bool>("clear_dy_obs", clear_dy_obs_);
  declare_parameter<double>("min_dy_obs_dis", min_dy_obs_dis_);
  declare_parameter<double>("min_dy_obs_angle", min_dy_obs_angle_);
  declare_parameter<double>("min_dy_obs_rel_z", min_dy_obs_rel_z_);
  declare_parameter<double>("abs_dy_obs_rel_z_thre", abs_dy_obs_rel_z_thre_);
  declare_parameter<double>("min_dy_obs_vfov", min_dy_obs_vfov_);
  declare_parameter<double>("max_dy_obs_vfov", max_dy_obs_vfov_);
  declare_parameter<int>("min_dy_obs_point_num", min_dy_obs_point_num_);
  declare_parameter<bool>("no_data_obstacle", no_data_obstacle_);
  declare_parameter<int>("no_data_block_skip_num", no_data_block_skip_num_);
  declare_parameter<int>("min_block_point_num", min_block_point_num_);
  declare_parameter<double>("vehicle_height", vehicle_height_);
  declare_parameter<double>("max_slope", max_slope_);
  declare_parameter<int>("voxel_point_update_thre", voxel_point_update_thre_);
  declare_parameter<double>("voxel_time_update_thre", voxel_time_update_thre_);
  declare_parameter<double>("min_rel_z", min_rel_z_);
  declare_parameter<double>("max_rel_z", max_rel_z_);
  declare_parameter<double>("dis_ratio_z", dis_ratio_z_);
  declare_parameter<bool>("enable_obstacle_clearing", enable_obstacle_clearing_);
  declare_parameter<double>("obstacle_clearing_dis", obstacle_clearing_dis_);
  declare_parameter<double>("obstacle_height_thre", obstacle_height_thre_);
  declare_parameter<int>("obstacle_confirm_frames", obstacle_confirm_frames_);
  declare_parameter<int>("obstacle_persist_frames", obstacle_persist_frames_);
  declare_parameter<int>("side_blind_obstacle_preserve_frames", 30);  // 侧面盲区检测到障碍物后的保护帧数
  declare_parameter<double>("obstacle_presence_min_dis", 0.08);
  declare_parameter<double>("near_keep_radius", -1.0);
  declare_parameter<double>("near_keep_radius_ratio", 0.4);
  declare_parameter<double>("near_keep_radius_min", 0.6);
  declare_parameter<double>("near_keep_radius_max", 1.5);
  declare_parameter<double>("blind_zone_angle_bin_deg", 2.0);
  declare_parameter<int>("blind_zone_observation_window_bins", 1);
  declare_parameter<bool>("enable_obstacle_sector_filter", true);
  declare_parameter<double>("front_hfov_deg", 160.0);
  declare_parameter<double>("rear_hfov_deg", 160.0);
  declare_parameter<double>("obstacle_blind_radius", 0.6);
  declare_parameter<int>("num_layers", num_layers_);  
  declare_parameter<double>("layer_redundancy", layer_redundancy_);
  declare_parameter<double>("layer_height_offset", layer_height_offset_);
  declare_parameter<bool>("auto_layer_height_offset", auto_layer_height_offset_);
  declare_parameter<int>("terrain_publish_half_width", terrain_publish_half_width_);  


  get_parameter("sensor_frame", sensor_frame_);
  get_parameter("scan_voxel_size", scan_voxel_size_);
  get_parameter("decay_time", decay_time_);
  get_parameter("no_decay_dis", no_decay_dis_);
  get_parameter("clearing_dis", clearing_dis_);
  get_parameter("use_sorting", use_sorting_);
  get_parameter("quantile_z", quantile_z_);
  get_parameter("consider_drop", consider_drop_);
  get_parameter("limit_ground_lift", limit_ground_lift_);
  get_parameter("max_ground_lift", max_ground_lift_);
  get_parameter("clear_dy_obs", clear_dy_obs_);
  get_parameter("min_dy_obs_dis", min_dy_obs_dis_);
  get_parameter("min_dy_obs_angle", min_dy_obs_angle_);
  get_parameter("min_dy_obs_rel_z", min_dy_obs_rel_z_);
  get_parameter("abs_dy_obs_rel_z_thre", abs_dy_obs_rel_z_thre_);
  get_parameter("min_dy_obs_vfov", min_dy_obs_vfov_);
  get_parameter("max_dy_obs_vfov", max_dy_obs_vfov_);
  get_parameter("min_dy_obs_point_num", min_dy_obs_point_num_);
  get_parameter("no_data_obstacle", no_data_obstacle_);
  get_parameter("no_data_block_skip_num", no_data_block_skip_num_);
  get_parameter("min_block_point_num", min_block_point_num_);
  get_parameter("vehicle_height", vehicle_height_);
  get_parameter("max_slope", max_slope_);
  get_parameter("voxel_point_update_thre", voxel_point_update_thre_);
  get_parameter("voxel_time_update_thre", voxel_time_update_thre_);
  get_parameter("min_rel_z", min_rel_z_);
  get_parameter("max_rel_z", max_rel_z_);
  get_parameter("dis_ratio_z", dis_ratio_z_);
  get_parameter("enable_obstacle_clearing", enable_obstacle_clearing_);
  get_parameter("obstacle_clearing_dis", obstacle_clearing_dis_);
  get_parameter("obstacle_height_thre", obstacle_height_thre_);
  get_parameter("obstacle_confirm_frames", obstacle_confirm_frames_);
  get_parameter("obstacle_persist_frames", obstacle_persist_frames_);
  get_parameter("side_blind_obstacle_preserve_frames", side_blind_obstacle_preserve_frames_);
  get_parameter("num_layers", num_layers_);
  get_parameter("layer_redundancy", layer_redundancy_);
  get_parameter("layer_height_offset", layer_height_offset_);
  get_parameter("auto_layer_height_offset", auto_layer_height_offset_);
  get_parameter("terrain_publish_half_width", terrain_publish_half_width_);  

  if(terrain_publish_half_width_ <= 0)
  {
    terrain_publish_half_width_ = terrain_voxel_half_width_;
  }
  else if(terrain_publish_half_width_ > terrain_voxel_half_width_)
  {
    terrain_publish_half_width_ = terrain_voxel_half_width_;
  }

  if (num_layers_ < 1) {
    num_layers_ = 1;
  }
  if (num_layers_ > 3) {
    num_layers_ = 3;
  }
  use_layer_filter_ = (num_layers_ > 1);
  if (!use_layer_filter_) {
    current_layer_index_ = 0;
  }

  if(auto_layer_height_offset_)
  {
    layer_height_offset_inited_ = false;
    layer_height_offset_ = 0.0;
  }

  RCLCPP_WARN(
    get_logger(),
    "[terrain_analysis] startup params: useLayerFilter=%s, numLayers=%d, "
    "layerRedundancy=%.3f, currentLayerIndex=%d, vehicleHeight=%.3f, scanVoxelSize=%.3f, "
    "layerHeightOffset=%.3f, autoLayerHeightOffset=%s, terrainPublishHalfWidth=%d",
    use_layer_filter_ ? "true" : "false",
    static_cast<int>(num_layers_),
    layer_redundancy_,
    static_cast<int>(current_layer_index_),
    vehicle_height_,
    scan_voxel_size_,
    layer_height_offset_,
    auto_layer_height_offset_ ? "true" : "false",
    terrain_publish_half_width_);

  laser_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  laser_cloud_crop_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  laser_cloud_dwz_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  terrain_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  terrain_cloud_elev_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (auto & i : terrain_voxel_cloud_) {
    i = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  }

  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "lidar_odometry", 5,
    std::bind(&TerrainAnalysisNode::odometryHandler, this, std::placeholders::_1));
  laser_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan", 5,
    std::bind(&TerrainAnalysisNode::laserCloudHandler, this, std::placeholders::_1));
  joystick_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 5, std::bind(&TerrainAnalysisNode::joystickHandler, this, std::placeholders::_1));
  clearing_sub_ = create_subscription<example_interfaces::msg::Float32>(
    "map_clearing", 5,
    std::bind(&TerrainAnalysisNode::clearingHandler, this, std::placeholders::_1));
  terrain_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("terrain_map", 2);
  obstacle_clear_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("obstacle_clear_cloud", 2);
  obstacle_clear_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  for (auto & i : terrain_voxel_cloud_) {
    i = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  }

  down_size_filter_.setLeafSize(scan_voxel_size_, scan_voxel_size_, scan_voxel_size_);
}

void TerrainAnalysisNode::odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  odom_ = *odom;
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geo_quat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w))
    .getRPY(roll, pitch, yaw);

  sin_vehicle_roll_ = sin(roll);
  cos_vehicle_roll_ = cos(roll);
  sin_vehicle_pitch_ = sin(pitch);
  cos_vehicle_pitch_ = cos(pitch);
  sin_vehicle_yaw_ = sin(yaw);
  cos_vehicle_yaw_ = cos(yaw);

  if (no_data_inited_ == 0) {
    vehicle_x_rec_ = odom_.pose.pose.position.x;
    vehicle_y_rec_ = odom_.pose.pose.position.y;
    no_data_inited_ = 1;
  }
  if (no_data_inited_ == 1) {
    float dis = sqrt(
      (odom_.pose.pose.position.x - vehicle_x_rec_) *
        (odom_.pose.pose.position.x - vehicle_x_rec_) +
      (odom_.pose.pose.position.y - vehicle_y_rec_) *
        (odom_.pose.pose.position.y - vehicle_y_rec_));
    if (dis >= no_decay_dis_) no_data_inited_ = 2;
  }

  if (auto_layer_height_offset_ && !layer_height_offset_inited_) {
    layer_height_offset_ = odom_.pose.pose.position.z;
    layer_height_offset_inited_ = true;
    RCLCPP_INFO(
      get_logger(), "[terrain_analysis] layerHeightOffset auto init: %.3f",
      layer_height_offset_);
  }
}

void TerrainAnalysisNode::laserCloudHandler(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud)
{
  laser_cloud_time_ = rclcpp::Time(laserCloud->header.stamp).seconds();
  if (!system_inited_) {
    system_init_time_ = laser_cloud_time_;
    system_inited_ = true;
  }

  laser_cloud_->clear();
  pcl::fromROSMsg(*laserCloud, *laser_cloud_);

  pcl::PointXYZI point;
  laser_cloud_crop_->clear();
  int laser_cloud_size = laser_cloud_->points.size();
  for (int i = 0; i < laser_cloud_size; i++) {
    point = laser_cloud_->points[i];

    float point_x = point.x;
    float point_y = point.y;
    float point_z = point.z;

    float dis = sqrt(
      (point_x - odom_.pose.pose.position.x) * (point_x - odom_.pose.pose.position.x) +
      (point_y - odom_.pose.pose.position.y) * (point_y - odom_.pose.pose.position.y));
    if (
      point_z - odom_.pose.pose.position.z > min_rel_z_ - dis_ratio_z_ * dis &&
      point_z - odom_.pose.pose.position.z < max_rel_z_ + dis_ratio_z_ * dis &&
      dis < terrain_voxel_size_ * (terrain_voxel_half_width_ + 1)) {
      point.x = point_x;
      point.y = point_y;
      point.z = point_z;
      point.intensity = laser_cloud_time_ - system_init_time_;
      laser_cloud_crop_->push_back(point);
    }
  }

  processLaserCloud();
}

void TerrainAnalysisNode::joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  if (joy->buttons[5] > 0.5) {
    no_data_inited_ = 0;
    clearing_cloud_ = true;
  }
}

void TerrainAnalysisNode::clearingHandler(
  const example_interfaces::msg::Float32::ConstSharedPtr dis)
{
  no_data_inited_ = 0;
  clearing_dis_ = dis->data;
  clearing_cloud_ = true;
}

void TerrainAnalysisNode::processLaserCloud()
{
  // === START: Layer selection for current frame ===
  if (use_layer_filter_) {
    current_layer_index_ = 0;
    double min_diff = fabs(odom_.pose.pose.position.z - layer_heights_[0]);
    for (int ind = 1; ind < num_layers_; ind++) {
      double z_diff = fabs(odom_.pose.pose.position.z - layer_heights_[ind]);
      if (z_diff < min_diff) {
        min_diff = z_diff;
        current_layer_index_ = ind;
      }
    }

    // RCLCPP_INFO_THROTTLE(
    //   get_logger(), *get_clock(), 1000,
    //   "层过滤开关: on, 当前层: %d (参考高度 %.2fm), 车体高度=%.3f, 层数=%d",
    //   current_layer_index_, layer_heights_[current_layer_index_],
    //   odom_.pose.pose.position.z, num_layers_);
  } else {
    current_layer_index_ = 0;

    // RCLCPP_INFO_THROTTLE(
    //   get_logger(), *get_clock(), 1000,
    //   "层过滤关闭: numLayers=%d, 车体高度=%.3f", num_layers_,
    //   odom_.pose.pose.position.z);
  }
  // === END: Layer selection for current frame ===

  // terrain voxel roll over
  float terrain_voxel_cen_x = terrain_voxel_size_ * terrain_voxel_shift_x_;
  float terrain_voxel_cen_y = terrain_voxel_size_ * terrain_voxel_shift_y_;

  while (odom_.pose.pose.position.x - terrain_voxel_cen_x < -terrain_voxel_size_) {
    for (int ind_y = 0; ind_y < terrain_voxel_width_; ind_y++) {
      auto terrain_voxel_cloud_ptr =
        terrain_voxel_cloud_[terrain_voxel_width_ * (terrain_voxel_width_ - 1) + ind_y];
      for (int ind_x = terrain_voxel_width_ - 1; ind_x >= 1; ind_x--) {
        terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + ind_y] =
          terrain_voxel_cloud_[terrain_voxel_width_ * (ind_x - 1) + ind_y];
      }
      terrain_voxel_cloud_[ind_y] = terrain_voxel_cloud_ptr;
      terrain_voxel_cloud_[ind_y]->clear();
    }
    terrain_voxel_shift_x_--;
    terrain_voxel_cen_x = terrain_voxel_size_ * terrain_voxel_shift_x_;
  }

  while (odom_.pose.pose.position.x - terrain_voxel_cen_x > terrain_voxel_size_) {
    for (int ind_y = 0; ind_y < terrain_voxel_width_; ind_y++) {
      auto terrain_voxel_cloud_ptr = terrain_voxel_cloud_[ind_y];
      for (int ind_x = 0; ind_x < terrain_voxel_width_ - 1; ind_x++) {
        terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + ind_y] =
          terrain_voxel_cloud_[terrain_voxel_width_ * (ind_x + 1) + ind_y];
      }
      terrain_voxel_cloud_[terrain_voxel_width_ * (terrain_voxel_width_ - 1) + ind_y] =
        terrain_voxel_cloud_ptr;
      terrain_voxel_cloud_[terrain_voxel_width_ * (terrain_voxel_width_ - 1) + ind_y]->clear();
    }
    terrain_voxel_shift_x_++;
    terrain_voxel_cen_x = terrain_voxel_size_ * terrain_voxel_shift_x_;
  }

  while (odom_.pose.pose.position.y - terrain_voxel_cen_y < -terrain_voxel_size_) {
    for (int ind_x = 0; ind_x < terrain_voxel_width_; ind_x++) {
      auto terrain_voxel_cloud_ptr =
        terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + (terrain_voxel_width_ - 1)];
      for (int ind_y = terrain_voxel_width_ - 1; ind_y >= 1; ind_y--) {
        terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + ind_y] =
          terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + (ind_y - 1)];
      }
      terrain_voxel_cloud_[terrain_voxel_width_ * ind_x] = terrain_voxel_cloud_ptr;
      terrain_voxel_cloud_[terrain_voxel_width_ * ind_x]->clear();
    }
    terrain_voxel_shift_y_--;
    terrain_voxel_cen_y = terrain_voxel_size_ * terrain_voxel_shift_y_;
  }

  while (odom_.pose.pose.position.y - terrain_voxel_cen_y > terrain_voxel_size_) {
    for (int ind_x = 0; ind_x < terrain_voxel_width_; ind_x++) {
      auto terrain_voxel_cloud_ptr = terrain_voxel_cloud_[terrain_voxel_width_ * ind_x];
      for (int ind_y = 0; ind_y < terrain_voxel_width_ - 1; ind_y++) {
        terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + ind_y] =
          terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + (ind_y + 1)];
      }
      terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + (terrain_voxel_width_ - 1)] =
        terrain_voxel_cloud_ptr;
      terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + (terrain_voxel_width_ - 1)]->clear();
    }
    terrain_voxel_shift_y_++;
    terrain_voxel_cen_y = terrain_voxel_size_ * terrain_voxel_shift_y_;
  }

  // stack registered laser scans
  pcl::PointXYZI point;
  int laser_cloud_crop_size = laser_cloud_crop_->points.size();

  bool enable_obstacle_sector_filter = true;
  double front_hfov_deg_param = 160.0;
  double rear_hfov_deg_param = 160.0;
  double obstacle_blind_radius_param = 0.6;
  get_parameter("enable_obstacle_sector_filter", enable_obstacle_sector_filter);
  get_parameter("front_hfov_deg", front_hfov_deg_param);
  get_parameter("rear_hfov_deg", rear_hfov_deg_param);
  get_parameter("obstacle_blind_radius", obstacle_blind_radius_param);

  if (front_hfov_deg_param < 0.0) {
    front_hfov_deg_param = 0.0;
  }
  if (front_hfov_deg_param > 360.0) {
    front_hfov_deg_param = 360.0;
  }
  if (rear_hfov_deg_param < 0.0) {
    rear_hfov_deg_param = 0.0;
  }
  if (rear_hfov_deg_param > 360.0) {
    rear_hfov_deg_param = 360.0;
  }
  if (obstacle_blind_radius_param < 0.0) {
    obstacle_blind_radius_param = 0.0;
  }

  const float front_hfov_half =
    static_cast<float>(front_hfov_deg_param * static_cast<double>(M_PI) / 360.0);
  const float rear_hfov_half =
    static_cast<float>(rear_hfov_deg_param * static_cast<double>(M_PI) / 360.0);
  const float obstacle_blind_radius = static_cast<float>(obstacle_blind_radius_param);

  const auto is_obstacle_observable =
    [enable_obstacle_sector_filter, front_hfov_half, rear_hfov_half, obstacle_blind_radius](
      float yaw_rad, float dis_xy) {
      if (dis_xy <= obstacle_blind_radius) {
        return false;
      }
      if (!enable_obstacle_sector_filter) {
        return true;
      }
      const float abs_yaw = fabs(yaw_rad);
      const bool in_front = abs_yaw <= front_hfov_half;
      const bool in_rear = fabs(static_cast<float>(M_PI) - abs_yaw) <= rear_hfov_half;
      return in_front || in_rear;
    };

  std::set<int> updated_terrain_voxels;
  for (int i = 0; i < laser_cloud_crop_size; i++) {
    point = laser_cloud_crop_->points[i];

    // START: Terrain voxel insertion with layer filtering
    float point_z_rel = point.z - odom_.pose.pose.position.z;
    if (use_layer_filter_ &&
        fabs(point_z_rel - layer_heights_[current_layer_index_]) > layer_redundancy_) {
      continue;
    }
    // END: Terrain voxel insertion with layer filtering 

    int ind_x =
      static_cast<int>(
        (point.x - odom_.pose.pose.position.x + terrain_voxel_size_ / 2) / terrain_voxel_size_) +
      terrain_voxel_half_width_;
    int ind_y =
      static_cast<int>(
        (point.y - odom_.pose.pose.position.y + terrain_voxel_size_ / 2) / terrain_voxel_size_) +
      terrain_voxel_half_width_;
    if (point.x - odom_.pose.pose.position.x + terrain_voxel_size_ / 2 < 0) ind_x--;
    if (point.y - odom_.pose.pose.position.y + terrain_voxel_size_ / 2 < 0) ind_y--;
    if (ind_x >= 0 && ind_x < terrain_voxel_width_ && ind_y >= 0 && ind_y < terrain_voxel_width_) {
      updated_terrain_voxels.insert(terrain_voxel_width_ * ind_x + ind_y);
    }
  }

  // 1. Clear terrain voxels that were NOT updated this frame (moved BEFORE accumulating new points
  //    to prevent stale dynamic-obstacle points from surviving one extra frame)
  for (int ind = 0; ind < terrain_voxel_num_; ind++) {
    if (updated_terrain_voxels.find(ind) == updated_terrain_voxels.end()) {
      const int ind_x = ind / terrain_voxel_width_;
      const int ind_y = ind % terrain_voxel_width_;
      const float world_x =
        terrain_voxel_size_ * (ind_x - terrain_voxel_half_width_) + odom_.pose.pose.position.x;
      const float world_y =
        terrain_voxel_size_ * (ind_y - terrain_voxel_half_width_) + odom_.pose.pose.position.y;
      const float rel_x = world_x - odom_.pose.pose.position.x;
      const float rel_y = world_y - odom_.pose.pose.position.y;
      const float rel_x_body = rel_x * cos_vehicle_yaw_ + rel_y * sin_vehicle_yaw_;
      const float rel_y_body = -rel_x * sin_vehicle_yaw_ + rel_y * cos_vehicle_yaw_;
      const float rel_yaw = atan2(rel_y_body, rel_x_body);
      const float dis_xy = sqrt(rel_x * rel_x + rel_y * rel_y);
      if (!is_obstacle_observable(rel_yaw, dis_xy)) {
        continue;
      }
      terrain_voxel_cloud_[ind]->clear();
    }
  }

   
  // 2. Clear terrain voxels that were NOT updated this frame to prevent
  //    stale dynamic-obstacle points from flickering in and out
  for (int i = 0; i < laser_cloud_crop_size; i++) {
    point = laser_cloud_crop_->points[i];

    int ind_x =
      static_cast<int>(
        (point.x - odom_.pose.pose.position.x + terrain_voxel_size_ / 2) / terrain_voxel_size_) +
      terrain_voxel_half_width_;
    int ind_y =
      static_cast<int>(
        (point.y - odom_.pose.pose.position.y + terrain_voxel_size_ / 2) / terrain_voxel_size_) +
      terrain_voxel_half_width_;

    if (point.x - odom_.pose.pose.position.x + terrain_voxel_size_ / 2 < 0) ind_x--;
    if (point.y - odom_.pose.pose.position.y + terrain_voxel_size_ / 2 < 0) ind_y--;

    if (ind_x >= 0 && ind_x < terrain_voxel_width_ && ind_y >= 0 && ind_y < terrain_voxel_width_) {
      terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + ind_y]->push_back(point);
      terrain_voxel_update_num_[terrain_voxel_width_ * ind_x + ind_y]++;
    }
  }

  for (int ind = 0; ind < terrain_voxel_num_; ind++) {
    if (
      terrain_voxel_update_num_[ind] >= voxel_point_update_thre_ ||
      laser_cloud_time_ - system_init_time_ - terrain_voxel_update_time_[ind] >=
        voxel_time_update_thre_ ||
      clearing_cloud_) {
      auto terrain_voxel_cloud_ptr = terrain_voxel_cloud_[ind];

      laser_cloud_dwz_->clear();
      down_size_filter_.setInputCloud(terrain_voxel_cloud_ptr);
      down_size_filter_.filter(*laser_cloud_dwz_);

      terrain_voxel_cloud_ptr->clear();
      int laser_cloud_dwz_size = laser_cloud_dwz_->points.size();
      for (int i = 0; i < laser_cloud_dwz_size; i++) {
        point = laser_cloud_dwz_->points[i];
        float dis = sqrt(
          (point.x - odom_.pose.pose.position.x) * (point.x - odom_.pose.pose.position.x) +
          (point.y - odom_.pose.pose.position.y) * (point.y - odom_.pose.pose.position.y));
        if (
          point.z - odom_.pose.pose.position.z > min_rel_z_ - dis_ratio_z_ * dis &&
          point.z - odom_.pose.pose.position.z < max_rel_z_ + dis_ratio_z_ * dis &&
          (laser_cloud_time_ - system_init_time_ - point.intensity < decay_time_ ||
           dis < no_decay_dis_) &&
          (dis >= clearing_dis_ || !clearing_cloud_)) {
          terrain_voxel_cloud_ptr->push_back(point);
        }
      }

      terrain_voxel_update_num_[ind] = 0;
      terrain_voxel_update_time_[ind] = laser_cloud_time_ - system_init_time_;
    }
  }

  terrain_cloud_->clear();
  // for (int ind_x = terrain_voxel_half_width_ - 5; ind_x <= terrain_voxel_half_width_ + 5; ind_x++) {
  //   for (int ind_y = terrain_voxel_half_width_ - 5; ind_y <= terrain_voxel_half_width_ + 5;
  //        ind_y++) {
  //     *terrain_cloud_ += *terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + ind_y];
  //   }
  // }
  const int terrain_publish_x_min = terrain_voxel_half_width_ - terrain_publish_half_width_;
  const int terrain_publish_x_max = terrain_voxel_half_width_ + terrain_publish_half_width_;
  const int terrain_publish_y_min = terrain_voxel_half_width_ - terrain_publish_half_width_;
  const int terrain_publish_y_max = terrain_voxel_half_width_ + terrain_publish_half_width_;
  for(int ind_x = terrain_publish_x_min; ind_x <= terrain_publish_x_max; ind_x++){
    for(int ind_y = terrain_publish_y_min; ind_y <= terrain_publish_y_max; ind_y++){
      *terrain_cloud_ += *terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + ind_y];
    }
  }

  // 3. Only publish from updated terrain voxels
  for (int ind : updated_terrain_voxels) {
    *terrain_cloud_ += *terrain_voxel_cloud_[ind];
  }

  // estimate ground and compute elevation for each point
  for (int i = 0; i < planar_voxel_num_; i++) {
    planar_voxel_elev_[i] = 0;
    planar_voxel_edge_[i] = 0;
    planar_voxel_dy_obs_[i] = 0;
    planar_point_elev_[i].clear();
  }

  int terrain_cloud_size = terrain_cloud_->points.size();
  for (int i = 0; i < terrain_cloud_size; i++) {
    point = terrain_cloud_->points[i];

    // START: Terrain voxel insertion with layer filtering
    float point_z_rel = point.z - odom_.pose.pose.position.z;
    if (use_layer_filter_ &&
        fabs(point_z_rel - layer_heights_[current_layer_index_]) > layer_redundancy_) {
      continue;
    }
    // END: Terrain voxel insertion with layer filtering 

    int ind_x =
      static_cast<int>(
        (point.x - odom_.pose.pose.position.x + planar_voxel_size_ / 2) / planar_voxel_size_) +
      planar_voxel_half_width_;
    int ind_y =
      static_cast<int>(
        (point.y - odom_.pose.pose.position.y + planar_voxel_size_ / 2) / planar_voxel_size_) +
      planar_voxel_half_width_;

    if (point.x - odom_.pose.pose.position.x + planar_voxel_size_ / 2 < 0) ind_x--;
    if (point.y - odom_.pose.pose.position.y + planar_voxel_size_ / 2 < 0) ind_y--;

    if (
      point.z - odom_.pose.pose.position.z > min_rel_z_ &&
      point.z - odom_.pose.pose.position.z < max_rel_z_) {
      for (int d_x = -1; d_x <= 1; d_x++) {
        for (int d_y = -1; d_y <= 1; d_y++) {
          if (
            ind_x + d_x >= 0 && ind_x + d_x < planar_voxel_width_ && ind_y + d_y >= 0 &&
            ind_y + d_y < planar_voxel_width_) {
            planar_point_elev_[planar_voxel_width_ * (ind_x + d_x) + ind_y + d_y].push_back(
              point.z);
          }
        }
      }
    }

    if (clear_dy_obs_) {
      if (ind_x >= 0 && ind_x < planar_voxel_width_ && ind_y >= 0 && ind_y < planar_voxel_width_) {
        float point_x1 = point.x - odom_.pose.pose.position.x;
        float point_y1 = point.y - odom_.pose.pose.position.y;
        float point_z1 = point.z - odom_.pose.pose.position.z;

        float dis1 = sqrt(point_x1 * point_x1 + point_y1 * point_y1);
        if (dis1 > min_dy_obs_dis_) {
          float angle1 = atan2(point_z1 - min_dy_obs_rel_z_, dis1) * 180.0 / M_PI;
          if (angle1 > min_dy_obs_angle_) {
            float point_x2 = point_x1 * cos_vehicle_yaw_ + point_y1 * sin_vehicle_yaw_;
            float point_y2 = -point_x1 * sin_vehicle_yaw_ + point_y1 * cos_vehicle_yaw_;
            float point_z2 = point_z1;

            float point_x3 = point_x2 * cos_vehicle_pitch_ - point_z2 * sin_vehicle_pitch_;
            float point_y3 = point_y2;
            float point_z3 = point_x2 * sin_vehicle_pitch_ + point_z2 * cos_vehicle_pitch_;

            float point_x4 = point_x3;
            float point_y4 = point_y3 * cos_vehicle_roll_ + point_z3 * sin_vehicle_roll_;
            float point_z4 = -point_y3 * sin_vehicle_roll_ + point_z3 * cos_vehicle_roll_;

            float dis4 = sqrt(point_x4 * point_x4 + point_y4 * point_y4);
            float angle4 = atan2(point_z4, dis4) * 180.0 / M_PI;
            if (
              (angle4 > min_dy_obs_vfov_ && angle4 < max_dy_obs_vfov_) ||
              fabs(point_z4) < abs_dy_obs_rel_z_thre_) {
              planar_voxel_dy_obs_[planar_voxel_width_ * ind_x + ind_y]++;
            }
          }
        } else {
          planar_voxel_dy_obs_[planar_voxel_width_ * ind_x + ind_y] += min_dy_obs_point_num_;
        }
      }
    }
  }

  if (clear_dy_obs_) {
    for (int i = 0; i < laser_cloud_crop_size; i++) {
      point = laser_cloud_crop_->points[i];

      int ind_x =
        static_cast<int>(
          (point.x - odom_.pose.pose.position.x + planar_voxel_size_ / 2) / planar_voxel_size_) +
        planar_voxel_half_width_;
      int ind_y =
        static_cast<int>(
          (point.y - odom_.pose.pose.position.y + planar_voxel_size_ / 2) / planar_voxel_size_) +
        planar_voxel_half_width_;

      if (point.x - odom_.pose.pose.position.x + planar_voxel_size_ / 2 < 0) ind_x--;
      if (point.y - odom_.pose.pose.position.y + planar_voxel_size_ / 2 < 0) ind_y--;

      if (ind_x >= 0 && ind_x < planar_voxel_width_ && ind_y >= 0 && ind_y < planar_voxel_width_) {
        float point_x1 = point.x - odom_.pose.pose.position.x;
        float point_y1 = point.y - odom_.pose.pose.position.y;
        float point_z1 = point.z - odom_.pose.pose.position.z;

        float dis1 = sqrt(point_x1 * point_x1 + point_y1 * point_y1);
        float angle1 = atan2(point_z1 - min_dy_obs_rel_z_, dis1) * 180.0 / M_PI;
        if (angle1 > min_dy_obs_angle_) {
          planar_voxel_dy_obs_[planar_voxel_width_ * ind_x + ind_y] = 0;
        }
      }
    }
  }

  if (use_sorting_) {
    for (int i = 0; i < planar_voxel_num_; i++) {
      int planar_point_elev_size = planar_point_elev_[i].size();
      if (planar_point_elev_size > 0) {
        std::sort(planar_point_elev_[i].begin(), planar_point_elev_[i].end());

        int quantile_id = static_cast<int>(quantile_z_ * planar_point_elev_size);
        if (quantile_id < 0)
          quantile_id = 0;
        else if (quantile_id >= planar_point_elev_size)
          quantile_id = planar_point_elev_size - 1;

        if (
          planar_point_elev_[i][quantile_id] > planar_point_elev_[i][0] + max_ground_lift_ &&
          limit_ground_lift_) {
          planar_voxel_elev_[i] = planar_point_elev_[i][0] + max_ground_lift_;
        } else {
          planar_voxel_elev_[i] = planar_point_elev_[i][quantile_id];
        }
      }
    }
  } else {
    for (int i = 0; i < planar_voxel_num_; i++) {
      int planar_point_elev_size = planar_point_elev_[i].size();
      if (planar_point_elev_size > 0) {
        float min_z = 1000.0;
        int min_id = -1;
        for (int j = 0; j < planar_point_elev_size; j++) {
          if (planar_point_elev_[i][j] < min_z) {
            min_z = planar_point_elev_[i][j];
            min_id = j;
          }
        }

        if (min_id != -1) {
          planar_voxel_elev_[i] = planar_point_elev_[i][min_id];
        }
      }
    }
  }

  terrain_cloud_elev_->clear();
  int terrain_cloud_elev_size = 0;
  for (int i = 0; i < terrain_cloud_size; i++) {
    point = terrain_cloud_->points[i];
    if (
      point.z - odom_.pose.pose.position.z > min_rel_z_ &&
      point.z - odom_.pose.pose.position.z < max_rel_z_) {
      int ind_x =
        static_cast<int>(
          (point.x - odom_.pose.pose.position.x + planar_voxel_size_ / 2) / planar_voxel_size_) +
        planar_voxel_half_width_;
      int ind_y =
        static_cast<int>(
          (point.y - odom_.pose.pose.position.y + planar_voxel_size_ / 2) / planar_voxel_size_) +
        planar_voxel_half_width_;

      if (point.x - odom_.pose.pose.position.x + planar_voxel_size_ / 2 < 0) ind_x--;
      if (point.y - odom_.pose.pose.position.y + planar_voxel_size_ / 2 < 0) ind_y--;

      if (ind_x >= 0 && ind_x < planar_voxel_width_ && ind_y >= 0 && ind_y < planar_voxel_width_) {
        if (
          planar_voxel_dy_obs_[planar_voxel_width_ * ind_x + ind_y] < min_dy_obs_point_num_ ||
          !clear_dy_obs_) {
          float dis_z = point.z - planar_voxel_elev_[planar_voxel_width_ * ind_x + ind_y];
          if (consider_drop_) dis_z = fabs(dis_z);
          int planar_point_elev_size =
            planar_point_elev_[planar_voxel_width_ * ind_x + ind_y].size();
          if (
            dis_z >= 0 && dis_z < vehicle_height_ &&
            planar_point_elev_size >= min_block_point_num_) {
            // terrain_cloud_elev_->push_back(point);
            // terrain_cloud_elev_->points[terrain_cloud_elev_size].intensity = dis_z;

            // terrain_cloud_elev_size++;
            
            // Slope tolerance: if the slope angle to all neighbors is small enough,
            // treat it as a gentle slope (e.g. ramp) rather than an obstacle.
            bool is_slope = false;
            if (max_slope_ > 0) {
              bool all_neighbors_below_slope = true;
              for (int d_x = -1; d_x <= 1; d_x++) {
                for (int d_y = -1; d_y <= 1; d_y++) {
                  if (d_x == 0 && d_y == 0) continue;
                  int n_x = ind_x + d_x;
                  int n_y = ind_y + d_y;
                  if (
                    n_x >= 0 && n_x < planar_voxel_width_ && n_y >= 0 &&
                    n_y < planar_voxel_width_) {
                    float neighbor_elev =
                      planar_voxel_elev_[planar_voxel_width_ * n_x + n_y];
                    float dz = point.z - neighbor_elev;
                    float dist = planar_voxel_size_ * sqrt(static_cast<float>(d_x * d_x + d_y * d_y));
                    float slope_angle = atan2(fabs(dz), dist);
                    if (slope_angle >= max_slope_) {
                      all_neighbors_below_slope = false;
                      break;
                    }
                  }
                }
                if (!all_neighbors_below_slope) break;
              }
              is_slope = all_neighbors_below_slope;
            }

            if (!is_slope) {
              terrain_cloud_elev_->push_back(point);
              terrain_cloud_elev_->points[terrain_cloud_elev_size].intensity = dis_z;
              terrain_cloud_elev_size++;
            }
          }
        }
      }
    }
  }

  if (no_data_obstacle_ && no_data_inited_ == 2) {
    for (int i = 0; i < planar_voxel_num_; i++) {
      int planar_point_elev_size = planar_point_elev_[i].size();
      if (planar_point_elev_size < min_block_point_num_) {
        planar_voxel_edge_[i] = 1;
      }
    }

    for (int no_data_block_skip_count = 0; no_data_block_skip_count < no_data_block_skip_num_;
         no_data_block_skip_count++) {
      for (int i = 0; i < planar_voxel_num_; i++) {
        if (planar_voxel_edge_[i] >= 1) {
          int ind_x = static_cast<int>(i / planar_voxel_width_);
          int ind_y = i % planar_voxel_width_;
          bool edge_voxel = false;
          for (int d_x = -1; d_x <= 1; d_x++) {
            for (int d_y = -1; d_y <= 1; d_y++) {
              if (
                ind_x + d_x >= 0 && ind_x + d_x < planar_voxel_width_ && ind_y + d_y >= 0 &&
                ind_y + d_y < planar_voxel_width_) {
                if (
                  planar_voxel_edge_[planar_voxel_width_ * (ind_x + d_x) + ind_y + d_y] <
                  planar_voxel_edge_[i]) {
                  edge_voxel = true;
                }
              }
            }
          }

          if (!edge_voxel) planar_voxel_edge_[i]++;
        }
      }
    }

    for (int i = 0; i < planar_voxel_num_; i++) {
      if (planar_voxel_edge_[i] > no_data_block_skip_num_) {
        int ind_x = static_cast<int>(i / planar_voxel_width_);
        int ind_y = i % planar_voxel_width_;

        point.x =
          planar_voxel_size_ * (ind_x - planar_voxel_half_width_) + odom_.pose.pose.position.x;
        point.y =
          planar_voxel_size_ * (ind_y - planar_voxel_half_width_) + odom_.pose.pose.position.y;
        point.z = odom_.pose.pose.position.z;
        point.intensity = vehicle_height_;

        point.x -= planar_voxel_size_ / 4.0;
        point.y -= planar_voxel_size_ / 4.0;
        terrain_cloud_elev_->push_back(point);

        point.x += planar_voxel_size_ / 2.0;
        terrain_cloud_elev_->push_back(point);

        point.y += planar_voxel_size_ / 2.0;
        terrain_cloud_elev_->push_back(point);

        point.x -= planar_voxel_size_ / 2.0;
        terrain_cloud_elev_->push_back(point);
      }
    }
  }

  // ========== OBSTACLE CLEARING LOGIC ==========
  // 低矮窄台阶近场保护参数（可通过参数文件调参）
  double obstacle_presence_min_dis_param = 0.08;
  double near_keep_radius_param = -1.0;
  double near_keep_radius_ratio_param = 0.4;
  double near_keep_radius_min_param = 0.6;
  double near_keep_radius_max_param = 1.5;
  double blind_zone_angle_bin_deg_param = 2.0;
  int blind_zone_observation_window_bins = 1;

  get_parameter("obstacle_presence_min_dis", obstacle_presence_min_dis_param);
  get_parameter("near_keep_radius", near_keep_radius_param);
  get_parameter("near_keep_radius_ratio", near_keep_radius_ratio_param);
  get_parameter("near_keep_radius_min", near_keep_radius_min_param);
  get_parameter("near_keep_radius_max", near_keep_radius_max_param);
  get_parameter("blind_zone_angle_bin_deg", blind_zone_angle_bin_deg_param);
  get_parameter("blind_zone_observation_window_bins", blind_zone_observation_window_bins);

  if (obstacle_presence_min_dis_param < 0.0) {
    obstacle_presence_min_dis_param = 0.0;
  }
  if (near_keep_radius_ratio_param < 0.0) {
    near_keep_radius_ratio_param = 0.0;
  }
  if (near_keep_radius_min_param < 0.0) {
    near_keep_radius_min_param = 0.0;
  }
  if (near_keep_radius_max_param < near_keep_radius_min_param) {
    near_keep_radius_max_param = near_keep_radius_min_param;
  }
  if (blind_zone_angle_bin_deg_param < 0.5) {
    blind_zone_angle_bin_deg_param = 0.5;
  }
  if (blind_zone_angle_bin_deg_param > 30.0) {
    blind_zone_angle_bin_deg_param = 30.0;
  }
  if (blind_zone_observation_window_bins < 0) {
    blind_zone_observation_window_bins = 0;
  }

  const float obstacle_presence_min_dis = static_cast<float>(obstacle_presence_min_dis_param);
  float near_keep_radius = 0.0f;
  if (near_keep_radius_param > 0.0) {
    near_keep_radius = static_cast<float>(near_keep_radius_param);
  } else {
    near_keep_radius = static_cast<float>(obstacle_clearing_dis_ * near_keep_radius_ratio_param);
  }
  if (near_keep_radius < static_cast<float>(near_keep_radius_min_param)) {
    near_keep_radius = static_cast<float>(near_keep_radius_min_param);
  }
  if (near_keep_radius > static_cast<float>(near_keep_radius_max_param)) {
    near_keep_radius = static_cast<float>(near_keep_radius_max_param);
  }

  // Build per-frame angular observability mask.
  // Only clear cells in directions that are actually observed this frame.
  // This avoids clearing static obstacles that fall into side blind zones.
  const float angle_bin_size =
    static_cast<float>(blind_zone_angle_bin_deg_param * static_cast<double>(M_PI) / 180.0);
  int angle_bin_count = static_cast<int>((2.0f * static_cast<float>(M_PI)) / angle_bin_size);
  if (angle_bin_count < 1) {
    angle_bin_count = 1;
  }
  if (blind_zone_observation_window_bins > angle_bin_count / 2) {
    blind_zone_observation_window_bins = angle_bin_count / 2;
  }
  std::vector<unsigned char> angle_observed(angle_bin_count, 0);

  const float odom_x = odom_.pose.pose.position.x;
  const float odom_y = odom_.pose.pose.position.y;
  const float odom_z = odom_.pose.pose.position.z;

  // Track which planar voxels have obstacles this frame
  // Directly check from laser_cloud_crop_ instead of terrain_cloud_elev_
  // This ensures obstacles can be cleared even when vehicle is stationary
  std::set<int> current_obstacle_voxels;
  
  // Scan laser_cloud_crop_ to find current obstacles
  laser_cloud_crop_size = laser_cloud_crop_->points.size();
  for (int i = 0; i < laser_cloud_crop_size; i++) {
    const auto& p = laser_cloud_crop_->points[i];
    
    // Calculate planar voxel index for this point
    int ind_x = static_cast<int>(
      (p.x - odom_.pose.pose.position.x + planar_voxel_size_ / 2) / planar_voxel_size_) +
      planar_voxel_half_width_;
    int ind_y = static_cast<int>(
      (p.y - odom_.pose.pose.position.y + planar_voxel_size_ / 2) / planar_voxel_size_) +
      planar_voxel_half_width_;
    
    if (p.x - odom_.pose.pose.position.x + planar_voxel_size_ / 2 < 0) ind_x--;
    if (p.y - odom_.pose.pose.position.y + planar_voxel_size_ / 2 < 0) ind_y--;
    
    if (ind_x >= 0 && ind_x < planar_voxel_width_ && ind_y >= 0 && ind_y < planar_voxel_width_) {
      int ind = planar_voxel_width_ * ind_x + ind_y;
      
      // Calculate height relative to estimated ground
      float rel_z = p.z - planar_voxel_elev_[ind];
      float dis_xy = sqrt(
        (p.x - odom_x) * (p.x - odom_x) +
        (p.y - odom_y) * (p.y - odom_y));
      const float rel_x = p.x - odom_x;
      const float rel_y = p.y - odom_y;
      const float rel_x_body = rel_x * cos_vehicle_yaw_ + rel_y * sin_vehicle_yaw_;
      const float rel_y_body = -rel_x * sin_vehicle_yaw_ + rel_y * cos_vehicle_yaw_;
      const float rel_yaw = atan2(rel_y_body, rel_x_body);

      // Calculate bin index first (needed for both observable and blind zone tracking)
      int obs_bin_index = static_cast<int>((rel_yaw + static_cast<float>(M_PI)) / angle_bin_size);
      if (obs_bin_index < 0) {
        obs_bin_index = 0;
      } else if (obs_bin_index >= angle_bin_count) {
        obs_bin_index = angle_bin_count - 1;
      }

      // Calculate absolute yaw for blind zone check
      // const float abs_yaw = fabs(rel_yaw);

      // Mark angle bin as having point data (even if in blind zone, for tracking purposes)
      if (obs_bin_index < MAX_ANGLE_BINS) {
        angle_obstacle_detected_[obs_bin_index] = 1;
        angle_obstacle_frames_since_[obs_bin_index] = 0;
      }

      // Now check if point is observable (within sensor FOV)
      if (!is_obstacle_observable(rel_yaw, dis_xy)) {
        continue;
      }

      if (p.z - odom_z > min_rel_z_ &&
          p.z - odom_z < max_rel_z_ &&
          dis_xy > obstacle_presence_min_dis)
      {
        int bin_index = static_cast<int>((rel_yaw + static_cast<float>(M_PI)) / angle_bin_size);
        if (bin_index < 0) {
          bin_index = 0;
        } else if (bin_index >= angle_bin_count) {
          bin_index = angle_bin_count - 1;
        }
        angle_observed[bin_index] = 1;
      }
      
      // Only consider as obstacle if:
      // 1. Point is within valid Z range
      // 2. Point is above ground by more than obstacle_height_thre_
      // 3. Point is not too close to vehicle (avoid self-detection)
      if (rel_z > obstacle_height_thre_ && 
          p.z - odom_z > min_rel_z_ &&
          p.z - odom_z < max_rel_z_ &&
          dis_xy > obstacle_presence_min_dis) {
        
        current_obstacle_voxels.insert(ind);
        
        // Store world position for clearing point generation
        if (obstacle_cell_state_[ind] == 0) {
          obstacle_cell_state_[ind] = 1;
          obstacle_cell_world_x_[ind] = p.x;
          obstacle_cell_world_y_[ind] = p.y;
          obstacle_cell_world_z_[ind] = p.z;
        }
        // Reset clear counter since obstacle is present
        obstacle_clear_counter_[ind] = 0;
        // Note: angle_obstacle_detected_ is already marked earlier for blind zone tracking
      }
    }
  }
  
  // ========== PROCESS OBSTACLE CLEARING ==========
  obstacle_clear_cloud_->clear();

  if (enable_obstacle_clearing_) {
    // First, update frame counters for all angle bins
    for (int bin_i = 0; bin_i < angle_bin_count && bin_i < MAX_ANGLE_BINS; bin_i++) {
      if (angle_obstacle_detected_[bin_i]) {
        angle_obstacle_frames_since_[bin_i]++;
      }
    }

    for (int ind = 0; ind < planar_voxel_num_; ind++) {
      // Calculate world position of this voxel
      int ind_x = ind / planar_voxel_width_;
      int ind_y = ind % planar_voxel_width_;
      float world_x = planar_voxel_size_ * (ind_x - planar_voxel_half_width_) + odom_.pose.pose.position.x;
      float world_y = planar_voxel_size_ * (ind_y - planar_voxel_half_width_) + odom_.pose.pose.position.y;
      float dis_from_vehicle = sqrt(
        (world_x - odom_x) * (world_x - odom_x) +
        (world_y - odom_y) * (world_y - odom_y));

      // Only process cells within clearing distance
      if (dis_from_vehicle > obstacle_clearing_dis_) {
        continue;
      }

      // If this direction has no observation this frame, we are likely in blind zone.
      // Do not clear obstacle state in unobserved directions.
      {
        const float rel_x = world_x - odom_x;
        const float rel_y = world_y - odom_y;
        const float rel_x_body = rel_x * cos_vehicle_yaw_ + rel_y * sin_vehicle_yaw_;
        const float rel_y_body = -rel_x * sin_vehicle_yaw_ + rel_y * cos_vehicle_yaw_;
        const float rel_yaw = atan2(rel_y_body, rel_x_body);

        if (!is_obstacle_observable(rel_yaw, dis_from_vehicle)) {
          // Check side blind zone protection even when not directly observable
          int bin_index_obs = static_cast<int>((rel_yaw + static_cast<float>(M_PI)) / angle_bin_size);
          if (bin_index_obs < 0) {
            bin_index_obs = 0;
          } else if (bin_index_obs >= angle_bin_count) {
            bin_index_obs = angle_bin_count - 1;
          }
          
          const float abs_yaw = fabs(rel_yaw);
          const bool in_front_or_rear = (abs_yaw <= front_hfov_half) ||
                                        (fabs(static_cast<float>(M_PI) - abs_yaw) <= rear_hfov_half);
          const bool in_side_blind_zone = !in_front_or_rear;
          
          if (in_side_blind_zone && bin_index_obs < MAX_ANGLE_BINS &&
              angle_obstacle_detected_[bin_index_obs] &&
              angle_obstacle_frames_since_[bin_index_obs] < side_blind_obstacle_preserve_frames_) {
            // Side blind zone with recent obstacle detection, don't clear
            obstacle_clear_counter_[ind] = 0;
            if (obstacle_cell_state_[ind] == 2) {
              obstacle_cell_state_[ind] = 1;
            }
            continue;
          }
          
          obstacle_clear_counter_[ind] = 0;
          if (obstacle_cell_state_[ind] == 2) {
            obstacle_cell_state_[ind] = 1;
          }
          continue;
        }

        int bin_index = static_cast<int>((rel_yaw + static_cast<float>(M_PI)) / angle_bin_size);
        if (bin_index < 0) {
          bin_index = 0;
        } else if (bin_index >= angle_bin_count) {
          bin_index = angle_bin_count - 1;
        }

        bool has_observation = false;

        // Check if this direction is in the side blind zone (not directly front or rear)
        const float abs_yaw = fabs(rel_yaw);
        const float front_half_rad = front_hfov_half;
        const float rear_half_rad = rear_hfov_half;
        const bool in_front_or_rear = (abs_yaw <= front_half_rad) ||
                                      (fabs(static_cast<float>(M_PI) - abs_yaw) <= rear_half_rad);
        const bool in_side_blind_zone = !in_front_or_rear;

        // First check: if no observation in this direction, don't clear
        for (int d = -blind_zone_observation_window_bins;
             d <= blind_zone_observation_window_bins; d++) {
          int check_idx = bin_index + d;
          if (check_idx < 0) {
            check_idx += angle_bin_count;
          } else if (check_idx >= angle_bin_count) {
            check_idx -= angle_bin_count;
          }
          if (angle_observed[check_idx] != 0) {
            has_observation = true;
            break;
          }
        }

        if (!has_observation) {
          // No observation in this direction, check side blind zone protection
          if (in_side_blind_zone && bin_index < MAX_ANGLE_BINS &&
              angle_obstacle_detected_[bin_index] &&
              angle_obstacle_frames_since_[bin_index] < side_blind_obstacle_preserve_frames_) {
            // Side blind zone with recent obstacle detection, don't clear
            obstacle_clear_counter_[ind] = 0;
            if (obstacle_cell_state_[ind] == 2) {
              obstacle_cell_state_[ind] = 1;
            }
            continue;
          }
          obstacle_clear_counter_[ind] = 0;
          if (obstacle_cell_state_[ind] == 2) {
            obstacle_cell_state_[ind] = 1;
          }
          continue;
        }

        // Second check: if in side blind zone with current observation, check for side blind protection
        if (in_side_blind_zone) {
          // Check if this obstacle just moved into the blind zone
          bool has_recent_obstacle_detection = false;
          for (int d = -blind_zone_observation_window_bins;
               d <= blind_zone_observation_window_bins; d++) {
            int check_idx = bin_index + d;
            if (check_idx < 0) {
              check_idx += angle_bin_count;
            } else if (check_idx >= angle_bin_count) {
              check_idx -= angle_bin_count;
            }
            if (check_idx < MAX_ANGLE_BINS &&
                angle_obstacle_detected_[check_idx] &&
                angle_obstacle_frames_since_[check_idx] < side_blind_obstacle_preserve_frames_) {
              has_recent_obstacle_detection = true;
              break;
            }
          }
          if (has_recent_obstacle_detection) {
            // Obstacle was recently in this blind zone, don't clear yet
            obstacle_clear_counter_[ind] = 0;
            if (obstacle_cell_state_[ind] == 2) {
              obstacle_cell_state_[ind] = 1;
            }
            continue;
          }
        }
      }

      // Near-field protection: keep low/narrow steps from being cleared too aggressively.
      if (dis_from_vehicle <= near_keep_radius) {
        obstacle_clear_counter_[ind] = 0;
        if (obstacle_cell_state_[ind] == 2) {
          obstacle_cell_state_[ind] = 1;
        }
        continue;
      }

      if (obstacle_cell_state_[ind] == 1 || obstacle_cell_state_[ind] == 2) {
        // This cell was previously marked as obstacle
        if (current_obstacle_voxels.find(ind) == current_obstacle_voxels.end()) {
          // No obstacle detected in this cell this frame
          obstacle_clear_counter_[ind]++;

          if (obstacle_cell_state_[ind] == 1 && obstacle_clear_counter_[ind] >= obstacle_confirm_frames_) {
            // Transition to clearing state
            obstacle_cell_state_[ind] = 2;
            obstacle_clear_counter_[ind] = 0;
          } else if (obstacle_cell_state_[ind] == 2) {
            // Already in clearing state, check persistence
            if (obstacle_clear_counter_[ind] >= obstacle_persist_frames_) {
              // Publish clearing point at ground level
              pcl::PointXYZI clear_point;
              clear_point.x = world_x;
              clear_point.y = world_y;
              clear_point.z = (planar_voxel_elev_[ind] > 0)
                              ? planar_voxel_elev_[ind]
                              : odom_.pose.pose.position.z;
              clear_point.intensity = 0.0;
              obstacle_clear_cloud_->push_back(clear_point);

              // Reset state
              obstacle_cell_state_[ind] = 0;
              obstacle_clear_counter_[ind] = 0;
            }
          }
        } else {
          // Obstacle still present, reset clear counter
          obstacle_clear_counter_[ind] = 0;
          if (obstacle_cell_state_[ind] == 2) {
            // Was clearing but obstacle reappeared, revert to obstacle state
            obstacle_cell_state_[ind] = 1;
          }
        }
      }
    }
  }
  // ========== END OBSTACLE CLEARING ==========

  clearing_cloud_ = false;

  // Publish points with elevation
  sensor_msgs::msg::PointCloud2 terrain_cloud;
  pcl::toROSMsg(*terrain_cloud_elev_, terrain_cloud);
  terrain_cloud.header.stamp = rclcpp::Time(static_cast<uint64_t>(laser_cloud_time_ * 1e9));
  terrain_cloud.header.frame_id = "odom";

  // Transform point cloud to lidar frame
  sensor_msgs::msg::PointCloud2 terrain_cloud_lidar;
  tf2::Transform tf_odom_to_lidar;
  tf2::fromMsg(odom_.pose.pose, tf_odom_to_lidar);
  pcl_ros::transformPointCloud(
    sensor_frame_, tf_odom_to_lidar.inverse(), terrain_cloud, terrain_cloud_lidar);

  // Publish the transformed point cloud
  terrain_map_pub_->publish(terrain_cloud_lidar);
  if (enable_obstacle_clearing_ && !obstacle_clear_cloud_->points.empty())
  {
    sensor_msgs::msg::PointCloud2 clear_cloud_msg;
    pcl::toROSMsg(*obstacle_clear_cloud_, clear_cloud_msg);
    clear_cloud_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(laser_cloud_time_ * 1e9));
    clear_cloud_msg.header.frame_id = "odom";

    sensor_msgs::msg::PointCloud2 clear_cloud_lidar;
    pcl_ros::transformPointCloud(
      sensor_frame_, tf_odom_to_lidar.inverse(), clear_cloud_msg, clear_cloud_lidar);

    obstacle_clear_pub_->publish(clear_cloud_lidar);
  }
}
}  // namespace terrain_analysis

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(terrain_analysis::TerrainAnalysisNode)