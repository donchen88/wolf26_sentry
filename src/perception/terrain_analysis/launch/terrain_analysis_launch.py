# Copyright 2025 Lihan Chen
# Copyright 2024 Hongbiao Zhu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Original work based on sensor_scan_generation package by Hongbiao Zhu.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    start_terrain_analysis_cmd = Node(
        package="terrain_analysis",
        executable="terrain_analysis_node",
        name="terrain_analysis",
        output="screen",
        namespace="",
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        parameters=[
            {"sensor_frame": "front_mid360"},
            {"scan_voxel_size": 0.05},
            {"decay_time": 1.0},
            {"no_decay_dis": 1.75},
            {"clearing_dis": 8.0},
            {"use_sorting": True},
            {"quantile_z": 0.5},
            {"consider_drop": False},
            {"limit_ground_lift": False},
            {"max_ground_lift": 0.15},
            {"clear_dy_obs": True},
            {"min_dy_obs_dis": 0.3},
            {"min_dy_obs_angle": 0.0},
            {"min_dy_obs_rel_z": -0.3},
            {"abs_dy_obs_rel_z_thre": 0.2},
            {"min_dy_obs_vfov": -28.0},
            {"max_dy_obs_vfov": 33.0},
            {"min_dy_obs_point_num": 1},
            {"num_layers": 1},
            {"layer_redundancy": 0.025},
            {"auto_layer_height_offset": True},
            {"layer_height_offset": 0.0},
            {"no_data_obstacle": False},
            {"no_data_block_skip_num": 0},
            {"min_block_point_num": 10},
            {"vehicle_height": 0.5},
            {"max_slope": 0.6}
            {"voxel_point_update_thre": 100},
            {"voxel_time_update_thre": 2.0},
            {"min_rel_z": -1.5},
            {"max_rel_z": 0.3},
            {"dis_ratio_z": 0.2},
            {"enable_obstacle_clearing": True},
            {"obstacle_clearing_dis": 8.0},
            {"obstacle_height_thre": 0.5},
            {"obstacle_confirm_frames": 2},
            {"obstacle_persist_frames": 1},
            {"side_blind_obstacle_preserve_frames": 30},
            {"enable_obstacle_sector_filter": true},
            {"front_hfov_deg": 150.0},   # 前向视野 150°（左右各75°），左右盲区约各30°
            {"rear_hfov_deg": 150.0},    # 后向视野 150°（左右各75°），左右盲区约各30°
            {"obstacle_blind_radius": 0.6},
            {"blind_zone_angle_bin_deg": 2.0},
            {"blind_zone_observation_window_bins": 1},
        ],
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(start_terrain_analysis_cmd)

    return ld
