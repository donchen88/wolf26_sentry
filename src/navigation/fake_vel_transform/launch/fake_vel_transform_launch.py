# Copyright 2025 Lihan Chen
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


from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    fake_vel_transform_node = Node(
        package="fake_vel_transform",
        executable="fake_vel_transform_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "enable_velocity_filter": True,
            "filter_time_constant": 0.10,
            "linear_speed_deadband": 0.06,
            "angular_speed_deadband": 0.05,
            "max_linear_accel": 5.0,
            "max_angular_accel": 6.0,
            "enable_velocity_jump_check": True,
            "velocity_jump_linear_threshold": 1.0,
            "velocity_jump_angular_threshold": 1.0,
            "velocity_direction_jump_threshold": 0.8,
            "direction_valid_speed_threshold": 0.06,
            "auto_aim_disable_rotation": True,
            "gimbal_angle_change_threshold": 0.5,
            "speed_reduction_factor": 0.3,
            "recovery_time": 0.5,
        }],
    )

    ld = LaunchDescription()

    ld.add_action(fake_vel_transform_node)

    return ld
