import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bt_config_dir = os.path.join(
        get_package_share_directory('rm_behavior_tree'),
        'config'
    )

    style = LaunchConfiguration('style', default='complete.xml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    bt_xml = PathJoinSubstitution([bt_config_dir, style])

    rm_behavior_tree_node = Node(
        package='rm_behavior_tree',
        executable='rm_behavior_tree',
        respawn=True,
        respawn_delay=3,
        parameters=[
            {
              'bt_xml': bt_xml,
              'use_sim_time': use_sim_time,
            }
        ],
        remappings=[
            ('/tf', '/red_standard_robot1/tf'),
            ('/tf_static', '/red_standard_robot1/tf_static'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('style', default_value='complete.xml'),
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        rm_behavior_tree_node,
    ])
