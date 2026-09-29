from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params_file = PathJoinSubstitution([
        FindPackageShare('lidar_align_tool'),
        'config',
        'lidar_align.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='YAML file with default extrinsics / frame names.'),

        # 可在命令行覆盖的输入话题（指向你机器上的 Livox 原始话题）
        DeclareLaunchArgument(
            'lidar1_topic', default_value='livox/lidar_192_168_1_150',
            description='Lidar1 input CustomMsg topic.'),
        DeclareLaunchArgument(
            'lidar2_topic', default_value='livox/lidar_192_168_1_149',
            description='Lidar2 input CustomMsg topic.'),

        Node(
            package='lidar_align_tool',
            executable='lidar_align_tool',
            name='lidar_align_tool',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[
                ('livox/lidar_192_168_1_150', LaunchConfiguration('lidar1_topic')),
                ('livox/lidar_192_168_1_149', LaunchConfiguration('lidar2_topic')),
            ],
        ),
    ])
