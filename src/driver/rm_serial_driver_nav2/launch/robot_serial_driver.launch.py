import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rm_serial_driver_nav2'), 'config', 'serial_driver.yaml')

    rm_serial_driver_node = Node(
        name='rm_serial_driver_nav2_node',
        package='rm_serial_driver_nav2',
        executable='rm_serial_driver_nav2_node',
        namespace='/move',
        output='screen',
        emulate_tty=True,
        parameters=[config],
        # nav 的 TF 话题在 /red_standard_robot1/ 下；
        # driver 节点在 /move namespace 下，tf2_ros::TransformListener 默认会订阅
        # /move/tf 与 /move/tf_static（namespace 自动加前缀），但实际 publisher 在
        # /red_standard_robot1/ 下、全局 /tf 与 /tf_static 上**根本没人发**，所以必须 remap。
        # 这里用绝对目标名（以 "/" 开头）在 ROS2 humble/foxy 上语义稳定，
        # 会直接覆盖 namespace 的隐式前缀，让节点去订阅 /red_standard_robot1/tf{,_static}。
        # ⚠️ 之前用 ('tf', '/red_standard_robot1/tf') 这种带前缀的绝对目标是正确的；
        # 如果仍然报 odom -> base_footprint 不存在，请同步运行下方
        #   ros2 node info /move/rm_serial_driver_nav2_node
        # 看 Subscriptions，确认 /red_standard_robot1/tf 与 /red_standard_robot1/tf_static
        # 是否真的出现在订阅列表里。如果出现的是 /tf、/tf_static 或 /move/tf、/move/tf_static，
        # 说明这次环境里绝对路径 remap 被吞了，得改回「相对 namespace」的写法，见 git history。
        remappings=[
            ('tf', '/red_standard_robot1/tf'),
            ('tf_static', '/red_standard_robot1/tf_static'),
        ],
    )

    return LaunchDescription([rm_serial_driver_node])
