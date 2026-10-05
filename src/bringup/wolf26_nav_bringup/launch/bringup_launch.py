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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import (
    IfCondition,
    LaunchConfigurationEquals,
    LaunchConfigurationNotEquals,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("wolf26_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    slam = LaunchConfiguration("slam")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    slowdown_map = LaunchConfiguration("slowdown_map")
    use_slowdown_zone = LaunchConfiguration("use_slowdown_zone")
    relocate = LaunchConfiguration("relocate")
    use_map_save = LaunchConfiguration("use_map_save")
    map_save_path = LaunchConfiguration("map_save_path")
    map_save_interval = LaunchConfiguration("map_save_interval")

    # In SLAM mapping mode the slowdown zone is never used, so the effective
    # value is forced to false regardless of what the user requested.
    use_slowdown_zone_effective = PythonExpression(
        ["'false' if '", slam, "'.lower() == 'true' else '", use_slowdown_zone, "'"]
    )

    # In SLAM mapping mode slam_launch.py already brings up map_saver_server and
    # its lifecycle manager; outside it nobody would, so periodic saving needs
    # its own map_saver chain.
    map_saver_standalone = PythonExpression(
        [
            "'",
            use_map_save,
            "'.lower() == 'true' and '",
            slam,
            "'.lower() != 'true'",
        ]
    )

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file}

    # Only it applies when `namespace` is not empty.
    # '<robot_namespace>' keyword shall be replaced by 'namespace' launch argument
    # in config file 'nav2_multirobot_params.yaml' as a default & example.
    # User defined config file should contain '<robot_namespace>' keyword for the replacements.
    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ("")},
        condition=LaunchConfigurationEquals("namespace", ""),
    )

    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ("/", namespace)},
        condition=LaunchConfigurationNotEquals("namespace", ""),
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="False", description="Whether run a SLAM"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", description="Full path to map yaml file to load"
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file", description="Full path to prior PCD file to load"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "reality", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_slowdown_map_cmd = DeclareLaunchArgument(
        "slowdown_map",
        default_value="",
        description="Full path to slowdown zone map yaml file",
    )

    declare_use_slowdown_zone_cmd = DeclareLaunchArgument(
        "use_slowdown_zone",
        default_value="true",
        description="Whether to use the slowdown zone feature (always disabled in SLAM mapping mode)",
    )

    declare_relocate_cmd = DeclareLaunchArgument(
        "relocate",
        default_value="true",
        description=(
            "Whether to launch small_gicp_relocalization. Set false when the "
            "map->odom transform is supplied from elsewhere, e.g. replaying a "
            "rosbag that already carries the full TF tree."
        ),
    )

    declare_use_map_save_cmd = DeclareLaunchArgument(
        "use_map_save",
        default_value="false",
        description="Whether to periodically save the map while the stack runs",
    )

    declare_map_save_path_cmd = DeclareLaunchArgument(
        "map_save_path",
        default_value="~/sentry26_maps/map",
        description=(
            "Output prefix for the periodic map save (no extension): "
            "<map_save_path>.pgm and <map_save_path>.yaml are overwritten each round"
        ),
    )

    declare_map_save_interval_cmd = DeclareLaunchArgument(
        "map_save_interval",
        default_value="60.0",
        description="Seconds between two periodic map saves",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            Node(
                condition=IfCondition(use_composition),
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[configured_params, {"autostart": autostart}],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "slam_launch.py")
                ),
                condition=IfCondition(slam),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "use_respawn": use_respawn,
                    "params_file": params_file,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "localization_launch.py")
                ),
                # 同上：裸 token 会拼成 `not false` → NameError。本文件自己声明的 slam
                # 默认是 "False"（能侥幸跑通），但上层 nav_launch.py 传进来的
                # mapping_mode 默认是小写 "false"，就会炸。
                condition=IfCondition(
                    PythonExpression(["'", slam, "'.lower() != 'true'"])
                ),
                launch_arguments={
                    "namespace": namespace,
                    "map": map_yaml_file,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "prior_pcd_file": prior_pcd_file,
                    "relocate": relocate,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "navigation_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                    "slowdown_map": slowdown_map,
                    "use_slowdown_zone": use_slowdown_zone_effective,
                }.items(),
            ),
        ]
    )

    # ── 周期存图 ──────────────────────────────────────────────────────────
    # nav2 的 map_saver_server 在 C++ 里把节点名硬编码成 "map_saver"（与 yaml 里的
    # `map_saver:` 参数段、lifecycle 的 node_names 对齐），这里显式写出来避免歧义。
    # 它是一个 lifecycle 节点，save_map 服务要等 lifecycle manager 把它 activate
    # 之后才存在，所以下面必须连着 map_saver_server + lifecycle manager 一起起。
    start_map_saver_server_cmd = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver",
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(map_saver_standalone),
    )

    start_map_saver_lifecycle_manager_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_saver",
        namespace=namespace,
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": ["map_saver"]},
        ],
        condition=IfCondition(map_saver_standalone),
    )

    start_periodic_map_saver_cmd = Node(
        package="wolf26_nav_bringup",
        executable="periodic_map_saver.py",
        name="periodic_map_saver",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_save_path": map_save_path,
                "save_interval": map_save_interval,
            }
        ],
        condition=IfCondition(use_map_save),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_slowdown_map_cmd)
    ld.add_action(declare_use_slowdown_zone_cmd)
    ld.add_action(declare_relocate_cmd)
    ld.add_action(declare_use_map_save_cmd)
    ld.add_action(declare_map_save_path_cmd)
    ld.add_action(declare_map_save_interval_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    # 周期存图（独立于 nav2 组合容器，map_saver 本来就是独立 lifecycle 节点）
    ld.add_action(start_map_saver_server_cmd)
    ld.add_action(start_map_saver_lifecycle_manager_cmd)
    ld.add_action(start_periodic_map_saver_cmd)

    return ld
