cmake_minimum_required(VERSION 3.5)
project(nav2_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(pb_back_up_frees_space_behavior SHARED
  src/behaviors/back_up_free_space.cpp
)

ament_auto_add_library(pb_bt_nodes SHARED
  src/behaviors/compute_path_to_pose_with_edge_waypoint.cpp
)
ament_target_dependencies(pb_bt_nodes SYSTEM nav2_behavior_tree nav2_costmap_2d)
target_compile_definitions(pb_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

# 库名不能叫 layers：Nav2 自带一份同 SONAME 的 liblayers.so（InflationLayer/
# ObstacleLayer/StaticLayer 都在里面），本包的 intensity_* 层继承自
# nav2_costmap_2d::ObstacleLayer，其 typeinfo 只在 Nav2 那份里。同名会让 dlopen 时
# 本库的 NEEDED liblayers.so 被自己顶掉，导致「配置里只有本包的层」时符号缺失、
# 整个库加载失败。
ament_auto_add_library(nav2_plugins_layers SHARED
  src/layers/intensity_obstacle_layer.cpp
  src/layers/intensity_voxel_layer.cpp
  src/layers/slowdown_zone_layer.cpp
)

ament_auto_add_library(escape_bias_layer SHARED
  src/layers/escape_bias_layer.cpp
)

if(BUILD_TESTING)
set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

# Install BT plugin XML for discovery by the BT engine
install(FILES behavior_tree_nodes.xml
        DESTINATION share/${PROJECT_NAME})

ament_auto_package()
