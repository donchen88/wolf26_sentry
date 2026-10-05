cmake_minimum_required(VERSION 3.8)
project(wolf26_nav_bringup)

find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

# 可执行脚本：launch 里用 Node(package=..., executable=...) 直接拉起
install(PROGRAMS
  scripts/periodic_map_saver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  map
  maps
  pcd
  config
  behavior_trees
  rviz
)
