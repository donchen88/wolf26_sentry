# 先验点云地图（不入库）

本目录存放定位模块（`point_lio` / `small_gicp_relocalization`）使用的先验点云地图。

**这些文件不纳入版本控制**：完整的 `pcd/` 约 6.3GB，其中多个单文件超过 300MB，
远超 GitHub 单文件 100MB 的硬性上限。见仓库根目录 `.gitignore`。

## 目录约定

```
pcd/
├── reality/<world>.pcd        # 实车地图，文件名与 bringup 的 world 参数同名
└── simulation/<world>.pcd     # 仿真地图
```

`rm_navigation_reality_launch.py` 等入口按
`pcd/reality/<world>.pcd` 的规则拼路径（`world` 默认 `rmul_2024`）。

## 如何准备

**方式一：从已有工作区软链**（推荐，不占额外磁盘）

```bash
ln -sf ~/nav/guosai_nav/src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality/*.pcd \
       src/bringup/wolf26_nav_bringup/pcd/reality/
```

**方式二：自行拷贝**，把 `<world>.pcd` 放到 `pcd/reality/` 或 `pcd/simulation/`。

## 注意

- `CMakeLists.txt` 的 `ament_auto_package(INSTALL_TO_SHARE ... pcd ...)` 会安装本目录。
  用 `--symlink-install` 时是符号链接（不占空间）；**不带 `--symlink-install` 的全量
  build 会尝试拷贝 6.3GB**，请务必带上该参数。
- 栅格地图（`map/`）与减速区地图（`maps/`）**已入库**，无需在此准备。
