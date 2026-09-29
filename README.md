# wolf26_nav

RM 2026 赛季哨兵导航工作区：**定位 / 感知 / 导航 / 行为树 / 串口驱动 / bringup**。

代码整理自赛前工作区 `guosai_nav`，来源为三个上游（SMBU-PolarBear 的
`pb2025_sentry_nav`、`SMBU-POLARBEAR/rm_behavior_tree`、`wolf_nav` 的
`wildwolf_serial_nav2`）。上游代码**全部内联**在 `src/` 下（无 submodule），
一次 clone 即可编译。

---

## 1. 目录结构

按**数据流方向**分组：定位产出 TF → 感知产出点云/栅格 → 导航消费二者并输出速度 →
行为树做决策；`interfaces` / `driver` / `bringup` 是横切的契约、执行与装配层。

```
src/
├── localization/   状态估计与定位
│   ├── point_lio/                     LiDAR 惯性里程计
│   ├── small_gicp_relocalization/     基于先验点云的重定位
│   ├── loam_interface/                LOAM 系接口适配
│   └── sensor_scan_generation/        传感器扫描生成
├── perception/     传感器驱动与点云处理
│   ├── livox_ros_driver2/             Livox MID-360 驱动
│   ├── merge_cloud/                   双雷达点云融合
│   ├── lidar_align_tool/              雷达外参标定
│   ├── terrain_analysis/              地形分析
│   ├── pointcloud_to_laserscan/       点云转激光扫描
│   └── ign_sim_pointcloud_tool/       仿真点云转换
├── navigation/     Nav2 插件与控制器
│   ├── pb_nav2_plugins/               自定义 costmap 层与行为插件（含减速区层）
│   ├── behavior_ext_plugins/          扩展行为插件
│   ├── costmap_intensity/             代价地图强度层（**在研，未启用**）
│   ├── pb_omni_pid_pursuit_controller/ 全向 PID 追踪控制器
│   ├── pb_teleop_twist_joy/           手柄遥控
│   ├── spatio_temporal_voxel_layer/   STVL 三维体素层（LGPL v2.1）
│   └── fake_vel_transform/            速度变换（底盘坐标系适配）
├── behavior_tree/  行为树决策层
│   ├── rm_behavior_tree/              比赛决策行为树（发送 NavigateToPose）
│   └── BehaviorTree.ROS2/             BehaviorTree.CPP 的 ROS 2 绑定
├── interfaces/     消息契约
│   ├── rm_decision_interfaces/        决策层自定义消息
│   ├── robot_msgs/                    底盘/裁判系统消息
│   ├── roborts_msgs/                  RoboRTS 消息（GPL-3.0，见 §6）
│   ├── sp_msgs/                       与自瞄共享的契约
│   └── auto_aim_interfaces/           与视觉共享的契约
├── driver/
│   └── rm_serial_driver_nav2/         下位机串口驱动
└── bringup/
    └── wolf26_nav_bringup/            启动文件与参数（唯一改名的包）
```

> **为什么 `sp_msgs` / `auto_aim_interfaces` 不加 `wolf26_` 前缀**：它们是与自瞄/视觉
> 共享的**线缆契约**。ROS 2 中「包名 = 话题上的消息 type 字符串」，改包名会让对端
> 代码在 `create_subscription<T>()` 处静默炸掉。同理，其余上游包名一律保持原样，
> 以便继续与上游 `git diff`。

---

## 2. 依赖

- **ROS 2 Humble**

```bash
# Nav2 与相关组件
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-nav2-smac-planner ros-humble-slam-toolbox

# spatio_temporal_voxel_layer 依赖（注意：这是 apt 包，不是源码）
sudo apt install ros-humble-openvdb-vendor
```

- **Livox SDK2** —— `livox_ros_driver2` 编译前需先安装，见
  [src/perception/livox_ros_driver2/README.md](src/perception/livox_ros_driver2/README.md)。
  本机已装在 `/usr/local`（`/usr/local/include/livox_sdk.h`）。
- ROS 依赖一键安装：`rosdep install --from-paths src --ignore-src -r -y`

---

## 3. 编译

```bash
cd ~/nav/wolf26_nav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`--symlink-install` **建议始终带上**：本工作区是 symlink-install 布局，launch 与
python 改动无需重新 build 即生效；且 bringup 包会把 `pcd/` 一并 install，
不带 symlink 时会尝试拷贝大文件。

### 3.1 先验点云地图（`pcd/`）不入库

`pcd/` 共 6.3GB，远超 GitHub 单文件 100MB 硬限，因此**不入版本控制**（见 `.gitignore`）。
首次使用需自行准备，或从已有工作区软链过来：

```bash
mkdir -p src/bringup/wolf26_nav_bringup/pcd
ln -sf ~/nav/guosai_nav/src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality/*.pcd \
       src/bringup/wolf26_nav_bringup/pcd/
```

**栅格地图（`map/`）与减速区地图（`maps/`）已入库**，无需额外准备。

---

## 4. 启动

```bash
# 实车导航（nav2_params.yaml）
ros2 launch wolf26_nav_bringup rm_navigation_reality_launch.py

# 防守配置（fangshou.yaml + navigate_to_pose_no_heading_extension.xml）
ros2 launch wolf26_nav_bringup rm_fangshou_launch.py

# 建图（SLAM）
ros2 launch wolf26_nav_bringup mapping_launch.py

# 行为树决策
ros2 launch rm_behavior_tree rm_behavior_tree.launch.py
```

### 减速区开关

实车入口支持在启动前切换减速区功能：

```bash
ros2 launch wolf26_nav_bringup rm_navigation_reality_launch.py use_slowdown_zone:=false
ros2 launch wolf26_nav_bringup rm_fangshou_launch.py use_slowdown_zone:=false
```

- 默认值：reality / fangshou 为 `true`，simulation 为 `false`
- **`slam:=True` 时强制置为 `false`** —— 建图模式不需要减速区地图，
  且 `slowdown_map_server` 不会被拉起（`lifecycle_manager` 的 `node_names`
  会相应不含它，避免等待不存在的节点而死锁）

### 仿真入口

`rm_navigation_simulation_launch.py` / `rm_multi_navigation_simulation_launch.py`
需要 Gazebo 世界文件与仿真地图，而 **`gazebo_simulator` 不在本仓库内**
（体积 183MB，绝大多数是上游 rmoss 代码），因此这两个入口在本仓库中无法独立运行。

---

## 5. 致谢与许可

本仓库根目录 `LICENSE` 为 **Apache-2.0，仅覆盖本仓库作者的原创部分**。
`src/` 下的第三方包各自适用其自带许可证（含 **LGPL v2.1** 与 **GPL**），
**不因本仓库的 Apache-2.0 而改变**。逐包署名、上游 URL 与基线 commit 见
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

特别感谢以下上游项目与作者：

- **SMBU-PolarBear-Robotics-Team** —— `pb2025_sentry_nav` 及其子模块
  （`point_lio`、`small_gicp_relocalization`、`terrain_analysis`、
  `pb_omni_pid_pursuit_controller`、`pb_teleop_twist_joy`、`pointcloud_to_laserscan`、
  `pb_nav2_plugins`、`auto_aim_interfaces`），作者 Lihan Chen 等
- **SMBU-POLARBEAR** —— `rm_behavior_tree`、`rm_decision_interfaces`
- **Steve Macenski** —— `spatio_temporal_voxel_layer`（LGPL v2.1）
- **Livox / Livox-SDK** —— `livox_ros_driver2`
- **Davide Faconti** —— `BehaviorTree.ROS2`
- **RoboMaster / RoboRTS** —— `roborts_msgs`（GPL-3.0）

---

## 6. 已知问题

- **`pcd/` 不入库**，首次使用需按 §3.1 准备。
- **`costmap_intensity` 为在研包**，当前没有任何 `nav2_params.yaml` 引用它
  （引用均被注释掉），不影响运行。
- **`roborts_msgs` 被 colcon 识别为 `ros.catkin`**：它的 `package.xml` 缺少
  `<export><build_type>ament_cmake</build_type></export>` 块，colcon 因此回退到
  ROS 1 catkin 判定。实际内容与 `CMakeLists.txt` 都是标准 ament_cmake，编译与安装
  均正常，仅会在 build 时产生两条 `CATKIN_* 未使用变量` 的 CMake 警告。
  本仓库刻意保持原样未改。
- **`point_lio` 内含 `include/IKFoM/`（GPL-2.0）且实际参与编译**，因此
  `point_lio` 整体应视为 GPL-2.0 衍生作品，详见 THIRD_PARTY_NOTICES.md。
- 已移除上游 `pb2025_nav_bringup` 对**不存在的包** `terrain_analysis_ext` 的依赖声明。
- `bringup_launch.py` / `localization_launch.py` / `slam_launch.py` /
  `bringup_fangshou_launch.py` 的 `params_file` 默认值原指向不存在的 `params/` 目录，
  已修正为 `config/reality/`。
  （正常入口 `rm_*_launch.py` 会显式传参，此前不受影响。）
