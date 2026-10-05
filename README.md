# 野狼哨兵导航系统（wolf26_nav）

RM 2026 赛季野狼战队哨兵导航工作区：**定位 / 感知 / 导航 / 机器人描述 / 串口驱动 / bringup**。

## 项目介绍

本项目是一个基于 Nav2 框架、面向 RoboMaster 哨兵机器人的 ROS 2 导航项目，由我作为野狼战队成员开发。系统以稳定和高鲁棒性为主要开发目标，让机器人能够快速通过复杂地形；在没有先验地图的情况下，也能稳定完成比赛。

为了在能够通过低矮障碍物的前提下维持车身姿态的稳定，我希望车尽可能在水平地面高速移动，因此增设了带 cost 的减速区。原本想在坡度识别里做自适应 cost，但地形识别效果实在不太好，就简化了思路：以先验地图为主体新增一个减速层，在减速带路段也做特殊速度处理，既保证通过速度也保证通过率（速度太快冲上去，车身姿态有概率倾斜卡住）。加入 cost 之后，规划的路线也能维持在非减速区上，高效移动去往目标点，避免路线穿过复杂地形。在建图导航上也对未知区域的导航进行了目标点的修改处理（计算出可达的最近目标点再前往未知区域，原本想直接用在赛场上但效果并不理想，只在区域赛适应性时尝试用过一次），所以整体的赛场流程还是在无先验的前提时，用纯lio手摆的方式先跑一场比赛，比赛后再用rosbag的录包用mapping_launch.py来复现赛场建图。

开发这个项目时，大一的学习过程没有学扎实，理论和技术的经验都相当不足，所以绝大部分内容只是基于已有开源做小幅度、针对性的适配改动，以调参测试为主要实现方式。在整一年做导航的过程中也走了不少弯路，最终的效果也没有很好，但还是希望自己能够在这个比赛中留下点什么吧。目前只公开了导航部分，后续有概率更新优化或公开决策或剩下的内容。

## 项目结构

```text
src/
├── localization/      状态估计与定位
│   ├── point_lio/                     LiDAR 惯性里程计
│   ├── small_gicp_relocalization/     基于先验点云的重定位
│   ├── loam_interface/                LOAM 系接口适配
│   └── sensor_scan_generation/        传感器扫描生成
├── perception/        传感器驱动与点云处理
│   ├── livox_ros_driver2/             Livox MID-360 驱动
│   ├── merge_cloud/                   双雷达点云融合
│   ├── lidar_align_tool/              雷达外参标定
│   ├── terrain_analysis/              地形分析
│   └── pointcloud_to_laserscan/       点云转激光扫描
├── navigation/        Nav2 插件与控制器
│   ├── nav2_plugins/                  自定义 costmap 层与行为插件（含减速区层）
│   ├── behavior_ext_plugins/          扩展行为插件
│   ├── pb_omni_pid_pursuit_controller/ 全向 PID 追踪控制器
│   ├── spatio_temporal_voxel_layer/   STVL 三维体素层（LGPL v2.1）
│   └── fake_vel_transform/            速度变换（底盘坐标系适配）
├── interfaces/        消息契约
│   ├── robot_msgs/                    底盘/裁判系统消息
│   ├── roborts_msgs/                  RoboRTS 消息（GPL-3.0）
│   ├── sp_msgs/                       与自瞄共享消息
│   └── auto_aim_interfaces/           与视觉共享消息
├── description/       机器人模型描述
│   ├── pb2025_robot_description/      SDF(xmacro) 整机模型与静态外参（MIT）
│   ├── sdformat_tools/                SDF → URDF 转换（launch 期调用）
│   └── rmoss_gz_resources/            RoboMaster 模型资源（Apache-2.0）
├── driver/
│   └── rm_serial_driver_nav2/         下位机串口驱动
└── bringup/
    └── wolf26_nav_bringup/            启动文件、参数、地图与行为树
        ├── launch/                    顶层入口与各模块 launch
        ├── config/reality/            参数文件与雷达 user_config
        ├── behavior_trees/            Nav2 行为树
        ├── map/  maps/  pcd/          栅格地图 / 减速区地图 / 先验点云
        ├── rviz/                      RViz 配置
        ├── docs/upstream/             上游 README 存档（见 THIRD_PARTY_NOTICES）
        └── scripts/                   周期存图脚本
```


## 启动配置

两个顶层入口：实车导航与建图各一个。

| 启动文件 | 用途 | 默认参数文件 | 默认起 RViz |
|---|---|---|---|
| `rm_navigation_reality_launch.py` | 实车导航（定位 + Nav2）/建图 | `config/reality/nav2_params.yaml` | 是 |
| `mapping_launch.py` | 录包建图（point_lio 建图 + slam_toolbox） | `config/reality/slam_param.yaml` | 是 |


### 减速区开关

默认 `true`。**`slam:=true` 时强制置为 `false`** —— 建图模式不需要减速区地图，
且 `slowdown_map_server` 不会被拉起（`lifecycle_manager` 的 `node_names`
会相应不含它，避免等待不存在的节点而死锁）。
（打开减速区后需要将修改出来减速栅格地图放在`maps`文件夹里面，并将`yaml`文件中的地图类型修改为`scale`,两个地图的相对坐标系不要去修改，不然会对不上）

`rm_navigation_reality_launch.py` 开启减速区时，`world` 必须对应到 `maps/reality/`
下真实存在的 `<world>_slowdown.yaml`，否则 `slowdown_map_server` 加载不到图就起不来
—— 要么自己补一张减速区图，要么 `use_slowdown_zone:=false`。



## 环境与构建

当前开发环境：

- Ubuntu 22.04
- ROS 2 Humble

依赖安装：

```bash
# Nav2 与相关组件
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-nav2-smac-planner ros-humble-slam-toolbox

# spatio_temporal_voxel_layer 依赖（注意：这是 apt 包，不是源码）
sudo apt install ros-humble-openvdb-vendor

# 其余 ROS 依赖一键安装
rosdep install --from-paths src --ignore-src -r -y

# use_robot_state_pub:=true（RViz 显示机器人模型）前置：xmacro 是 pip 包，
# rosdep / apt 都装不到，需单独 pip 安装（本机已有 1.2.1）
pip install xmacro
```

**Livox SDK2** —— `livox_ros_driver2` 编译前需先安装，见
[src/perception/livox_ros_driver2/README.md](src/perception/livox_ros_driver2/README.md)。
本机已装在 `/usr/local`（`/usr/local/include/livox_sdk.h`）。

### 编译

```bash
cd ~/nav/wolf26_nav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`--symlink-install` **建议始终带上**：本工作区是 symlink-install 布局，launch 与
python 改动无需重新 build 即生效；且 bringup 包会把 `pcd/` 一并 install，
不带 symlink 时会尝试拷贝大文件。

## 实车部署

仓库中的 IP、外参与串口设备名都对应本车，换车或移植到其他机器人前需要按实际硬件修改。

### 雷达网络与驱动配置

两台 Livox MID-360 的地址与端口定义在
[`config/reality/mid360_user_config.json`](src/bringup/wolf26_nav_bringup/config/reality/mid360_user_config.json)：

- `host_net_info` 为 `192.168.1.50`：NUC 连接雷达网卡的静态 IP；
- `lidar_configs` 为两台雷达 `192.168.1.150` / `192.168.1.149`；
- 端口：cmd `56100/56101`、push `56200/56201`、point `56300/56301`、imu `56400/56401`。

部署时至少需要改这几处，并保证 NUC 与雷达处于同一网段、上述端口没有被其他程序占用。
参数文件中的 `user_config_path` 指向这份 json（当前为
`$(find-pkg-share wolf26_nav_bringup)/config/reality/mid360_user_config.json`）。

### 双雷达与单雷达切换

当前两份参数文件（`nav2_params.yaml` / `slam_param.yaml`）都是双雷达配置：

- 驱动侧 `multi_topic: 1`，两台雷达各发一路话题：`livox/lidar_192_168_1_150`、
  `livox/lidar_192_168_1_149`（话题名由雷达 IP 生成）；
- `merge_cloud` 订阅这两路，做帧级时间对齐后发布 `merged_custom`（同时发布 `merged_cloud`）；
- `point_lio` 的 `lid_topic` 配的正是 `merged_custom`。

> **注意**：`merge_cloud` 的订阅话题名在源码里写死为上面两个 IP 形式
> （[merge_cloud_code.cpp:31-32](src/perception/merge_cloud/src/merge_cloud_code.cpp#L31-L32)），
> 改雷达 IP 时要同步改源码。

单雷达时，`lidar_configs` 只保留实际使用的那台，并把 `multi_topic` 改为 `0`：
驱动会直接发布 `livox/lidar`，需要把 `point_lio` 的 `lid_topic` 一并改成 `livox/lidar`，
否则定位收不到点云。

### 串口与底盘通信

- `expected_yaw` 等「目标相对当前底盘 yaw」的计算参数与整车朝向相关，换车或改动安装后
  需要重新标定。（是针对于过起伏路段，以map坐标系下的绝对正方向来向下位机提供通过起伏路段的底盘朝向）

### TF 与静态外参

整机静态 TF（雷达、底盘之间的外参）实车上由独立的机器人启动模块提供，本仓库只消费
TF 树；桌面回放调试时，`/tf`、`/tf_static` 也可由 rosbag 提供。

仓库已在 `src/description/` 内联了机器人描述包，**`use_robot_state_pub:=True`** 时会启动
`robot_state_publisher` 与 `joint_state_publisher`：launch 期由 `sdformat_tools` 把
SDF(xmacro) 模型转成 URDF（mesh 解析成 `install/` 下绝对路径，所以要先 build 并 source
本工作区），发布 `robot_description` 与整机 TF（顶层入口默认在 `red_standard_robot1`
命名空间下，话题即 `/red_standard_robot1/robot_description`），供 RViz 的 RobotModel
显示——桌面或没有整车模块时用它补 TF 树即可。若单独运行导航链路（既没有整车模块、bag 里也没有
TF），也可只关注 `front_mid360` 等 frame 自行补静态变换，否则 TF 树会断。

## 运行

```bash
# 实车导航:
ros2 launch wolf26_nav_bringup rm_navigation_reality_launch.py \
world:=<YOUR_WORLD_NAME> \
slam:=False \
use_robot_state_pub:=True

#建图：
ros2 launch wolf26_nav_bringup rm_navigation_reality_launch.py \
    slam:=True \
    use_robot_state_pub:=True\
    use_slowdown_zone:=false

# 录包建图：
ros2 launch wolf26_nav_bringup mapping_launch.py \
slam:=True \
use_robot_state_pub:=True
# 建图模式（slam:=True）下 slam_launch.py 已把 pcd_save.pcd_save_en 强制为 true，无需改参数；
# 正常关闭（Ctrl-C）后点云保存到 src/localization/point_lio/PCD/scans.pcd

# 保存栅格地图
ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>

# 选的地图没有配减速区图时，关掉减速区
ros2 launch wolf26_nav_bringup rm_navigation_reality_launch.py \
    slam:=False \
    use_robot_state_pub:=True\
    world:=rmul_2025 \
    use_slowdown_zone:=false
```

回放带 `/clock` 的包时加 `use_sim_time:=true`；包本身已带完整 TF 树时再关掉 small_gicp，
避免两边往同一根 TF 上写：

```bash
ros2 bag play <bag-path> --clock
ros2 launch wolf26_nav_bringup rm_navigation_reality_launch.py \
    use_sim_time:=true relocate:=false
```

实车导航前记得先按 [pcd/README.md](src/bringup/wolf26_nav_bringup/pcd/README.md)
（「先验点云地图」）把 `pcd/reality/<world>.pcd` 备好（small_gicp 重定位要用，仓库里
`pcd/` 只有 `.gitkeep`）；回放或桌面上没有实车时，
`livox_ros_driver2` / `merge_cloud` / `rm_serial_driver_nav2` 这几个无条件拉起的节点会报
「找不到设备」之类的错误，但不影响其余节点。

## 参考与致谢

本仓库根目录 `LICENSE` 为 **Apache-2.0，仅覆盖本仓库作者的原创部分**。
`src/` 下的第三方包各自适用其自带许可证（含 **LGPL v2.1** 与 **GPL**），
**不因本仓库的 Apache-2.0 而改变**。逐包署名、上游 URL 与基线 commit 见
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

特别感谢以下上游项目与作者：

- [SMBU PolarBear Robotics Team：pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav)
- [SMBU PolarBear Robotics Team：rmu_gazebo_simulator](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator)
- [PolarisXQ：SCURM_SentryNavigation](https://github.com/PolarisXQ/SCURM_SentryNavigation)
- [yuzhuohao111：hx_Sentry_2025](https://github.com/yuzhuohao111/hx_Sentry_2025)
- [Zhiang QI：cod_-rm2026_-navigation](https://gitee.com/codnavgation/cod_-rm2026_-navigation)
- 中国科学技术大学RoboWalker战队《2025赛季哨兵技术报告》
- **Steve Macenski** —— `spatio_temporal_voxel_layer`（LGPL v2.1）
- **Livox / Livox-SDK** —— `livox_ros_driver2`
- **RoboMaster / RoboRTS** —— `roborts_msgs`（GPL-3.0）

后期也在不断学习如何调出适合狗洞哨的导航，但在各种因素的导致下还是胎死腹中，算是这年比赛的一大遗憾吧。虽然本人不是很厉害，但还是希望此开源会有所帮助，也算是给这个赛季的一个交代了。本人是第一次写开源，可能会存在不妥的地方，如有任何代码问题请及时指出 qq：1678894833 欢迎指点批评