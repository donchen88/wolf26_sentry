# 第三方组件声明 / Third-Party Notices

本仓库 `src/` 下的以下内容来自上游开源项目。它们**保留各自的许可证与版权**，
**不受本仓库根目录 Apache-2.0 许可的影响**。在本仓库中它们是**内联**的（非 submodule），
下表记录其上游来源与所用基线 commit，便于日后与上游比对。

> 基线 commit 是整理本仓库时，各子目录工作树所对应的上游提交。

---

## 1. 来自 SMBU-PolarBear-Robotics-Team

| 目录 | 上游项目 | 许可证 | 基线 commit |
|---|---|---|---|
| `src/localization/point_lio` | [SMBU-PolarBear-Robotics-Team/point_lio](https://github.com/SMBU-PolarBear-Robotics-Team/point_lio) | **BSD-3-Clause**，但 `include/IKFoM/` 为 **GPL-2.0**（见 §3） | `641424b` (2024-12-25) |
| `src/localization/small_gicp_relocalization` | [SMBU-PolarBear-Robotics-Team/small_gicp_relocalization](https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization) | Apache-2.0 | `4b9d31d` (2025-02-28) |
| `src/perception/terrain_analysis` | [SMBU-PolarBear-Robotics-Team/terrain_analysis](https://github.com/SMBU-PolarBear-Robotics-Team/terrain_analysis) | Apache-2.0 | `c82f23c` (2025-02-15) |
| `src/perception/pointcloud_to_laserscan` | [SMBU-PolarBear-Robotics-Team/pointcloud_to_laserscan](https://github.com/SMBU-PolarBear-Robotics-Team/pointcloud_to_laserscan)（`ros-perception/pointcloud_to_laserscan` 的 fork） | BSD-3-Clause（Willow Garage / Eurotec） | `5efaa8b` (2025-02-24) |
| `src/navigation/pb_omni_pid_pursuit_controller` | [SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller](https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller) | Apache-2.0 | `9901878` (2025-03-10) |
| `src/interfaces/auto_aim_interfaces` | [SMBU-PolarBear-Robotics-Team/auto_aim_interfaces](https://github.com/SMBU-PolarBear-Robotics-Team/auto_aim_interfaces) | MIT | `5eab0e0` (2025-02-06) |

以下包来自 **`pb2025_sentry_nav` 单体仓库**（作者 Lihan Chen，
[SMBU-PolarBear-Robotics-Team](https://github.com/SMBU-PolarBear-Robotics-Team)），
该仓库为 Apache-2.0：

| 目录 | 许可证 | 备注 |
|---|---|---|
| `src/bringup/wolf26_nav_bringup` | Apache-2.0 | 本仓库中**唯一改名的包**（原 `pb2025_nav_bringup`）；包根保留上游 `LICENSE`，源码文件头的版权声明原样未动 |
| `src/navigation/fake_vel_transform` | Apache-2.0 | |
| `src/navigation/behavior_ext_plugins` | Apache-2.0 | |
| `src/localization/loam_interface` | Apache-2.0 | |
| `src/localization/sensor_scan_generation` | Apache-2.0 | |

---

## 2. 其它上游

| 目录 | 上游项目 | 作者 | 许可证 | 基线 commit |
|---|---|---|---|---|
| `src/navigation/spatio_temporal_voxel_layer` | [SteveMacenski/spatio_temporal_voxel_layer](https://github.com/SteveMacenski/spatio_temporal_voxel_layer) | Steve Macenski | **LGPL v2.1**（见 §3） | `92896fe` (2025-04-15) |
| `src/perception/livox_ros_driver2` | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | Livox | MIT（`LICENSE.txt`；`3rdparty/rapidjson` 见其自带许可） | `6b9356c` (2024-09-11) |
| `src/interfaces/roborts_msgs` | [RoboMaster/RoboRTS](https://github.com/RoboMaster/RoboRTS) | RoboMaster / DJI | **GPL-3.0**（见 §3） | — |

---

## 3. 需要特别注意的许可证

### 3.1 `spatio_temporal_voxel_layer` — LGPL v2.1

本包采用 **LGPL v2.1**，比仓库其余部分更严格：

1. 该目录下的 `LICENSE` 与所有源文件头的版权声明**原样保留，未做修改**。
2. 本仓库对其做过本地修改，依据 LGPL v2.1 **§6** 在此声明。已知修改：
   - 移除了 `openvdb_vendor` 子模块目录（依赖关系改由 apt 包
     `ros-humble-openvdb-vendor` 提供，`find_package(openvdb_vendor REQUIRED)`
     仍可满足）
   - 移除了 `.git` / `.gitmodules`（因其在本仓库中以源码形式内联）
   - 修改了部分源文件（具体见与上游 `92896fe` 的 diff）
3. 以**源码形式**分发天然满足 LGPL“允许用户替换该库”的要求。
   但若将来以**二进制形式**（`install/` 目录、Docker 镜像、预编译产物）分发，
   **必须同时提供该包的完整源码或其获取方式**。
4. 请勿将其静态链接进闭源产物。

### 3.2 `point_lio` — 内含 GPL-2.0 且实际参与编译

- `point_lio` 自身为 BSD-3-Clause。
- 但其 `include/IKFoM/`（IMU 误差状态卡尔曼滤波工具包）为 **GPL-2.0**
  （见 `src/localization/point_lio/include/IKFoM/LICENSE`），且**确实被编译进包中**：
  `include/common_lib.h` 直接 `#include <../include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp>`。
- 因此 **`point_lio` 整体应视为 GPL-2.0 衍生作品**。它与本仓库其余部分通过 ROS 2
  话题通信、各自独立成进程，属于 GPL-2.0 与 Apache-2.0 在同一仓库共存的常见安排；
  但**不能在发布时声称“整个仓库都是 Apache-2.0”**。

### 3.3 `roborts_msgs` — GPL-3.0

- `package.xml` 声明 `<license>GPL 3.0</license>`，来自 RoboMaster/RoboRTS。
- **该包未附带 LICENSE 文件**（上游即如此），此处按 `package.xml` 的声明记录。
- 本仓库实际只用到其中的 `ChassisCmd`（一个 4 字段的消息），但按决定**原样保留整个包**。

---

## 4. 本团队自有包

以下包由本队维护，非上述上游项目，但目前 `package.xml` 中的许可证声明为
自动生成时留下的占位符 `TODO: License declaration`，**尚未正式确定**：

| 目录 | `package.xml` 当前声明 |
|---|---|
| `src/interfaces/robot_msgs` | `TODO: License declaration` |
| `src/perception/lidar_align_tool` | `TODO: License declaration` |
| `src/perception/merge_cloud` | `TODO: License declaration` |

以下包由本队维护或重度定制：

| 目录 | 来源 | 许可证 |
|---|---|---|
| `src/navigation/pb_nav2_plugins` | `donchen88/wolf_sentry`（本队 fork，`08fe22c` 2026-03-29） | Apache-2.0 |
| `src/driver/rm_serial_driver_nav2` | 本队 `wildwolf_serial_nav2` | MIT |
| `src/interfaces/robot_msgs` | 本队 `wildwolf_serial_nav2` | 见上（TODO） |
| `src/interfaces/sp_msgs` | 本队（与自瞄共享的契约） | MIT |
| `src/perception/merge_cloud` | 本队 | 见上（TODO） |
| `src/perception/lidar_align_tool` | 本队 | 见上（TODO） |

> `src/interfaces/sp_msgs` 与 `src/interfaces/auto_aim_interfaces` 目前在本仓库内
> **没有消费者**（原先的使用者 `rm_behavior_tree` 已移除），保留原因是它们属于
> 对端（自瞄 / 视觉）的线缆契约。

---

## 5. 上游文档

`src/bringup/wolf26_nav_bringup/docs/upstream/` 下保留了上游 `pb2025_sentry_nav`
仓库的 `README.md`、`README_en.md`、`CONTRIBUTING.md`，作为署名与历史记录。
其中提到的包名已同步更新为本仓库的 `wolf26_nav_bringup`。

**注意**：这些上游文档的目录树仍列有 `pb_teleop_twist_joy`、`rm_behavior_tree` 等
本仓库已移除的包，以及 `gazebo_simulator`（从未纳入本仓库）。它们是上游原貌的存档，
**不代表本仓库的实际内容**；实际内容以根目录 [README.md](README.md) 为准。
