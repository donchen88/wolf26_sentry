# lidar_align_tool

独立的**雷达外参对齐调试节点** —— 不修改原有 `merge_cloud_node`。

订阅两路 Livox `CustomMsg` 原始点云，一边转发原始包让你在 RViz 里看未对齐的点云，
一边按当前外参把它们变换到统一坐标系 `camera_init` 再发布，并在 `rqt_reconfigure`
中提供 12 个滑动条实时调外参；同时通过 `tf2` 周期广播两个雷达坐标系的位置，
在 RViz 中也能直接看到两个坐标系随着滑动条的拖动实时变化。

## 运行

```bash
# 0. source 工作空间
source /opt/ros/humble/setup.bash
source <workspace>/install/setup.bash

# 1. 启动节点（默认 192.168.1.150 / 192.168.1.149 的 Livox）
ros2 launch lidar_align_tool lidar_align_tool.launch.py

# 2. 另一个终端，启动 rqt_reconfigure
ros2 run rqt_reconfigure rqt_reconfigure
# 在左侧选 /lidar_align_tool，弹出 12 个外参滑动条

# 3. 再开 RViz：Fixed Frame 选 "camera_init"
#    添加 PointCloud2：
#      Topic /lidar1_raw      -> 看到 Lidar1 原始位置
#      Topic /lidar2_raw      -> 看到 Lidar2 原始位置
#      Topic /lidar1_aligned  -> Lidar1 对齐到 camera_init
#      Topic /lidar2_aligned  -> Lidar2 对齐到 camera_init
#      Topic /merged_aligned  -> 合并后，可看对齐程度
#    Topic 列表里默认有 88 / 89 两种 intensity，可做颜色渐变按 Intensity 显示
```

## 发布话题与坐标系

| 话题 | 坐标系 | 内容 |
| --- | --- | --- |
| `/lidar1_raw`     | `livox_frame1`  | Lidar1 原始（=话题原样） |
| `/lidar2_raw`     | `livox_frame2`  | Lidar2 原始（=话题原样） |
| `/lidar1_aligned` | `camera_init`   | 应用外参后的 Lidar1，intensity=88 |
| `/lidar2_aligned` | `camera_init`   | 应用外参后的 Lidar2，intensity=89 |
| `/merged_aligned` | `camera_init`   | 上面两者直接相加 |

intensity = 88 → Lidar1，89 → Lidar2。RViz 中按 Intensity 着色即可看到两雷达是否对齐。

## TF

该节点周期性发布：

```
camera_init -> livox_frame1
camera_init -> livox_frame2
```

滑动任意外参滑动条，TF 会跟着变；RViz 坐标系 / PointCloud2 都实时跟随。

## 参数（滑动条）

| 参数 | 默认值 | 范围 | 步长 |
| --- | --- | --- | --- |
| `roll1`/`pitch1`/`yaw1` | 0 / 0 / 0（deg） | ±180 / ±180 / ±360 | 0.01° |
| `tx1`/`ty1`/`tz1`       | 0.2310 / 0 / 0.2867 | ±2 m | 0.0001 m |
| `roll2`/`pitch2`/`yaw2` | 1.70 / 76.70 / 182.0 | ±180 / ±90 / ±360 | 0.01° |
| `tx2`/`ty2`/`tz2`       | -0.2201 / 0 / 0.1360 | ±2 m | 0.0001 m |

## 命令行覆盖

```bash
ros2 launch lidar_align_tool lidar_align_tool.launch.py \
    yaw2:=183.5 \
    lidar1_topic:=/livox/mid360_left \
    lidar2_topic:=/livox/mid360_right
```

修改默认值请编辑 `config/lidar_align.yaml`。

## 与原 merge_cloud_node 的关系

`lidar_align_tool` 与 `merge_cloud_node` 完全独立，可同时运行。一般用法：
- 运行 `lidar_align_tool` + `rqt_reconfigure` 找到好的外参；
- 把外参填回 `merge_cloud/config/merge_cloud.yaml` 给生产节点使用。

也可保留两个节点同时运行，互不影响。
