# 图构建器使用说明

## 功能概述

图构建器系统实现了从关键点到战术地图的自动构建：
1. **加载关键点**：从 YAML 配置文件加载战术关键点
2. **地图订阅**：订阅 `/map` 话题获取静态地图
3. **Raycast 检查**：使用 Bresenham 算法检查两点之间的连通性
4. **图构建**：自动构建无向图（节点+边）
5. **可视化**：在 rviz2 中显示关键点和边

## 使用方法

### 1. 编译

```bash
cd ~/ros/ros_ws
colcon build --packages-select rm_behavior_tree
source install/setup.bash
```

### 2. 运行测试节点

```bash
ros2 run rm_behavior_tree test_graph_builder \
  --ros-args \
  -p keypoint_yaml:="/home/mwk/ros/ros_ws/src/rm_behavior_tree/rm_behavior_tree/config/key_point.yaml" \
  -p map_topic:="/red_standard_robot1/map" \
  -p frame_id:="map"

# 可选参数（按需添加）
# -p max_edge_length:=12.0          # 最大连边距离（米，<=0 不限制）
# -p min_clearance_cells:=1         # Raycast 安全间隔（栅格数，建议 1~2）
# -p occupancy_threshold:=50        # 占用阈值（0-100）
# -p blacklist_edges:="[1,2, 3,4]"  # 黑名单边，成对写入 (id1,id2)
```

### 示例：一次性删除多条边
下面的命令会把以下边从图中移除：
- (1,7), (2,8), (8,13), (8,1), (9,3), (9,4), (10,12), (11,12), (27,29), (27,30)

```bash
ros2 run rm_behavior_tree test_graph_builder \
  --ros-args \
  -p keypoint_yaml:="/home/mwk/ros/ros_ws/src/rm_behavior_tree/rm_behavior_tree/config/key_point.yaml" \
  -p map_topic:="/red_standard_robot1/map" \
  -p frame_id:="map" \
  -p max_edge_length:=12.0 \
  -p min_clearance_cells:=1 \
  -p occupancy_threshold:=50 \
  -p blacklist_edges:="[1,7, 2,8, 8,13, 8,1, 9,3, 9,4, 10,12, 11,12, 27,29, 27,30, 26,29]"
```

##A* path from id 9 to 3 cost=10.020:
表示 A* 找到了一条从关键点 ID=9 到 ID=3 的路径，总代价（路径长度）为 10.020（单位：米）。
路径节点逐行解释：
node idx=8 id=9 (2.967, -3.887)：这里的 node idx 是在 graph.nodes 数组里的索引（从0开始），id 是你 YAML 中的关键点 ID，括号是该关键点的地图坐标 (x, y)。
接下来 node idx=0 id=1 (4.542, -1.530)，然后 node idx=2 id=3 (3.896, 5.626) —— 即实际经过的关键点序列是 ID 9 → ID 1 → ID 3。

### 3. 在 rviz2 中查看可视化

1. 启动 rviz2：
```bash
rviz2
```

2. 添加以下显示项：
   - **MarkerArray**：话题 `/graph_visualization`
     - 显示关键点（绿色球体）和边（蓝色线段）
   - **Map**：话题 `/map`（如果需要同时显示地图）

3. 设置 Fixed Frame 为 `map`

## 可视化说明

- **绿色球体**：关键点（节点）
- **蓝色线段**：边（表示两点之间可以直线到达）

## 验证要点

检查可视化结果是否正确：
- ✅ 关键点位置正确（在你标定的位置）
- ✅ 边没有穿过墙壁
- ✅ 通道是连通的（有边连接）
- ✅ 被墙隔开的点之间没有边

## 输出信息

节点会输出以下信息：
- 加载的关键点数量
- 地图信息（尺寸、分辨率）
- 图构建耗时
- 节点数和边数
- 每个节点的连接统计

## 下一步

完成图构建后，可以：
1. 使用图结构进行路径搜索（Dijkstra、A*等）
2. 在行为树中实现拦截节点
3. 根据敌人位置预测拦截点
 4. （建议的实现顺序）下面按步骤把从“图”到“拦截行为”所需的工作列成可执行的链路，便于按序实现和测试。

### 后续开发与测试步骤（按顺序）

1) 暴露路径查询接口（已实现）
   - 我已经实现了基于话题的简单接口：
     - 订阅：`/graph_planner/request` (std_msgs/Int32MultiArray)，请求格式：`[start_id, goal_id]`
     - 发布：`/graph_planner/response` (std_msgs/Int32MultiArray)，返回路径为关键点 ID 列表
     - 发布：`/graph_planner/response_cost` (std_msgs/Float64)，返回路径总代价（米）
   - 测试（示例）：
     ```bash
     # 运行 planner 节点（如果没启动）
     ros2 run rm_behavior_tree graph_planner_node \
       --ros-args -p keypoint_yaml:="/home/mwk/ros/ros_ws/src/rm_behavior_tree/rm_behavior_tree/config/key_point.yaml" \
       -p map_topic:="/red_standard_robot1/map"

     # 发送请求（从 9 到 3）
     ros2 topic pub /graph_planner/request std_msgs/Int32MultiArray "{data: [9, 3]}" --once

     # 查看响应
     ros2 topic echo /graph_planner/response --once
     ros2 topic echo /graph_planner/response_cost --once
     ```

2) 订阅敌人位置并估算速度（实现要点）
   - 订阅话题：优先 `/armors`（auto_aim_interfaces/msg/Armors）或 `/enemy_pose`（geometry_msgs/Pose）——两者都兼容。
   - 估算方法：
     - 缓存最近 N 帧（如 N=5）敌人位置与时间戳；
     - 使用差分或线性最小二乘拟合得到速度向量 v_enemy（m/s）；
     - 输出平滑的速度估计与可信度（cov 或 std）。
   - 接口/话题建议：
     - 订阅 `/armors` 或 `/enemy_pose`
     - 发布平滑的 `/enemy_state`（自定义 msg 包含 pose + velocity + stamp）以供决策节点使用

3) 预测敌人到关键点的到达时间（预测模型）
   - 对每个候选关键点 kp：
     - 计算敌人当前位置到 kp 的欧氏距离 d_enemy_kp；
     - 如果有速度向量，估算敌人到达时间 t_enemy = d_enemy_kp / |v_enemy|（若速度很小或不可靠，可退化为假设：敌人沿直线或使用多个假设方向）；
     - 如果敌人速度方向与 kp 方向不一致，可补偿或降低置信度。

4) 计算我方到关键点的到达时间（使用图与 A*）
   - 对每个候选关键点 kp：
     - 从我方当前所在最近关键点（或直接用当前位置插入图）使用 Planner（A*）查询到 kp 的路径代价（米）；
     - 估算行驶时间 t_self = cost / v_self（v_self 由机器人最大速度或当前移动速度给定）。

5) 选择拦截点（决策规则）
   - 候选条件（示例）：
     - 选择使得 t_self <= t_enemy 的点（我方能先到或同时到达）；
     - 如果多个点满足，选择时间差最小或优先级更高（例如靠近我方或视野好的点）；
     - 如果没有点满足，选择使得 (t_enemy - t_self) 最小的点（最接近拦截）。
   - 输出：选定关键点 ID 及路径（关键点序列）和估算时间差。

6) 在行为树（BT）中实现拦截节点
   - BT 子节点职责：
     - 读取 `/enemy_state`（或 `/armors`）；
     - 调用 planner（通过话题请求或直接调用 A* API）得到我方到候选点的路径/时间；
     - 根据规则选择拦截点；
     - 下发导航目标（使用现有 `send_goal` BT action 节点或直接调用 Nav2）；
     - 监控执行：到达/失败/被打断时重新计算或回退。
   - 可把该拦截节点封装为一个 SubTree，便于在行为树编辑器中替换策略。

7) 在 RViz 中高亮拦截点（可视化）
   - 利用 `GraphVisualizer` 增加一个单独的 Marker（颜色不同，且持续发布或周期更新）；
   - 话题建议：`/graph_intercept_point`（visualization_msgs/Marker），BT 节点在选定拦截点时发布该 Marker；
   - 这样操作人员能立即在 rviz 中看到当前拦截目标并做人工干预。

8) 测试流程（端到端）
   - 启动仿真/导航并确保 `/red_standard_robot1/map` 正常发布；
   - 运行 `graph_planner_node` 来响应路径查询；
   - 运行敌人发布节点（或利用仿真敌人）发送 `/enemy_pose`/`/armors`；
   - 运行拦截 BT（或手动调用 planner+visualizer），观察控制台输出与 rviz 可视化；
   - 调整参数（`min_clearance_cells`、`max_edge_length`、机器人速度估计）以获得期望行为。

9) 持久化配置建议
  - 把黑名单/白名单、速度参数、阈值写入参数 YAML（例如 `config/graph_planner_params.yaml`），在 launch 中通过 `--params-file` 加载；
  - 这样每次运行不需要在命令行重复传参，并在比赛中更容易调整与记录。

3) 用 params YAML（推荐用于比赛/持久化）

  - 建议把参数放入文件（例如 `config/enemy_state_params.yaml`），然后在 launch 文件或启动命令中传入：

    config/enemy_state_params.yaml 内容示例：

    ```yaml

    /enemy_state_node:

      ros__parameters:

        armors_topic: "/detector/armors"

        enemy_pose_topic: "/enemy_pose"

        enemy_state_topic: "/enemy_state"

        enemy_velocity_topic: "/enemy_velocity"

        frame_id: "map"

        window_size: 5

        smoothing_alpha: 0.6

        min_velocity_threshold: 0.02

    ```

  - 启动时加载：

    ros2 run rm_behavior_tree enemy_state_node --ros-args --params-file /absolute/path/to/config/enemy_state_params.yaml

  - 优点：可版本控制、比赛复现、与 launch 集成

如果你准备好了，我将下一步实现：**订阅 `/armors` 并输出平滑的 `/enemy_state`（pose + velocity）话题**，并把它与现有 `graph_planner_node` / A* 集成做一次端到端演示。是否现在开始？ 
