# 测试 2026_rmul.xml 行为树所需模拟的消息清单

## 必需订阅的 Topic（需要持续发布）

### 1. `/game_status` 
- **消息类型**: `robot_msgs::msg::GameStatus`
- **用途**: 游戏状态，用于 `IsGameTime` 判断比赛时间窗口
- **关键字段**:
  - `game_progress` (uint8): 游戏进度，需要设置为 `4`（对应比赛阶段）
  - `remain_time` (uint16): 剩余时间（秒），需要在 `[0, 420]` 范围内才能进入战斗逻辑
- **测试建议**:
  ```bash
  # 示例：设置 game_progress=4, remain_time=300（在战斗时间窗口内）
  ros2 topic pub /game_status robot_msgs/msg/GameStatus "{game_progress: 4, remain_time: 300}"
  ```

### 2. `/robot_status`
- **消息类型**: `robot_msgs::msg::RobotStatus`
- **用途**: 机器人状态，用于 `IsStatusOK`（血量/热量检查）和 `IsAttaked`（被击打检测）
- **关键字段**:
  - `current_hp` (uint16): 当前血量，需要 >= 100 才能通过 `IsStatusOK`（hp_threshold=100）
  - `shooter_heat` (uint16): 发射器热量，需要 <= 350 才能通过 `IsStatusOK`（heat_threshold=350）
  - `shooter_barrel_heat_limit` (uint16): 枪管热量限制，需要 >= 0（barrel_heat_limit_threshold=0）
  - 被击打状态：`IsAttaked` 通过检测 `current_hp` **下降**来判断被击打（比较当前血量和上一次血量）
- **测试建议**:
  ```bash
  # 正常状态（血量充足、热量正常）
  ros2 topic pub /robot_status robot_msgs/msg/RobotStatus "{current_hp: 200, shooter_heat: 100, shooter_barrel_heat_limit: 0}"
  
  # 状态不好（血量低或热量高，会触发回家逻辑）
  ros2 topic pub /robot_status robot_msgs/msg/RobotStatus "{current_hp: 50, shooter_heat: 400, shooter_barrel_heat_limit: 0}"
  ```

### 3. `/auto_aim_target_pos`
- **消息类型**: `std_msgs::msg::String`
- **用途**: 目标位置（敌人位置），用于 `IsDetectEnemy` 检测敌人
- **数据格式**: `"x,y,z,w"`（逗号分隔的4个浮点数）
  - `x, y, z`: 目标位置坐标（单位：米）
  - `w`: 额外参数（可能用于朝向等）
- **测试建议**:
  ```bash
  # 有敌人时：发送目标位置（例如在 (5.0, 3.0, 0.5) 处有敌人）
  ros2 topic pub /auto_aim_target_pos std_msgs/msg/String "{data: '5.0,3.0,0.5,1.0'}"
  
  # 无敌人时：发送空字符串或无效格式（IsDetectEnemy 会返回失败）
  ros2 topic pub /auto_aim_target_pos std_msgs/msg/String "{data: ''}"
  ```

### 4. `/red_standard_robot1/odometry`
- **消息类型**: `nav_msgs::msg::Odometry`
- **用途**: 机器人当前位置，用于 `GetCurrentLocation` 和 `IsDetectEnemy` 的距离计算
- **关键字段**:
  - `pose.pose.position`: 机器人位置（x, y, z）
  - `pose.pose.orientation`: 机器人朝向（四元数）
  - `header.frame_id`: 坐标系，建议设置为 `"map"` 或 `"odom"`
  - `child_frame_id`: 子坐标系，通常为 `"base_link"`
- **测试建议**:
  ```bash
  # 示例：机器人在 (0, 0, 0) 位置
  ros2 topic pub /red_standard_robot1/odometry nav_msgs/msg/Odometry "{header: {frame_id: 'map'}, child_frame_id: 'base_link', pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
  ```

## 需要设置到 Blackboard 的数据

### 5. `{@referee_rfidStatus}` (Blackboard Key)
- **消息类型**: `robot_msgs::msg::RfidStatus`
- **用途**: RFID 状态，用于 `IsRfidDetected` 判断是否在中心增益点
- **关键字段**:
  - `base_gain_point` (bool): 己方基地增益点
  - `friendly_fortress_gain_point` (bool): 己方堡垒增益点
  - `center_gain_point` (bool): **中心增益点（RMUL）**，需要设置为 `true` 才能通过占点检测
- **设置方式**: 
  - 需要通过行为树的 blackboard 设置，或者有一个订阅 referee 系统并写入 blackboard 的节点
  - 如果使用 Groot 或行为树运行时，可以通过 blackboard 接口设置
- **测试建议**:
  ```cpp
  // 在 C++ 代码中设置 blackboard（示例）
  // blackboard->set("referee_rfidStatus", rfid_status_msg);
  
  // 或者通过 ROS 2 服务/参数设置（如果实现了相应接口）
  ```

## 行为树发布的 Topic（用于验证输出）

### 6. `/robot_state` (行为树发布)
- **消息类型**: 根据你的实现（可能是 `std_msgs::msg::UInt8` 或自定义类型）
- **预期值**:
  - `1`: 占点/赶路状态
  - `3`: 攻击模式

### 7. `/auto_aim_mode` (行为树发布)
- **消息类型**: 根据你的实现
- **预期值**: `0`（默认瞄敌人）

### 8. `/chassis_mode` (行为树发布)
- **消息类型**: 根据你的实现
- **预期值**: `1`（加速底盘旋转）- 当检测到敌人或被击打时发布

### 9. `/red_standard_robot1/navigate_to_pose` (Action Server)
- **类型**: ROS 2 Action
- **用途**: 导航目标点
- **测试建议**: 需要启动一个 mock 的 navigate_to_pose action server，或者使用真实的导航系统

---

## 完整测试流程建议

### 场景 1: 正常占点 → 攻击流程
1. **设置游戏状态**（比赛时间内）:
   ```bash
   ros2 topic pub -r 10 /game_status robot_msgs/msg/GameStatus "{game_progress: 4, remain_time: 300}"
   ```

2. **设置机器人状态**（正常）:
   ```bash
   ros2 topic pub -r 10 /robot_status robot_msgs/msg/RobotStatus "{current_hp: 200, shooter_heat: 100, shooter_barrel_heat_limit: 0}"
   ```

3. **设置机器人位置**（不在中心点）:
   ```bash
   ros2 topic pub -r 10 /red_standard_robot1/odometry nav_msgs/msg/Odometry "{header: {frame_id: 'map'}, child_frame_id: 'base_link', pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
   ```

4. **设置 RFID 状态**（未在中心点）:
   - 通过 blackboard 设置 `referee_rfidStatus.center_gain_point = false`
   - **预期**: 行为树应该发布 `SendGoal` 到中心点

5. **设置 RFID 状态**（到达中心点）:
   - 通过 blackboard 设置 `referee_rfidStatus.center_gain_point = true`
   - **预期**: 行为树应该切换到 `robot_state=3`，进入攻击模式

6. **设置敌人位置**（有敌人）:
   ```bash
   ros2 topic pub -r 10 /auto_aim_target_pos std_msgs/msg/String "{data: '5.0,3.0,0.5,1.0'}"
   ```
   - **预期**: 行为树应该发布 `chassis_mode=1`

### 场景 2: 状态不好 → 回家 → 恢复 → 重新占点
1. **设置机器人状态**（状态不好）:
   ```bash
   ros2 topic pub -r 10 /robot_status robot_msgs/msg/RobotStatus "{current_hp: 50, shooter_heat: 400, shooter_barrel_heat_limit: 0}"
   ```
   - **预期**: 行为树应该发布 `robot_state=1` 和 `SendGoal` 到 `(0,0,0)` 回家点

2. **恢复状态**（状态变好）:
   ```bash
   ros2 topic pub -r 10 /robot_status robot_msgs/msg/RobotStatus "{current_hp: 200, shooter_heat: 100, shooter_barrel_heat_limit: 0}"
   ```
   - **预期**: 行为树应该重新进入占点流程

### 场景 3: 被击打触发底盘加速
1. **初始状态**（血量正常）:
   ```bash
   ros2 topic pub -r 10 /robot_status robot_msgs/msg/RobotStatus "{current_hp: 200, shooter_heat: 100, shooter_barrel_heat_limit: 0}"
   ```

2. **模拟被击打**（血量下降）:
   ```bash
   ros2 topic pub -r 10 /robot_status robot_msgs/msg/RobotStatus "{current_hp: 150, shooter_heat: 100, shooter_barrel_heat_limit: 0}"
   ```
   - **预期**: `IsAttaked` 检测到血量从 200 降到 150，返回 SUCCESS，行为树应该发布 `chassis_mode=1`

---

## 快速测试脚本（bash）

可以创建一个脚本循环发布这些消息：

```bash
#!/bin/bash
# test_rmul_bt.sh

# 1. 游戏状态（比赛时间内）
ros2 topic pub -r 10 /game_status robot_msgs/msg/GameStatus "{game_progress: 4, remain_time: 300}" &

# 2. 机器人状态（正常）
ros2 topic pub -r 10 /robot_status robot_msgs/msg/RobotStatus "{current_hp: 200, shooter_heat: 100, shooter_barrel_heat_limit: 0}" &

# 3. 机器人位置
ros2 topic pub -r 10 /red_standard_robot1/odometry nav_msgs/msg/Odometry "{header: {frame_id: 'map'}, child_frame_id: 'base_link', pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" &

# 4. 敌人位置（有敌人）
ros2 topic pub -r 10 /auto_aim_target_pos std_msgs/msg/String "{data: '5.0,3.0,0.5,1.0'}" &

echo "所有测试消息已发布，按 Ctrl+C 停止"
wait
```

---

## 注意事项

1. **消息频率**: 建议以 10Hz 或更高频率发布，确保行为树能及时响应
2. **RFID 状态**: `referee_rfidStatus` 需要通过 blackboard 设置，可能需要额外的节点或服务
3. **Action Server**: `/red_standard_robot1/navigate_to_pose` 需要有一个 action server 响应，否则 `SendGoal` 节点可能会失败
4. **坐标系**: 确保所有坐标都在同一坐标系（建议使用 `map` 坐标系）
5. **消息类型**: 如果实际的消息类型与上述不同，需要根据你的 `robot_msgs` 包定义调整

