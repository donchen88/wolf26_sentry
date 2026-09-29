# 2026_rmul.xml 行为树测试 - 快速参考

## 需要模拟发送的消息（一次性清单）

### 1. `/game_status` (robot_msgs::msg::GameStatus)
```bash
ros2 topic pub -r 10 /game_status robot_msgs/msg/GameStatus "{game_progress: 4, remain_time: 300}"
```
- **必需字段**: `game_progress=4`, `remain_time` 在 `[0, 420]` 范围内

### 2. `/robot_status` (robot_msgs::msg::RobotStatus)
```bash
# 正常状态
ros2 topic pub -r 10 /robot_status robot_msgs/msg/RobotStatus "{current_hp: 200, shooter_heat: 100, shooter_barrel_heat_limit: 0}"

# 状态不好（触发回家）
ros2 topic pub -r 10 /robot_status robot_msgs/msg/RobotStatus "{current_hp: 50, shooter_heat: 400, shooter_barrel_heat_limit: 0}"

# 被击打（血量下降，从 200 降到 150）
ros2 topic pub -r 10 /robot_status robot_msgs/msg/RobotStatus "{current_hp: 150, shooter_heat: 100, shooter_barrel_heat_limit: 0}"
```
- **必需字段**: 
  - `current_hp` >= 100 才能通过 IsStatusOK
  - `shooter_heat` <= 350 才能通过 IsStatusOK
  - `IsAttaked` 通过检测 `current_hp` 下降来判断被击打

### 3. `/auto_aim_target_pos` (std_msgs::msg::String)
```bash
# 有敌人
ros2 topic pub -r 10 /auto_aim_target_pos std_msgs/msg/String "{data: '5.0,3.0,0.5,1.0'}"

# 无敌人
ros2 topic pub -r 10 /auto_aim_target_pos std_msgs/msg/String "{data: ''}"
```
- **格式**: `"x,y,z,w"` (逗号分隔的4个浮点数)

### 4. `/red_standard_robot1/odometry` (nav_msgs::msg::Odometry)
```bash
ros2 topic pub -r 10 /red_standard_robot1/odometry nav_msgs/msg/Odometry "{header: {frame_id: 'map', stamp: {sec: 0, nanosec: 0}}, child_frame_id: 'base_link', pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### 5. Blackboard: `{@referee_rfidStatus}` (robot_msgs::msg::RfidStatus)
- **设置方式**: 通过行为树 blackboard 设置
- **关键字段**: `center_gain_point` (bool)
  - `true`: 在中心增益点（占点成功）
  - `false`: 不在中心增益点（需要发导航）

### 6. Action Server: `/red_standard_robot1/navigate_to_pose`
- **类型**: ROS 2 Action
- **用途**: 响应导航目标请求
- **注意**: 需要启动 mock action server 或使用真实导航系统

---

## 行为树发布的 Topic（用于验证）

- `/robot_state`: `1` (占点) 或 `3` (攻击)
- `/auto_aim_mode`: `0` (默认瞄敌人)
- `/chassis_mode`: `1` (加速底盘旋转，当检测到敌人或被击打时)

---

## 快速测试命令

```bash
# 使用测试脚本
./test_rmul_bt.sh 1  # 场景1: 正常占点+攻击
./test_rmul_bt.sh 2  # 场景2: 状态不好回家
./test_rmul_bt.sh 3  # 场景3: 被击打触发

# 或手动发布所有消息
ros2 topic pub -r 10 /game_status robot_msgs/msg/GameStatus "{game_progress: 4, remain_time: 300}" &
ros2 topic pub -r 10 /robot_status robot_msgs/msg/RobotStatus "{current_hp: 200, shooter_heat: 100, shooter_barrel_heat_limit: 0}" &
ros2 topic pub -r 10 /auto_aim_target_pos std_msgs/msg/String "{data: '5.0,3.0,0.5,1.0'}" &
ros2 topic pub -r 10 /red_standard_robot1/odometry nav_msgs/msg/Odometry "{header: {frame_id: 'map'}, child_frame_id: 'base_link', pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" &
```

---

## 测试流程检查点

1. ✅ 游戏状态正常 → 进入战斗逻辑
2. ✅ 机器人状态正常 → 通过 IsStatusOK
3. ✅ 未在中心点 → 发布 SendGoal 到中心点
4. ✅ 到达中心点（RFID=true） → 切换到攻击模式（robot_state=3）
5. ✅ 检测到敌人 → 发布 chassis_mode=1
6. ✅ 被击打（血量下降） → 发布 chassis_mode=1
7. ✅ 状态不好 → 发布回家 SendGoal
8. ✅ 状态恢复 → 重新进入占点流程




