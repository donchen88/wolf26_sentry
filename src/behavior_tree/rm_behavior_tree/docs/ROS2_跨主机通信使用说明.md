# ROS2 跨主机通信使用说明

## 一、工作原理

**是的，可以直接通过ROS2话题跨主机通信，无需串口！**

ROS2使用DDS（Data Distribution Service）作为底层通信机制，支持：
- ✅ **跨主机通信**：不同电脑上的ROS2节点可以自动发现并通信
- ✅ **无需串口**：通过以太网（TCP/UDP）直接通信
- ✅ **自动发现**：只要在同一网络，节点会自动发现对方
- ✅ **实时通信**：低延迟，适合实时控制

## 二、配置ROS2跨主机通信

### 2.1 网络配置

#### 方式一：通过路由器/交换机连接（推荐）

确保两台电脑在**同一局域网**（同一网段），例如：
- 导航主机：`192.168.1.10`
- 自瞄主机：`192.168.1.20`

#### 方式二：网线直连（对插）

如果两台电脑用网线**直接对插**，需要手动配置静态IP地址：

**步骤1：配置导航主机IP地址**

```bash
# 查看网卡名称（通常是 eth0 或 enp0s3 等）
ip addr show

# 方法1：使用 nmcli（NetworkManager）
sudo nmcli connection modify "Wired connection 1" \
    ipv4.addresses 192.168.1.10/24 \
    ipv4.method manual \
    ipv4.gateway 192.168.1.1
sudo nmcli connection up "Wired connection 1"

# 方法2：使用 netplan（Ubuntu 18.04+）
sudo nano /etc/netplan/01-netcfg.yaml
```

netplan 配置示例（导航主机）：
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:  # 替换为你的网卡名称
      addresses:
        - 192.168.1.10/24
      gateway4: 192.168.1.1  # 直连时可以不设置网关
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```

```bash
sudo netplan apply
```

**步骤2：配置自瞄主机IP地址**

```bash
# 使用 nmcli
sudo nmcli connection modify "Wired connection 1" \
    ipv4.addresses 192.168.1.20/24 \
    ipv4.method manual \
    ipv4.gateway 192.168.1.1
sudo nmcli connection up "Wired connection 1"
```

netplan 配置示例（自瞄主机）：
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:  # 替换为你的网卡名称
      addresses:
        - 192.168.1.20/24
      gateway4: 192.168.1.1  # 直连时可以不设置网关
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```

**步骤3：验证网络连通性**

```bash
# 在导航主机上 ping 自瞄主机
ping 192.168.1.20

# 在自瞄主机上 ping 导航主机
ping 192.168.1.10
```

**注意事项**：
- 网线直连需要使用**交叉网线**（或现代网卡支持自动翻转，普通网线也可以）
- IP地址必须在同一网段（例如都是 `192.168.1.x`）
- 子网掩码通常是 `/24`（即 `255.255.255.0`）
- 直连时可以不设置网关，但建议设置一个虚拟网关地址

### 2.2 环境变量配置

**在导航主机上**（发布消息）：
```bash
export ROS_DOMAIN_ID=0  # 两台电脑必须相同
export ROS_DISCOVERY_SERVER=  # 如果使用发现服务器，否则留空

# 可选：指定ROS2使用的网络接口（如果有多张网卡）
export ROS_LOCALHOST_ONLY=0  # 0表示允许跨主机通信
```

**在自瞄主机上**（订阅消息）：
```bash
export ROS_DOMAIN_ID=0  # 必须与导航主机相同
export ROS_DISCOVERY_SERVER=  # 如果使用发现服务器，否则留空
export ROS_LOCALHOST_ONLY=0
```

**永久设置环境变量**（推荐）：

在 `~/.bashrc` 或 `~/.bash_profile` 中添加：
```bash
# ROS2 跨主机通信配置
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

然后执行：
```bash
source ~/.bashrc
```

### 2.3 防火墙设置

确保两台电脑的防火墙允许ROS2通信端口（通常为7400-7500范围）：
```bash
# Ubuntu示例
sudo ufw allow 7400:7500/tcp
sudo ufw allow 7400:7500/udp

# 或者临时关闭防火墙测试（不推荐用于生产环境）
sudo ufw disable
```

### 2.4 网线直连的特殊配置

如果使用网线直连，可能还需要：

**方法1：使用 ROS_DISCOVERY_SERVER（推荐）**

如果自动发现有问题，可以手动指定发现服务器：

```bash
# 在导航主机上启动发现服务器（使用导航主机的IP）
export ROS_DISCOVERY_SERVER=192.168.1.10:11811

# 在自瞄主机上连接到发现服务器
export ROS_DISCOVERY_SERVER=192.168.1.10:11811
```

**方法2：使用 RMW_IMPLEMENTATION 环境变量**

某些情况下，切换RMW实现可能有助于连接：

```bash
# 使用 FastRTPS（默认）
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 或使用 Cyclone DDS
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
```

## 三、行为树配置示例

### 3.1 在行为树开头持续发布消息

这些节点可以放在行为树开头，**持续运行**并发布消息：

```xml
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4">
  <BehaviorTree ID="maintree">
    <ReactiveSequence>
      <!-- 1. 订阅自瞄发送的目标位置（自瞄 → 导航） -->
      <SubTargetPos topic_name="auto_aim_target_pos"
                    target_frame="map"
                    armors="{armors}"/>
      
      <!-- 2. 发布无敌敌方ID列表（导航 → 自瞄） -->
      <PubEnemyStatus topic_name="enemy_status"
                      invincible_enemy_ids="{invincible_ids}"/>
      
      <!-- 3. 发布瞄准目标ID列表（导航 → 自瞄） -->
      <PubAutoaimTarget topic_name="autoaim_target"
                        target_ids="{target_ids}"/>
      
      <!-- 其他行为树逻辑... -->
      <SubTree ID="normal"/>
    </ReactiveSequence>
  </BehaviorTree>
</root>
```

### 3.2 动态设置ID列表（从黑板读取）

如果ID列表需要动态计算，可以从黑板读取：

```xml
<BehaviorTree ID="maintree">
  <ReactiveSequence>
    <!-- 计算无敌敌方ID列表（示例） -->
    <Sequence>
      <!-- 这里可以添加计算逻辑，将结果写入黑板 -->
      <SetBlackboard output_key="invincible_ids" value="[1, 3, 5]"/>
    </Sequence>
    
    <!-- 发布无敌敌方ID列表 -->
    <PubEnemyStatus topic_name="enemy_status"
                    invincible_enemy_ids="{invincible_ids}"/>
    
    <!-- 发布瞄准目标ID列表 -->
    <PubAutoaimTarget topic_name="autoaim_target"
                      target_ids="{target_ids}"/>
  </ReactiveSequence>
</BehaviorTree>
```

### 3.3 条件发布（根据状态决定）

```xml
<BehaviorTree ID="maintree">
  <ReactiveSequence>
    <SubTargetPos topic_name="auto_aim_target_pos"
                  target_frame="map"
                  armors="{armors}"/>
    
    <Fallback>
      <!-- 如果检测到敌人，发布瞄准目标 -->
      <Sequence>
        <IsDetectEnemy message="{armors}"/>
        <PubAutoaimTarget topic_name="autoaim_target"
                          target_ids="[1, 2, 3]"/>
      </Sequence>
      
      <!-- 否则发布空列表 -->
      <PubAutoaimTarget topic_name="autoaim_target"
                        target_ids="[]"/>
    </Fallback>
  </ReactiveSequence>
</BehaviorTree>
```

## 四、节点详细说明

### 4.1 PubEnemyStatus（发布无敌敌方ID列表）

**功能**：向自瞄系统发送无敌敌方ID列表，自瞄会跳过这些敌人

**输入端口**：
- `topic_name`（字符串，默认：`"enemy_status"`）：话题名称
- `invincible_enemy_ids`（字符串，默认：空字符串）：无敌敌方ID列表，支持格式：
  - `"1,3,5"` - 逗号分隔
  - `"[1,3,5]"` - 带方括号
  - `""` - 空字符串表示空列表

**输出**：
- 发布 `sp_msgs::msg::EnemyStatusMsg` 消息到指定话题
- 自动设置时间戳

**示例**：
```xml
<!-- 发布ID为1、3、5的敌人为无敌（字符串格式） -->
<PubEnemyStatus topic_name="enemy_status"
                invincible_enemy_ids="1,3,5"/>

<!-- 或者使用方括号格式 -->
<PubEnemyStatus topic_name="enemy_status"
                invincible_enemy_ids="[1,3,5]"/>

<!-- 发布空列表（所有敌人都可攻击） -->
<PubEnemyStatus topic_name="enemy_status"
                invincible_enemy_ids=""/>
```

### 4.2 PubAutoaimTarget（发布瞄准目标ID列表）

**功能**：向自瞄系统发送优先瞄准的目标ID列表

**输入端口**：
- `topic_name`（字符串，默认：`"autoaim_target"`）：话题名称
- `target_ids`（字符串，默认：空字符串）：目标ID列表，支持格式：
  - `"1,3,5"` - 逗号分隔
  - `"[1,3,5]"` - 带方括号
  - `""` - 空字符串表示无目标

**输出**：
- 发布 `sp_msgs::msg::AutoaimTargetMsg` 消息到指定话题
- 自动设置时间戳

**示例**：
```xml
<!-- 优先瞄准ID为1、3、5的目标（字符串格式） -->
<PubAutoaimTarget topic_name="autoaim_target"
                  target_ids="1,3,5"/>

<!-- 或者使用方括号格式 -->
<PubAutoaimTarget topic_name="autoaim_target"
                  target_ids="[1,3,5]"/>

<!-- 无目标（空字符串） -->
<PubAutoaimTarget topic_name="autoaim_target"
                  target_ids=""/>
```

### 4.3 SubTargetPos（订阅目标位置）

**功能**：从自瞄系统接收目标位置信息

**输入端口**：
- `topic_name`（字符串，默认：`"auto_aim_target_pos"`）：话题名称
- `target_frame`（字符串，默认：`"map"`）：目标坐标系

**输出端口**：
- `armors`（`auto_aim_interfaces::msg::Armors`）：转换后的armors消息

**说明**：
- 订阅 `std_msgs::msg::String` 消息（格式：`"x,y,z,w"`）
- 自动解析并转换为 `Armors` 消息
- 可以放在行为树开头持续运行

## 五、自瞄端接收代码示例

自瞄端需要订阅这两个话题：

### 5.1 订阅 enemy_status

```cpp
// 自瞄端代码示例
void Subscribe2Nav::enemy_status_callback(
    const sp_msgs::msg::EnemyStatusMsg::SharedPtr msg) 
{
    std::vector<int8_t> ids = msg->invincible_enemy_ids;
    
    // 检查当前敌人是否在无敌列表中
    if (std::find(ids.begin(), ids.end(), current_enemy_id) != ids.end()) {
        // 该敌人无敌，跳过攻击逻辑
        RCLCPP_INFO(this->get_logger(), "Enemy %d is invincible, skipping", current_enemy_id);
        return;
    } else {
        // 进行普通攻击逻辑
        // ...
    }
}

// 创建订阅者
enemy_status_sub_ = this->create_subscription<sp_msgs::msg::EnemyStatusMsg>(
    "enemy_status", 10,
    std::bind(&Subscribe2Nav::enemy_status_callback, this, std::placeholders::_1));
```

### 5.2 订阅 autoaim_target

```cpp
// 自瞄端代码示例
void Subscribe2Nav::autoaim_target_callback(
    const sp_msgs::msg::AutoaimTargetMsg::SharedPtr msg) 
{
    std::vector<int8_t> target_ids = msg->target_ids;
    
    if (target_ids.empty()) {
        // 无目标，停止瞄准
        RCLCPP_INFO(this->get_logger(), "No target specified");
        return;
    }
    
    // 优先瞄准列表中的目标
    // 例如：优先瞄准ID=1、3、5的目标
    for (int8_t id : target_ids) {
        // 查找并瞄准该ID的敌人
        // ...
    }
}

// 创建订阅者
autoaim_target_sub_ = this->create_subscription<sp_msgs::msg::AutoaimTargetMsg>(
    "autoaim_target", 10,
    std::bind(&Subscribe2Nav::autoaim_target_callback, this, std::placeholders::_1));
```

## 六、验证通信

### 6.1 检查话题是否可见

**在导航主机上**：
```bash
ros2 topic list
# 应该能看到：
# /enemy_status
# /autoaim_target
# /auto_aim_target_pos
```

**在自瞄主机上**：
```bash
ros2 topic list
# 应该能看到相同的话题（如果网络配置正确）
```

### 6.2 查看消息内容

**在导航主机上发布测试消息**：
```bash
# 查看消息类型
ros2 interface show sp_msgs/msg/EnemyStatusMsg

# 手动发布测试消息
ros2 topic pub /enemy_status sp_msgs/msg/EnemyStatusMsg \
  "{timestamp: {sec: 0, nanosec: 0}, invincible_enemy_ids: [1, 3, 5]}"
```

**在自瞄主机上查看**：
```bash
# 监听消息
ros2 topic echo /enemy_status
```

### 6.3 检查节点连接

```bash
# 查看话题信息
ros2 topic info /enemy_status

# 应该显示：
# Type: sp_msgs/msg/EnemyStatusMsg
# Publisher count: 1  (导航主机)
# Subscription count: 1  (自瞄主机)
```

## 七、常见问题

### Q1: 自瞄收不到消息？

**网线直连的情况**：
- ✅ 检查IP地址是否配置正确（`ip addr show`）
- ✅ 检查两台电脑是否能互相ping通（`ping <对方IP>`）
- ✅ 检查 `ROS_DOMAIN_ID` 是否相同
- ✅ 检查防火墙设置
- ✅ 检查话题名称是否一致
- ✅ 尝试使用 `ROS_DISCOVERY_SERVER` 手动指定发现服务器
- ✅ 检查网线是否正常（尝试更换网线）

**通过路由器连接的情况**：
- ✅ 检查 `ROS_DOMAIN_ID` 是否相同
- ✅ 检查网络是否连通（`ping` 测试）
- ✅ 检查防火墙设置
- ✅ 检查话题名称是否一致
- ✅ 确认两台电脑在同一网段

### Q2: 消息延迟高？
- ✅ 确保在同一局域网（不要跨路由器）
- ✅ 检查网络带宽
- ✅ 考虑使用ROS2的QoS设置优化

### Q3: 如何设置队列大小？
当前实现使用默认队列大小（1）。如需修改，可以在基类中调整，或修改发布者创建代码。

## 八、总结

✅ **可以直接跨主机通信**：通过ROS2话题，无需串口  
✅ **可以放在行为树开头**：使用 `ReactiveSequence` 持续运行  
✅ **自动发现和连接**：只要网络配置正确，节点会自动连接  
✅ **实时通信**：低延迟，适合实时控制场景  

### 配置要点：

**网线直连**：
1. ✅ 手动配置静态IP地址（同一网段）
2. ✅ 验证网络连通性（`ping` 测试）
3. ✅ `ROS_DOMAIN_ID` 相同
4. ✅ 防火墙允许ROS2通信
5. ✅ 话题名称一致
6. ✅ 如遇问题，可尝试使用 `ROS_DISCOVERY_SERVER`

**通过路由器连接**：
1. ✅ 两台电脑在同一局域网
2. ✅ `ROS_DOMAIN_ID` 相同
3. ✅ 防火墙允许ROS2通信
4. ✅ 话题名称一致

然后就可以像使用本地话题一样使用跨主机通信了！

### 快速检查清单

```bash
# 1. 检查IP地址
ip addr show

# 2. 检查网络连通性
ping <对方IP>

# 3. 检查ROS2环境变量
echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY

# 4. 检查话题列表（在对方主机上应该能看到相同的话题）
ros2 topic list

# 5. 检查话题连接
ros2 topic info /enemy_status
```

### 使用配置脚本（网线直连）

我们提供了便捷的配置脚本：

**在导航主机上**：
```bash
cd /home/mwk/ros/ros_ws/src/rm_behavior_tree/rm_behavior_tree/scripts
sudo ./setup_direct_ethernet.sh 192.168.1.10 192.168.1.20 eth0
```

**在自瞄主机上**：
```bash
cd /home/mwk/ros/ros_ws/src/rm_behavior_tree/rm_behavior_tree/scripts
sudo ./setup_direct_ethernet.sh 192.168.1.20 192.168.1.10 eth0
```

**测试连接**：
```bash
./test_ros2_connection.sh <对方IP>
```

脚本会自动：
- ✅ 配置静态IP地址
- ✅ 设置防火墙规则
- ✅ 配置ROS2环境变量
- ✅ 测试网络连通性

