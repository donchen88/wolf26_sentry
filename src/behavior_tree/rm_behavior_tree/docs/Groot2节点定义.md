# Groot2 节点定义

## 新增的三个节点

在 Groot2 中，需要在 XML 配置文件的 `<TreeNodesModel>` 部分添加以下节点定义：

### 1. PubEnemyStatus（发布无敌敌方ID列表）

```xml
<Action ID="PubEnemyStatus"
        editable="true">
  <input_port name="topic_name"
              default="enemy_status">话题名称</input_port>
  <input_port name="invincible_enemy_ids"
              default="">无敌敌方ID列表，字符串格式，例如：1,3,5 或 [1,3,5]，空字符串表示所有敌人都可攻击</input_port>
</Action>
```

**说明**：
- `topic_name`：ROS2话题名称，默认值为 `"enemy_status"`
- `invincible_enemy_ids`：无敌敌方ID列表，支持字符串格式：
  - `"1,3,5"` - 逗号分隔
  - `"[1,3,5]"` - 带方括号
  - `""` - 空字符串表示所有敌人都可攻击

### 2. PubAutoaimTarget（发布瞄准目标ID列表）

```xml
<Action ID="PubAutoaimTarget"
        editable="true">
  <input_port name="topic_name"
              default="autoaim_target">话题名称</input_port>
  <input_port name="target_ids"
              default="">瞄准目标ID列表，字符串格式，例如：1,3,5 或 [1,3,5]，空字符串表示无目标</input_port>
</Action>
```

**说明**：
- `topic_name`：ROS2话题名称，默认值为 `"autoaim_target"`
- `target_ids`：瞄准目标ID列表，支持字符串格式：
  - `"1,3,5"` - 逗号分隔
  - `"[1,3,5]"` - 带方括号
  - `""` - 空字符串表示无目标

### 3. SubTargetPos（订阅目标位置，已存在）

```xml
<Action ID="SubTargetPos"
        editable="true">
  <input_port name="topic_name"
              default="auto_aim_target_pos"/>
  <input_port name="target_frame"
              default="map"/>
  <output_port name="armors"
               default="{armors}"/>
</Action>
```

**说明**：
- `topic_name`：ROS2话题名称，默认值为 `"auto_aim_target_pos"`
- `target_frame`：目标坐标系，默认值为 `"map"`
- `armors`：输出的Armors消息，会写入到黑板

## 完整示例（添加到 TreeNodesModel）

在你的 XML 配置文件中，找到 `<TreeNodesModel>` 标签，在 `</TreeNodesModel>` 之前添加：

```xml
<TreeNodesModel>
  <!-- 其他现有节点... -->
  
  <!-- 新增节点：发布无敌敌方ID列表 -->
  <Action ID="PubEnemyStatus"
          editable="true">
    <input_port name="topic_name"
                default="enemy_status">话题名称</input_port>
    <input_port name="invincible_enemy_ids"
                default="">无敌敌方ID列表，字符串格式，例如：1,3,5 或 [1,3,5]，空字符串表示所有敌人都可攻击</input_port>
  </Action>
  
  <!-- 新增节点：发布瞄准目标ID列表 -->
  <Action ID="PubAutoaimTarget"
          editable="true">
    <input_port name="topic_name"
                default="autoaim_target">话题名称</input_port>
    <input_port name="target_ids"
                default="">瞄准目标ID列表，字符串格式，例如：1,3,5 或 [1,3,5]，空字符串表示无目标</input_port>
  </Action>
  
  <!-- SubTargetPos 节点（如果还没有的话） -->
  <Action ID="SubTargetPos"
          editable="true">
    <input_port name="topic_name"
                default="auto_aim_target_pos"/>
    <input_port name="target_frame"
                default="map"/>
    <output_port name="armors"
                 default="{armors}"/>
  </Action>
</TreeNodesModel>
```

## 在 Groot2 中使用

添加节点定义后：

1. **重新加载配置文件**：在 Groot2 中重新加载 XML 文件
2. **在节点面板中找到新节点**：
   - `PubEnemyStatus` - 在 Action 节点列表中
   - `PubAutoaimTarget` - 在 Action 节点列表中
   - `SubTargetPos` - 在 Action 节点列表中（如果之前没有）
3. **拖拽到行为树**：直接拖拽到行为树中使用
4. **配置参数**：双击节点可以编辑输入端口的值

## 使用示例

在行为树中使用这些节点：

```xml
<BehaviorTree ID="maintree">
  <ReactiveSequence>
    <!-- 订阅自瞄发送的目标位置 -->
    <SubTargetPos topic_name="auto_aim_target_pos"
                  target_frame="map"
                  armors="{armors}"/>
    
    <!-- 发布无敌敌方ID列表 -->
    <PubEnemyStatus topic_name="enemy_status"
                    invincible_enemy_ids="1,3,5"/>
    
    <!-- 发布瞄准目标ID列表 -->
    <PubAutoaimTarget topic_name="autoaim_target"
                      target_ids="2,4"/>
    
    <!-- 其他逻辑... -->
  </ReactiveSequence>
</BehaviorTree>
```

## 注意事项

1. **节点ID必须匹配**：XML中的 `ID` 必须与代码中注册的节点名称完全一致
   - `PubEnemyStatus` ✅
   - `PubAutoaimTarget` ✅
   - `SubTargetPos` ✅

2. **端口名称必须匹配**：输入/输出端口的名称必须与代码中定义的端口名称一致

3. **默认值**：建议设置合理的默认值，方便在 Groot2 中直接使用

4. **描述信息**：在 `<input_port>` 标签内可以添加描述信息，帮助理解每个参数的用途



















































