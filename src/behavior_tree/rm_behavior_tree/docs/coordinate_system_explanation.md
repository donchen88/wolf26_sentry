# 坐标系说明

## 关键点坐标

**关键点的 x, y 坐标已经是世界坐标（map坐标系）**，可以直接使用：

- ✅ **可以直接用于导航**：发送给 `navigate_to_pose` action
- ✅ **自瞄系统给的坐标也是这个坐标系**：可以直接比较和使用
- ✅ **不需要任何坐标转换**

例如：
```cpp
// 直接使用关键点坐标发送导航目标
geometry_msgs::msg::PoseStamped goal;
goal.pose.position.x = keypoint.x;  // 直接使用，无需转换
goal.pose.position.y = keypoint.y;  // 直接使用，无需转换
goal.pose.position.z = 0.0;
```

## 为什么需要坐标转换？

坐标转换**仅用于内部实现**，具体原因：

### 1. 地图数据存储方式

`nav_msgs::msg::OccupancyGrid` 的地图数据是按**栅格**存储的：
- 数据是一个一维数组：`map.data[0], map.data[1], ...`
- 要访问某个位置的障碍物信息，需要计算数组索引
- 索引计算需要栅格坐标：`index = y * width + x`

### 2. Raycast 算法需要栅格坐标

Bresenham 算法在**栅格空间**中工作：
- 需要遍历路径上的每个栅格
- 检查每个栅格是否被占用
- 栅格坐标是整数，便于遍历

### 3. 转换是内部实现细节

```cpp
// 内部实现（你不需要关心）
bool VisibilityChecker::isVisible(const KeyPoint& a, const KeyPoint& b, ...)
{
    // 1. 世界坐标 → 栅格坐标（仅用于访问地图数据）
    int x0, y0, x1, y1;
    map.worldToMap(a.x, a.y, x0, y0);  // 内部转换
    
    // 2. 在栅格空间中检查路径
    // ... Bresenham 算法 ...
    
    // 3. 返回结果（仍然是世界坐标的概念）
    return true/false;
}
```

## 总结

| 用途 | 坐标类型 | 是否需要转换 |
|------|---------|-------------|
| 关键点坐标（YAML） | 世界坐标 | ❌ 不需要 |
| 发送导航目标 | 世界坐标 | ❌ 不需要 |
| 自瞄系统坐标 | 世界坐标 | ❌ 不需要 |
| 可视化（rviz2） | 世界坐标 | ❌ 不需要 |
| **Raycast 检查（内部）** | **栅格坐标** | ✅ **需要（内部自动处理）** |

**结论**：坐标转换是**内部实现细节**，你只需要知道关键点坐标可以直接使用即可！

