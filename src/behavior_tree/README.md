# rm_behavior_tree

> 本仓库为 [RM2024_SMBU_auto_sentry_ws](https://gitee.com/SMBU-POLARBEAR/RM2024_SMBU_auto_sentry_ws) 的子模块，**与父仓库的其他模块存在依赖关系**

基于 BehaviorTree.CPP 的 Robomaster 哨兵决策树，与导航模块和自瞄模块基于 ROS2 topic 和 action 进行信息传递，可在 [仿真环境](https://gitee.com/SMBU-POLARBEAR/pb_rmsimulation) 中进行决策预设开发，并部署到实体机器人上运行。

## 文件结构

- BehaviorTree.ROS2

    forcked from [BehaviorTree/BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2), provides a standard way to implement:

  - Action clients
  - Service Clients
  - Topic Subscribers
  - Topic Publishers

- rm_behavior_tree

    Robomaster 哨兵决策树部分

- rm_decision_interfaces

    对接裁判系统的自定义 ROS 消息类型

## 环境配置

当前开发环境为 Ubuntu22.04, ROS2 humble, BehaviorTree.CPP 4.5

1. 安装依赖

    ```sh
    sudo apt install ros-humble-behaviortree-cpp
    ```

2. 克隆仓库

    ```sh
    git clone https://gitee.com/SMBU-POLARBEAR/rm_behavior_tree.git
    cd rm_behavior_tree
    ```

3. 编译

    ```sh
    colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    ```

## 使用方法

1. 开启 [虚拟裁判系统话题发布](./rm_decision_interfaces/publish_script.sh)

    ```sh
    ./rm_decision_ws/rm_decision_interfaces/publish_script.sh
    ```

2. 启动行为树

    ```sh
    ros2 launch rm_behavior_tree rm_behavior_tree.launch.py \
    style:=test \
    use_sim_time:=True
    ```

    `style` 参数与决策树预设文件名一致，详见下文。

## 当前的行为树预设

- `attack_left`

    自身状态良好时，向敌方左侧进攻；若队友平均血量少于敌方平均血量，回退到中心增益点；血量低时，回退补给区；撤退过程中若被攻击，再次根据状态判断是否打断撤退任务；被攻击时，可原地走位；

- `attack_right`

    同上，但向右侧进攻。

- `retreat_attack_left`

    同上，自身状态良好时，向左侧进攻...；  
    添加特性：比赛最后1:30，固定回退到己方基地前侧墙边。

- `protect_supply`

    保守策略，比赛开始后，优先保护（挡子弹）我方补给区；一定时间后撤退到我方基地左侧苟着。保留基本加血和走位功能。

- `rmuc_01`

    比赛开始后去飞坡区堵路，如果堵路时被攻击会小范围走位。如果哨兵血量到达阈值会优先回去补血。优先级最低的是前哨站血量少于设定中会直接回家。

## 使用 Groot 可视化行为树

1. 下载 [Groot Linux installer](https://www.behaviortree.dev/groot)

2. 安装 Groot

    ```sh
    chmod +x Groot2-*-linux-installer.run
    ./Groot2-*-linux-installer.run
    ```

3. 运行 Groot

    ```sh
    cd ~/Groot2/bin
    ./groot2
    ```

4. 在 Groot 中打开 [Project.btproj](./rm_behavior_tree/config/Project.btproj)
