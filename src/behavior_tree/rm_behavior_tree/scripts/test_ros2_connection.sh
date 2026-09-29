#!/bin/bash
# ROS2 跨主机连接测试脚本
# 使用方法: ./test_ros2_connection.sh [对方IP]

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== ROS2 跨主机连接测试 ===${NC}"
echo ""

# 检查ROS2环境
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo -e "${YELLOW}警告: ROS_DOMAIN_ID 未设置，使用默认值 0${NC}"
    export ROS_DOMAIN_ID=0
fi

echo -e "${GREEN}当前ROS2配置:${NC}"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY:-未设置}"
echo ""

# 检查网络
echo -e "${YELLOW}1. 检查网络配置...${NC}"
LOCAL_IP=$(hostname -I | awk '{print $1}')
echo "  本机IP: $LOCAL_IP"

if [ $# -ge 1 ]; then
    REMOTE_IP=$1
    echo "  对方IP: $REMOTE_IP"
    
    if ping -c 3 -W 2 "$REMOTE_IP" &> /dev/null; then
        echo -e "  ${GREEN}✓ 网络连通正常${NC}"
    else
        echo -e "  ${RED}✗ 无法连接到 $REMOTE_IP${NC}"
        exit 1
    fi
else
    echo -e "  ${YELLOW}未指定对方IP，跳过网络测试${NC}"
fi

# 检查ROS2话题
echo ""
echo -e "${YELLOW}2. 检查ROS2话题...${NC}"
if command -v ros2 &> /dev/null; then
    echo "  可用话题列表:"
    ros2 topic list | sed 's/^/    /'
    
    echo ""
    echo -e "${YELLOW}3. 检查关键话题...${NC}"
    
    TOPICS=("enemy_status" "autoaim_target" "auto_aim_target_pos")
    for topic in "${TOPICS[@]}"; do
        if ros2 topic list | grep -q "^/$topic$"; then
            echo -e "  ${GREEN}✓ /$topic 存在${NC}"
            
            # 显示话题信息
            INFO=$(ros2 topic info "/$topic" 2>&1)
            PUB_COUNT=$(echo "$INFO" | grep "Publisher count" | awk '{print $3}')
            SUB_COUNT=$(echo "$INFO" | grep "Subscription count" | awk '{print $3}')
            
            echo "    发布者数量: $PUB_COUNT"
            echo "    订阅者数量: $SUB_COUNT"
        else
            echo -e "  ${YELLOW}○ /$topic 不存在（可能还未创建）${NC}"
        fi
    done
else
    echo -e "${RED}错误: 未找到 ros2 命令${NC}"
    echo "请确保已安装ROS2并source了setup.bash"
    exit 1
fi

# 测试消息发布（可选）
echo ""
echo -e "${YELLOW}4. 测试消息发布（可选）...${NC}"
read -p "是否测试发布消息到 /enemy_status? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "发布测试消息..."
    ros2 topic pub --once /enemy_status sp_msgs/msg/EnemyStatusMsg \
        "{timestamp: {sec: 0, nanosec: 0}, invincible_enemy_ids: [1, 2, 3]}"
    echo -e "${GREEN}✓ 消息已发布${NC}"
fi

echo ""
echo -e "${GREEN}=== 测试完成 ===${NC}"
echo ""
echo "如果对方主机上能看到相同的话题，说明连接成功！"
echo "在对方主机上运行: ros2 topic echo /enemy_status"

