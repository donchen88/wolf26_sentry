#!/bin/bash
# ROS2 网线直连配置脚本
# 使用方法: ./setup_direct_ethernet.sh [导航主机IP] [自瞄主机IP] [网卡名称]
# 示例: ./setup_direct_ethernet.sh 192.168.1.10 192.168.1.20 eth0

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== ROS2 网线直连配置脚本 ===${NC}"

# 检查参数
if [ $# -lt 2 ]; then
    echo -e "${RED}错误: 参数不足${NC}"
    echo "使用方法: $0 <本机IP> <对方IP> [网卡名称]"
    echo "示例: $0 192.168.1.10 192.168.1.20 eth0"
    exit 1
fi

LOCAL_IP=$1
REMOTE_IP=$2
INTERFACE=${3:-$(ip route | grep default | awk '{print $5}' | head -n1)}

if [ -z "$INTERFACE" ]; then
    echo -e "${RED}错误: 无法自动检测网卡名称${NC}"
    echo "请手动指定网卡名称，例如: $0 $LOCAL_IP $REMOTE_IP eth0"
    exit 1
fi

echo -e "${YELLOW}配置信息:${NC}"
echo "  本机IP: $LOCAL_IP"
echo "  对方IP: $REMOTE_IP"
echo "  网卡名称: $INTERFACE"
echo ""

# 检查是否为root
if [ "$EUID" -ne 0 ]; then 
    echo -e "${RED}错误: 请使用 sudo 运行此脚本${NC}"
    exit 1
fi

# 检测系统类型
if command -v nmcli &> /dev/null; then
    echo -e "${GREEN}检测到 NetworkManager，使用 nmcli 配置...${NC}"
    
    # 查找连接名称
    CONNECTION=$(nmcli -t -f NAME,DEVICE connection show | grep "$INTERFACE" | cut -d: -f1 | head -n1)
    
    if [ -z "$CONNECTION" ]; then
        CONNECTION="Wired connection 1"
        echo -e "${YELLOW}未找到现有连接，将创建新连接: $CONNECTION${NC}"
    fi
    
    # 配置静态IP
    nmcli connection modify "$CONNECTION" \
        ipv4.addresses "$LOCAL_IP/24" \
        ipv4.method manual \
        ipv4.gateway "$REMOTE_IP" \
        ipv4.dns "8.8.8.8,8.8.4.4"
    
    nmcli connection up "$CONNECTION"
    
    echo -e "${GREEN}✓ 网络配置完成${NC}"
    
elif [ -f /etc/netplan/ ]; then
    echo -e "${GREEN}检测到 netplan，使用 netplan 配置...${NC}"
    
    NETPLAN_FILE="/etc/netplan/01-ros2-direct-eth.yaml"
    
    cat > "$NETPLAN_FILE" << EOF
network:
  version: 2
  renderer: networkd
  ethernets:
    $INTERFACE:
      addresses:
        - $LOCAL_IP/24
      gateway4: $REMOTE_IP
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
EOF
    
    netplan apply
    echo -e "${GREEN}✓ 网络配置完成${NC}"
    
else
    echo -e "${RED}错误: 未检测到支持的网络管理工具${NC}"
    echo "请手动配置网络，或安装 NetworkManager 或 netplan"
    exit 1
fi

# 配置防火墙
echo -e "${YELLOW}配置防火墙...${NC}"
if command -v ufw &> /dev/null; then
    ufw allow 7400:7500/tcp
    ufw allow 7400:7500/udp
    echo -e "${GREEN}✓ 防火墙规则已添加${NC}"
else
    echo -e "${YELLOW}未检测到 ufw，请手动配置防火墙${NC}"
fi

# 测试网络连通性
echo -e "${YELLOW}测试网络连通性...${NC}"
if ping -c 3 -W 2 "$REMOTE_IP" &> /dev/null; then
    echo -e "${GREEN}✓ 网络连通正常${NC}"
else
    echo -e "${RED}✗ 无法连接到 $REMOTE_IP${NC}"
    echo "请检查:"
    echo "  1. 网线是否连接"
    echo "  2. IP地址配置是否正确"
    echo "  3. 网卡是否正常工作"
fi

# 配置ROS2环境变量
echo -e "${YELLOW}配置ROS2环境变量...${NC}"
BASHRC="$HOME/.bashrc"

if ! grep -q "ROS_DOMAIN_ID" "$BASHRC"; then
    cat >> "$BASHRC" << 'EOF'

# ROS2 跨主机通信配置
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
EOF
    echo -e "${GREEN}✓ ROS2环境变量已添加到 ~/.bashrc${NC}"
    echo -e "${YELLOW}请运行: source ~/.bashrc${NC}"
else
    echo -e "${YELLOW}ROS2环境变量已存在，跳过${NC}"
fi

echo ""
echo -e "${GREEN}=== 配置完成 ===${NC}"
echo ""
echo "下一步:"
echo "  1. 在另一台主机上运行相同的脚本（使用对应的IP地址）"
echo "  2. 运行: source ~/.bashrc"
echo "  3. 测试: ros2 topic list"
echo "  4. 在对方主机上应该能看到相同的话题列表"

