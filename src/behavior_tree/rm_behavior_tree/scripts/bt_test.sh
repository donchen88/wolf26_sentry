#!/usr/bin/env bash

set -e

cleanup() {
  echo ""
  echo ">>> Cleaning up BT test..."
  kill 0
}

trap cleanup SIGINT SIGTERM

echo "====start===="

source install/setup.bash

echo "====start BT===="
ros2 launch rm_behavior_tree rm_behavior_tree.launch.py \
  style:=2026_rmul use_sim_time:=True &

sleep 2   # 给 BT 一点时间起来（很重要）

echo "[2] Publish robot_status (once)"
ros2 topic pub -r 1 /robot_status robot_msgs/msg/RobotStatus "{
  current_hp: 50,
  shooter_heat: 200,
  shooter_barrel_heat_limit: 300
}"

echo "[3] Publish game_status (once)"
ros2 topic pub -r 1 /game_status robot_msgs/msg/GameStatus "{
  game_progress: 4,
  stage_remain_time: 420
}"

wait

