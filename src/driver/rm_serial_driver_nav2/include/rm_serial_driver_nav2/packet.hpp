// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver_nav2
{
struct ReceivePacket   //接收
{
  uint8_t start = 0x06;

  uint8_t rune_count;           // 已开小符次数：0次，1次，2次
  uint16_t enemy_outpost_hp;    // 敌方前哨站血量
  uint16_t outpost_hp;  // 前哨站血量（己方）
  uint16_t base_hp;     // 基地血量（己方）
  uint8_t game_progress;         // 比赛阶段
  uint16_t game_progress_remain;  // 该比赛阶段剩余时间
  uint16_t sentry_hp;                       // 实时血量 RMUC初始 400
  uint16_t bullet_remain;                    // 实时剩余弹药量 RMUC联盟赛初始 400
  uint16_t shooter_barrel_heat_limit;       // 实时枪口热量（电控发送的实时热量值）
  uint8_t team_color; //  1 True红方， 0False蓝方
  uint8_t remote_control_status;            // 遥控器模式：1=正常，2=保守，3=激进
  uint8_t base_gain_point_rfid;             // RFID 己方基地增益点：0=未检测到, 1=检测到
  uint8_t friendly_fortress_gain_point_rfid; // RFID 己方堡垒增益点：0=未检测到, 1=检测到
  uint8_t center_gain_point_rfid;           // RFID 中心增益点：0=未检测到, 1=检测到
  uint8_t auto_aim_status;                  // 自瞄状态：0=正常，1=不正常
   // 轮速数据 (m/s)
  float wheel_front_left;
  float wheel_front_right;
  float wheel_back_left;
  float wheel_back_right;
  // 陀螺仪数据 (弧度)
  float gyro_pitch;  // 俯仰角
  float gyro_roll;  // 横滚角
  float gimbal_target_x;
  float gimbal_target_y;
  uint16_t chassis_energy;      // 底盘剩余能量 (J)

  uint8_t end = 0x09;
} __attribute__((packed));

struct SendPacket    //发送
{
  uint8_t start = 0x05;
  float chassis_lx = 0.f;       // 底盘 x 轴速度
  float chassis_ly = 0.f;       // 底盘 y 轴速度
  float yaw_target = 0.f;       // 云台相对角度
  int8_t robot_state = 1;       // 机器人状态：1=移动姿态，2=防御姿态，3=进攻姿态
  uint8_t auto_aim_mode = 0;    // 0=默认瞄敌人, 1=瞄小符, 2=瞄大符, 3=瞄前哨
  uint8_t chassis_mode = 0;     // 0=正常底盘速度, 1=加速底盘旋转, 2=过起伏路段
  uint8_t super_capacitor = 0;  // 0=关闭超电, 1=开启超电
  uint8_t slowdown_flag = 0;    // 0=不在减速区, 1=前方 detection_dist 内有减速区
  uint8_t end = 0x06;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver_nav2

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
