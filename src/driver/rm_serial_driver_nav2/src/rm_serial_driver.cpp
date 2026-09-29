#include <tf2/time.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "roborts_msgs/msg/chassis_cmd.hpp"
#include "robot_msgs/msg/game_status.hpp"
#include "robot_msgs/msg/all_robot_hp.hpp"
#include "robot_msgs/msg/robot_status.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "robot_msgs/msg/remote_status.hpp"
#include "robot_msgs/msg/auto_aim_status.hpp"
#include "robot_msgs/msg/auto_aim_mode.hpp"
#include "robot_msgs/msg/rfid_status.hpp"
#include "robot_msgs/msg/nav_data.hpp"
#include "robot_msgs/msg/gimbal_target.hpp"
#include "robot_msgs/msg/chassis_energy.hpp"
#include "robot_msgs/msg/rune_status.hpp"

#include "tf2_msgs/msg/tf_message.hpp"

#include "rm_serial_driver_nav2/crc.hpp"
#include "rm_serial_driver_nav2/packet.hpp"
#include "rm_serial_driver_nav2/rm_serial_driver.hpp"


namespace rm_serial_driver_nav2
{
  RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions &options)
      : Node("rm_serial_driver_nav2_node", options),
        owned_ctx_{new IoContext(2)},
        serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
  {
    RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

    getParams();

    // 发给决策
    // 使用 transient_local QoS，让新订阅者立即收到最后一条消息
    rclcpp::QoS status_qos(rclcpp::KeepLast(1));
    status_qos.transient_local();
    status_qos.reliable();
    
    refree_hp = this->create_publisher<robot_msgs::msg::AllRobotHP>("/robot_hp", 10);
    refree_status = this->create_publisher<robot_msgs::msg::RobotStatus>("/robot_status", status_qos);
    refree_game_status = this->create_publisher<robot_msgs::msg::GameStatus>("/game_status", status_qos);
    rclcpp::QoS remote_status_qos(rclcpp::KeepLast(1));
    remote_status_qos.transient_local();
    remote_status_qos.reliable();
    refree_remote_status = this->create_publisher<robot_msgs::msg::RemoteStatus>("/remote_status", 10);
    refree_auto_aim_status = this->create_publisher<robot_msgs::msg::AutoAimStatus>("/auto_aim_status", status_qos);
    refree_rfid_status = this->create_publisher<robot_msgs::msg::RfidStatus>("/referee_rfidStatus", 10);
    refree_nav_data = this->create_publisher<robot_msgs::msg::NavData>("/nav_data", 10);
    refree_gimbal_target = this->create_publisher<robot_msgs::msg::GimbalTarget>("/gimbal_target_point", 10);   
    refree_chassis_energy = this->create_publisher<robot_msgs::msg::ChassisEnergy>("/chassis_energy", 10);
    refree_rune_status = this->create_publisher<robot_msgs::msg::RuneStatus>("/rune_status", status_qos);


    // Detect parameter client  检测客户端参数
    detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

    // Tracker reset service client 追踪重置客户端
    reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

    try
    {
      serial_driver_->init_port(device_name_, *device_config_);
      if (!serial_driver_->port()->is_open())
      {
        serial_driver_->port()->open();
        // 创建线程接收数据
        receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
      }
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(
          get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
      throw ex;
    }

    // 接受底盘速度、朝向的数据
    chassis_sub = this->create_subscription<roborts_msgs::msg::ChassisCmd>(
        "/chassis_cmd", rclcpp::SensorDataQoS(), std::bind(&RMSerialDriver::chassisCmd, this, std::placeholders::_1));
    
    // 接受机器人状态数据
    robot_state_sub = this->create_subscription<robot_msgs::msg::RobotState>(
        "/robot_state", rclcpp::SensorDataQoS(), std::bind(&RMSerialDriver::robotStateCmd, this, std::placeholders::_1));
    
    // 接受自瞄模式数据（0=默认瞄敌人, 1=瞄小符, 2=瞄大符, 3=瞄前哨）
    auto_aim_mode_sub = this->create_subscription<robot_msgs::msg::AutoAimMode>(
        "/auto_aim_mode", rclcpp::SensorDataQoS(), std::bind(&RMSerialDriver::autoAimModeCmd, this, std::placeholders::_1));

    // 接受底盘模式数据（0=正常底盘速度, 1=加速底盘旋转, 2=过起伏路段）
    chassis_mode_sub = this->create_subscription<robot_msgs::msg::ChassisMode>(
        "/chassis_mode", rclcpp::SensorDataQoS(), std::bind(&RMSerialDriver::chassisModeCmd, this, std::placeholders::_1));

    // 接受超电状态数据（0=关闭超电, 1=开启超电）
    super_capacitor_sub = this->create_subscription<robot_msgs::msg::SuperCapacitor>(
        "/super_capacitor", rclcpp::SensorDataQoS(), std::bind(&RMSerialDriver::superCapacitorCmd, this, std::placeholders::_1));

    // 心跳定时器：10Hz 定期发送完整数据包，保证下位机始终收到 14 字节
    heartbeat_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RMSerialDriver::heartbeatCallback, this));

    // TF 监听：查询 global_frame -> base_frame 的姿态，用于计算当前雷达(底盘)朝向
    // 不使用 tf2_ros::TransformListener 的隐式 subscription：它在 namespace +
    // remap 路径下会绕过 launch 的 remap，最终订阅节点 namespace 下的 /tf 与
    // /tf_static（而这两条全局话题根本没人 publish）。改为自己 create_subscription
    // 显式订阅 "tf" / "tf_static"，launch 里 ('tf', '/red_standard_robot1/tf') /
    // ('tf_static', '/red_standard_robot1/tf_static') 才能 100% 生效。
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // 关键：必须 setUsingDedicatedThread(true)。
    // 原因：tf_sub_ 的回调与 yawTargetTimer 在同一个 rclcpp executor 线程上；
    // lookupTransform 带 timeout 时会阻塞等待数据，期间 executor 无法分发 tf_sub_ 回调，
    // buffer 永远不会被填充 → 超时。开启 dedicated thread 后，setTransform 会入队
    // 由 dedicated thread 消费，主线程阻塞 lookup 时数据仍能不断灌入。
    tf_buffer_->setUsingDedicatedThread(true);
    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "tf", rclcpp::SensorDataQoS(),
        std::bind(&RMSerialDriver::tfMessageCallback, this, std::placeholders::_1));
    tf_static_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "tf_static", rclcpp::QoS(100).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(
                         rclcpp::DurabilityPolicy::TransientLocal),
        std::bind(&RMSerialDriver::tfStaticMessageCallback, this, std::placeholders::_1));

    // 周期计算并更新 yaw_target，写入下发的 SendPacket
    // dyaw = expected_yaw_(map 系下) - base_frame 在 map 下的 yaw
    yaw_target_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(16),
        std::bind(&RMSerialDriver::yawTargetTimer, this));

    yaw_target_pub_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_target_to_chassis", 10);
    yaw_current_pub_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_current_of_chassis", 10);

    // 减速区标志：omni_pid_pursuit_controller 按控制频率(20Hz) publish 当前是否在
    // 减速区前方的 detection_dist 范围内，串口驱动直接锁进 buff 里跟着 60Hz 发下去。
    // buff 初值 0（不在），节点启动后第一帧 TF/slowdown 未就绪也不会发 1 骗下位机。
    slowdown_flag_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/in_slowdown_zone", rclcpp::QoS(10),
        std::bind(&RMSerialDriver::slowdownFlagCallback, this, std::placeholders::_1));
  }
  RMSerialDriver::~RMSerialDriver()
  {
    if (receive_thread_.joinable())
    {
      receive_thread_.join();
    }

    if (serial_driver_->port()->is_open())
    {
      serial_driver_->port()->close();
    }

    if (owned_ctx_)
    {
      owned_ctx_->waitForExit();
    }
  }
  void RMSerialDriver::receiveData()
  {
    std::vector<uint8_t> receive_buffer;  // 接收缓冲区
    receive_buffer.reserve(sizeof(ReceivePacket) * 2);  // 预分配空间
    const size_t packet_size = sizeof(ReceivePacket);
    const uint8_t expected_header = 0x06;
    const uint8_t expected_end = 0x09;
    const size_t max_buffer_size = packet_size * 3;  // 最大缓冲区大小

    while (rclcpp::ok())
    {
      try
      {
        // 从串口接收数据到缓冲区
        // 使用串口互斥锁保护接收操作
        // 注意：接收操作应该优先，因为电控会持续发送数据，如果接收被阻塞太久可能导致数据丢失
        std::vector<uint8_t> temp_data(packet_size);  // 临时缓冲区，至少能容纳一个数据包
        bool data_received = false;
        
        // 获取锁（接收线程优先，确保不会丢失数据）
        // 发送操作应该很快完成，所以短暂的阻塞是可以接受的
        {
          std::lock_guard<std::mutex> serial_lock(serial_port_mtx_);
          try
          {
            serial_driver_->port()->receive(temp_data);
            data_received = true;
          }
          catch (const std::exception &ex)
          {
            // 如果接收失败，可能是没有数据或超时，继续循环
            rclcpp::sleep_for(std::chrono::milliseconds(1));
            continue;
          }
        }
        
        // 将接收到的数据添加到缓冲区（只添加实际接收到的数据）
        // 注意：receive() 可能会修改 temp_data 的大小，只包含实际接收到的数据
        if (data_received && !temp_data.empty())
        {
          receive_buffer.insert(receive_buffer.end(), temp_data.begin(), temp_data.end());
        }
        else
        {
          // 如果没有接收到数据，短暂等待后继续
          rclcpp::sleep_for(std::chrono::milliseconds(1));
          continue;
        }
        
        // 限制缓冲区大小，避免内存无限增长
        if (receive_buffer.size() > max_buffer_size)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5, 
                              "Receive buffer too large (%zu bytes), clearing...", 
                              receive_buffer.size());
          receive_buffer.clear();
          continue;
        }

        // 在缓冲区中查找完整的数据包
        bool packet_found = false;
        size_t packet_start_idx = 0;

        // 查找数据包：从头开始查找头字节
        // 如果缓冲区开头不是有效头字节，清理掉无效数据直到找到头字节或缓冲区太小
        while (receive_buffer.size() >= packet_size)
        {
          if (receive_buffer[0] == expected_header)
          {
            // 找到可能的头字节，检查是否是一个完整的数据包
            if (receive_buffer.size() >= packet_size)
            {
              // 检查尾字节
              if (receive_buffer[packet_size - 1] == expected_end)
              {
                // 找到完整的数据包
                packet_start_idx = 0;
                packet_found = true;
                break;
              }
              else
              {
                // 头字节正确但尾字节不对，可能是数据包不完整，继续接收
                break;
              }
            }
          }
          else
          {
            // 缓冲区开头不是有效头字节，移除第一个字节，继续查找
            receive_buffer.erase(receive_buffer.begin());
          }
        }

        if (!packet_found)
        {
          // 如果没有找到完整的数据包，保留最后 (packet_size - 1) 个字节（可能包含下一个数据包的头）
          if (receive_buffer.size() > packet_size - 1)
          {
            std::vector<uint8_t> remaining(receive_buffer.end() - (packet_size - 1), receive_buffer.end());
            receive_buffer = remaining;
          }
          // 如果缓冲区太小，继续接收数据
          continue;
        }

        // 提取完整的数据包
        std::vector<uint8_t> packet_data(receive_buffer.begin() + packet_start_idx, 
                                         receive_buffer.begin() + packet_start_idx + packet_size);
        
        // 从缓冲区中移除已处理的数据包
        receive_buffer.erase(receive_buffer.begin(), receive_buffer.begin() + packet_start_idx + packet_size);

        ReceivePacket packet = fromVector(packet_data);

        // 验证头字节和尾字节（双重检查）
        if (packet.start != expected_header || packet.end != expected_end)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5, 
                              "Invalid packet markers: start=0x%02X (expected 0x%02X), end=0x%02X (expected 0x%02X). Resynchronizing...", 
                              packet.start, expected_header, packet.end, expected_end);
          continue;
        }

        // 数据合理性验证：检查关键字段是否在合理范围内
        // 血量应该在 0-2000 范围内（根据实际情况调整）
        if (packet.sentry_hp > 2000)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5, 
                              "Invalid sentry_hp value: %u (expected <= 2000). Discarding packet...", 
                              packet.sentry_hp);
          continue;
        }
        
        // 剩余弹药量应该在合理范围内（根据实际情况调整，比如 0-1000）
        if (packet.bullet_remain > 1000)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5, 
                              "Invalid bullet_remain value: %u (expected <= 1000). Discarding packet...", 
                              packet.bullet_remain);
          continue;
        }
        
        // 枪口热量应该在合理范围内（根据实际情况调整，比如 0-1000）
        if (packet.shooter_barrel_heat_limit > 1000)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5, 
                              "Invalid shooter_barrel_heat_limit value: %u (expected <= 1000). Discarding packet...", 
                              packet.shooter_barrel_heat_limit);
          continue;
        }
        
        // 比赛阶段应该在合理范围内（通常 0-10）
        if (packet.game_progress > 10)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5, 
                              "Invalid game_progress value: %u (expected <= 10). Discarding packet...", 
                              packet.game_progress);
          continue;
        }

        // 轮速合理性验证（范围 -10 ~ 10 m/s，步兵正常移动速度）
        if (std::abs(packet.wheel_front_left) > 10.f ||
            std::abs(packet.wheel_front_right) > 10.f ||
            std::abs(packet.wheel_back_left) > 10.f ||
            std::abs(packet.wheel_back_right) > 10.f)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5,
                              "Invalid wheel speed values. Discarding packet...");
          continue;
        }

        // 陀螺仪角度合理性验证（范围 -180 ~ 180 度）
        if (std::abs(packet.gyro_pitch) > 360.f || std::abs(packet.gyro_roll) > 360.f)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5,
                              "Invalid gyro values: pitch=%.1f roll=%.1f. Discarding packet...",
                              packet.gyro_pitch, packet.gyro_roll);
          continue;
        }


        // 底盘能量应该在合理范围内（根据实际情况调整，比如 0-40000 J）
        if (packet.chassis_energy > 40000)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5,
                              "Invalid chassis_energy value: %u (expected <= 40000). Discarding packet...",
                              packet.chassis_energy);
          continue;
        }

        // 符文计数合法性检查（0/1/2）
        if (packet.rune_count > 2)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5,
                              "Invalid rune_count value: %u (expected 0..2). Discarding packet...",
                              packet.rune_count);
          continue;
        }

        // 验证CRC（如果数据包包含CRC）
        // 注意：根据ReceivePacket结构，尾字节是0x09，CRC可能在尾字节之前
        // 这里我们只验证尾字节，如果需要CRC校验，需要根据实际协议调整
        // 如果数据包格式包含CRC，可以使用：crc16::Verify_CRC16_Check_Sum(data.data(), data.size())

        // 数据包验证通过，处理数据
        robot_msgs::msg::AllRobotHP hp_msg;
        robot_msgs::msg::RobotStatus robot_status_msg; 
        robot_msgs::msg::GameStatus game_status_msg;

        // 设置己方前哨站血量（用于 IsOutpostOK 节点）
        hp_msg.outpost_hp = packet.outpost_hp;

        // 设置己方基地血量（用于 IsBaseAttacked 节点）
        hp_msg.base_hp = packet.base_hp;

        // 设置队伍颜色（用于判断己方和敌方前哨站）
        // IsOutpostOK 节点使用 team_color 判断己方前哨站
        // IsEnemyOutpostOK 节点使用 team_color 判断敌方前哨站
        if(packet.team_color == 1){//己方队伍颜色
          hp_msg.team_color = true; //红
          hp_msg.enemy_color = false; //蓝
        }else{
          hp_msg.team_color = false; //蓝
          hp_msg.enemy_color = true; //红
        }

        // 设置敌方前哨站血量（用于 IsEnemyOutpostOK 节点）
        // 由电控直接下发敌方前哨站血量；驱动不做推断
        hp_msg.enemy_outpost_hp = packet.enemy_outpost_hp;

        // 设置机器人状态消息（电控实时发送的数据）
        // IsStatusOKAction 节点只需要这三个字段进行判断
        robot_status_msg.current_hp = packet.sentry_hp;  // 实时血量
        robot_status_msg.shooter_barrel_heat_limit = packet.shooter_barrel_heat_limit;  // 实时枪口热量
        robot_status_msg.shooter_heat = packet.bullet_remain;  // 实时剩余弹药量
        // 其他字段（robot_id, team_color, is_attacked）会自动使用默认值（0, false, false）

        game_status_msg.game_progress = packet.game_progress;
        game_status_msg.stage_remain_time = packet.game_progress_remain;

        // 设置遥控器状态消息
        robot_msgs::msg::RemoteStatus remote_status_msg;
        remote_status_msg.remote_control_status = packet.remote_control_status;

        // 设置自瞄状态消息
        robot_msgs::msg::AutoAimStatus auto_aim_status_msg;
        auto_aim_status_msg.auto_aim_status = packet.auto_aim_status;

        // 设置 RFID 状态消息（将 uint8_t 转换为 bool）
        robot_msgs::msg::RfidStatus rfid_status_msg;
        rfid_status_msg.base_gain_point = (packet.base_gain_point_rfid == 1);
        rfid_status_msg.friendly_fortress_gain_point = (packet.friendly_fortress_gain_point_rfid == 1);
        rfid_status_msg.center_gain_point = (packet.center_gain_point_rfid == 1);

        refree_hp->publish(hp_msg);
        refree_status->publish(robot_status_msg);
        refree_game_status->publish(game_status_msg);
        refree_remote_status->publish(remote_status_msg);
        refree_auto_aim_status->publish(auto_aim_status_msg);
        refree_rfid_status->publish(rfid_status_msg);


        // 设置导航数据消息（轮速 + IMU）
        robot_msgs::msg::NavData nav_data_msg;
        nav_data_msg.header.stamp = this->now();
        nav_data_msg.wheel_speeds = {
          packet.wheel_front_left,
          packet.wheel_front_right,
          packet.wheel_back_right,
          packet.wheel_back_left
        };
        nav_data_msg.imu_angles = {packet.gyro_pitch, packet.gyro_roll};
        refree_nav_data->publish(nav_data_msg);


        // 发布云台手目标点
        robot_msgs::msg::GimbalTarget gimbal_target_msg;
        gimbal_target_msg.x = packet.gimbal_target_x;
        gimbal_target_msg.y = packet.gimbal_target_y;
        refree_gimbal_target->publish(gimbal_target_msg);

        // 设置底盘能量消息
        robot_msgs::msg::ChassisEnergy chassis_energy_msg;
        chassis_energy_msg.chassis_energy = packet.chassis_energy;
        refree_chassis_energy->publish(chassis_energy_msg);

        // 设置符文状态消息（用于 IsRuneStatus 节点）
        robot_msgs::msg::RuneStatus rune_status_msg;
        rune_status_msg.rune_count = packet.rune_count;
        refree_rune_status->publish(rune_status_msg);
      }
      catch (const std::exception &ex)
      {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
        reopenPort();
        // 发生异常后，等待一小段时间再继续
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  // 将底盘数据、朝向角数据发送到串口
  void RMSerialDriver::sendSerial()
  {
    if (!serial_driver_->port()->is_open())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5, "Serial port is not open, cannot send data");
      return;
    }

    std::lock_guard<std::mutex> serial_lock(serial_port_mtx_);
    try
    {
      SendPacket send_packet;
      {
        std::lock_guard<std::mutex> lock(cmd_mtx_);
        send_packet = robot_cmd_buff_;
      }

      std::vector<uint8_t> data = toVector(send_packet);
      // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2,
      //                       "Sending serial: [%02X] lx=%.3f ly=%.3f state=%d aim=%d chassis=%d cap=%d [%02X] (total %zu bytes)",
      //                       data[0], send_packet.chassis_lx, send_packet.chassis_ly,
      //                       send_packet.robot_state, send_packet.auto_aim_mode,
      //                       send_packet.chassis_mode, send_packet.super_capacitor,
      //                       data.back(), data.size());
      serial_driver_->port()->send(data);
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5, "Error sending data to serial port: %s", ex.what());
    }
  }

  void RMSerialDriver::chassisCmd(const roborts_msgs::msg::ChassisCmd::SharedPtr cmd)
  {
    {
      std::lock_guard<std::mutex> lock(cmd_mtx_);
      robot_cmd_buff_.chassis_lx = cmd->lx;
      robot_cmd_buff_.chassis_ly = cmd->ly;
    }

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *get_clock(), 1.0,
                          "x轴速度%lf, y轴速度%lf, 状态:%d, 自瞄模式:%d, 底盘模式:%d",
                          robot_cmd_buff_.chassis_lx, robot_cmd_buff_.chassis_ly, robot_cmd_buff_.robot_state,
                          robot_cmd_buff_.auto_aim_mode, robot_cmd_buff_.chassis_mode);

    sendSerial();
  }

  // 将机器人状态数据发送到串口
  void RMSerialDriver::robotStateCmd(const robot_msgs::msg::RobotState::SharedPtr state)
  {
    {
      std::lock_guard<std::mutex> lock(cmd_mtx_);
      robot_cmd_buff_.robot_state = state->state;
    }
    sendSerial();

    RCLCPP_INFO(this->get_logger(), "Robot state updated: %d", robot_cmd_buff_.robot_state);
  }

  void RMSerialDriver::autoAimModeCmd(const robot_msgs::msg::AutoAimMode::SharedPtr mode)
  {
    {
      std::lock_guard<std::mutex> lock(cmd_mtx_);
      robot_cmd_buff_.auto_aim_mode = mode->mode;
    }
    sendSerial();

    RCLCPP_INFO(this->get_logger(), "Auto aim mode updated: %d", robot_cmd_buff_.auto_aim_mode);
  }

  void RMSerialDriver::chassisModeCmd(const robot_msgs::msg::ChassisMode::SharedPtr mode)
  {
    {
      std::lock_guard<std::mutex> lock(cmd_mtx_);
      robot_cmd_buff_.chassis_mode = mode->mode;
    }
    sendSerial();

    RCLCPP_INFO(this->get_logger(),
                "Chassis mode updated: %d | lx=%.3f ly=%.3f state=%d aim=%d cap=%d",
                robot_cmd_buff_.chassis_mode,
                robot_cmd_buff_.chassis_lx, robot_cmd_buff_.chassis_ly,
                robot_cmd_buff_.robot_state, robot_cmd_buff_.auto_aim_mode,
                robot_cmd_buff_.super_capacitor);
  }

  void RMSerialDriver::superCapacitorCmd(const robot_msgs::msg::SuperCapacitor::SharedPtr state)
  {
    {
      std::lock_guard<std::mutex> lock(cmd_mtx_);
      robot_cmd_buff_.super_capacitor = state->state;
    }
    sendSerial();

    RCLCPP_INFO(this->get_logger(), "Super capacitor state updated: %d", robot_cmd_buff_.super_capacitor);
  }

  // 心跳定时器回调：10Hz 定期发送完整 14 字节数据包
  void RMSerialDriver::heartbeatCallback()
  {
    sendSerial();
  }

  // 减速区标志：omni_pid_pursuit_controller 在 applySlowdownZoneVelocityScaling
  // 算出来"前方 detection_dist 内有减速区"→ true。每发一帧就立刻锁进 buff，
  // 心跳/chassis 任一通道都会带着它 60Hz 拍给下位机。
  void RMSerialDriver::slowdownFlagCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cmd_mtx_);
    robot_cmd_buff_.slowdown_flag = msg->data ? 1 : 0;
  }

  // 显式订阅 tf / tf_static 的回调——保证 launch 文件中的 remap 一定生效。
  // dynamic TF 用 SensorDataQoS，static TF 必须用 TransientLocal + Reliable，
  // 否则 latched 的那 1 条可能在 subscriber 启动时收不到，buffer 会一直缺这个 frame。
  void RMSerialDriver::tfMessageCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (const auto & tf : msg->transforms) {
      tf_buffer_->setTransform(tf, "rm_serial_driver", false);
    }
  }

  void RMSerialDriver::tfStaticMessageCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (const auto & tf : msg->transforms) {
      tf_buffer_->setTransform(tf, "rm_serial_driver", true);
    }
  }

  // 周期计算当前底盘相对 map 的 yaw，与 yaml 中的 expected_yaw 作差得到 dyaw
  void RMSerialDriver::yawTargetTimer()
  {
    // 1. 查询 base_frame -> global_frame 的 TF
    double yaw_current = 0.0;
    try
    {
      geometry_msgs::msg::TransformStamped tf =
          tf_buffer_->lookupTransform(global_frame_, base_frame_, tf2::TimePointZero,
                                      tf2::durationFromSec(0.05));
      const auto & q = tf.transform.rotation;
      // yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
      const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
      const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      yaw_current = std::atan2(siny_cosp, cosy_cosp);
    }
    catch (const tf2::TransformException & ex)
    {
      // TF 还没好就别动 dyaw，避免指数平滑被错误的 0 拉偏
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5,
                          "TF lookup %s -> %s failed: %s",
                          global_frame_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }

    // 2. 归一化辅助
    auto wrap_pi = [](double a) {
      while (a > M_PI) a -= 2.0 * M_PI;
      while (a <= -M_PI) a += 2.0 * M_PI;
      return a;
    };

    // 3. 计算 dyaw = expected_yaw - yaw_current，并做指数平滑
    const double dyaw = wrap_pi(expected_yaw_ - yaw_current);

    static double dyaw_smoothed = 0.0;
    static bool dyaw_inited = false;
    if (!dyaw_inited)
    {
      dyaw_smoothed = dyaw;
      dyaw_inited = true;
    }
    else
    {
      const double a = std::clamp(yaw_smoothing_alpha_, 0.0, 1.0);
      // 选较短路径（避免越过 ±pi 跳变）
      const double delta = wrap_pi(dyaw - dyaw_smoothed);
      dyaw_smoothed = wrap_pi(dyaw_smoothed + (1.0 - a) * delta);
    }

    // 4. 写进下发缓冲
    {
      std::lock_guard<std::mutex> lock(cmd_mtx_);
      robot_cmd_buff_.yaw_target = static_cast<float>(dyaw_smoothed * (180.0 / M_PI));
    }

    // 5. 调试发布
    std_msgs::msg::Float64 msg_cur;
    msg_cur.data = yaw_current;
    yaw_current_pub_->publish(msg_cur);

    std_msgs::msg::Float64 msg_tgt;
    msg_tgt.data = dyaw_smoothed;
    yaw_target_pub_->publish(msg_tgt);
  }

  // 获取参数
  void RMSerialDriver::getParams()
  {
    // 无法获取到yaml文件中预设的值，直接在下列内容中手动设置

    // 定义流控制、奇偶校验和停止位的枚举类型
    using FlowControl = drivers::serial_driver::FlowControl;
    using Parity = drivers::serial_driver::Parity;
    using StopBits = drivers::serial_driver::StopBits;

    // 定义波特率变量
    uint32_t baud_rate{};

    // 初始化流控制、奇偶校验和停止位的变量
    auto fc = FlowControl::NONE;
    auto pt = Parity::NONE;
    auto sb = StopBits::ONE;

    try
    {
      // 设置串口名
      device_name_ = declare_parameter<std::string>("device_name", "/dev/ttyACM111"); // dev/ lslidar
      RCLCPP_INFO(get_logger(), "device_name parameter value: %s", device_name_.c_str());
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
      throw ex;
    }

    try
    {
      // 设置波特率
      baud_rate = std::stoi(declare_parameter<std::string>("baud_rate", "115200"));
      RCLCPP_INFO(get_logger(), "baud_rate parameter value: %d", baud_rate);
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
      throw ex;
    }
    try
    {
      // 定义flow_control参数，并设置默认值为"none"
      const auto fc_param = declare_parameter("flow_control", rclcpp::ParameterValue("none"));

      try
      {
        // 获取flow_control参数的值，并将其转换为字符串类型
        std::string fc_string = fc_param.get<std::string>();

        // 检查fc_string的值
        // RCLCPP_INFO(get_logger(), "Flow control parameter value: %s", fc_string.c_str());

        // 根据fc_string的值设置fc变量的值
        if (fc_string == "none")
        {
          fc = FlowControl::NONE;
        }
        else if (fc_string == "hardware")
        {
          fc = FlowControl::HARDWARE;
        }
        else if (fc_string == "software")
        {
          fc = FlowControl::SOFTWARE;
        }
        else
        {
          // 如果fc_string的值无效，则输出错误信息
          RCLCPP_ERROR(get_logger(), "Invalid value for flow_control: %s", fc_string.c_str());
        }
      }
      catch (const std::exception &ex)
      {
        // 如果处理flow_control参数时出现异常，则输出错误信息
        RCLCPP_ERROR(get_logger(), "Error processing flow_control parameter: %s", ex.what());
      }
    }
    catch (const std::exception &ex)
    {
      // 如果获取flow_control参数时出现异常，则输出错误信息，并重新抛出异常
      RCLCPP_ERROR(get_logger(), "Error getting flow_control parameter: %s", ex.what());
      throw ex;
    }

    try
    {
      // 定义parity参数，并设置默认值为"none"
      const auto pt_string = declare_parameter("parity", rclcpp::ParameterValue("none"));

      if (pt_string.get<std::string>() == "none")
      {
        // 如果parity的值为"none"，则设置pt变量的值为Parity::NONE
        pt = Parity::NONE;
      }
      else if (pt_string.get<std::string>() == "odd")
      {
        // 如果parity的值为"odd"，则设置pt变量的值为Parity::ODD
        pt = Parity::ODD;
      }
      else if (pt_string.get<std::string>() == "even")
      {
        // 如果parity的值为"even"，则设置pt变量的值为Parity::EVEN
        pt = Parity::EVEN;
      }
      else
      {
        // 如果parity的值无效，则输出错误信息
        RCLCPP_ERROR(get_logger(), "Invalid value for parity: %s", to_string(pt_string).c_str());
        // 没有抛出异常，因为使用了默认值，不会导致无效的参数值
      }
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      // 如果parity参数的类型无效，则输出错误信息，并重新抛出异常
      RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
      throw ex;
    }

    try
    {
      // 定义stop_bits参数，并设置默认值为"1"
      const auto sb_string_param = declare_parameter("stop_bits", rclcpp::ParameterValue("1"));
      std::string sb_string = sb_string_param.get<std::string>();

      if (sb_string == "1" || sb_string == "1.0")
      {
        // 如果stop_bits的值为"1"或"1.0"，则设置sb变量的值为StopBits::ONE
        sb = StopBits::ONE;
      }
      else if (sb_string == "1.5")
      {
        // 如果stop_bits的值为"1.5"，则设置sb变量的值为StopBits::ONE_POINT_FIVE
        sb = StopBits::ONE_POINT_FIVE;
      }
      else if (sb_string == "2" || sb_string == "2.0")
      {
        // 如果stop_bits的值为"2"或"2.0"，则设置sb变量的值为StopBits::TWO
        sb = StopBits::TWO;
      }
      else
      {
        // 如果stop_bits的值无效，则抛出异常
        throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
      }
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      // 如果stop_bits参数的类型无效，则输出错误信息，并重新抛出异常
      RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
      throw ex;
    }

    // 根据获取到的参数值创建device_config_对象
    device_config_ =
        std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
    
    // 计算 yaw_target 用到的 TF frame、平滑系数与期望朝向
    global_frame_ = this->declare_parameter<std::string>("global_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    expected_yaw_ = this->declare_parameter<double>("expected_yaw", 0.0);
    yaw_smoothing_alpha_ = this->declare_parameter<double>("yaw_smoothing_alpha", 0.7);
    RCLCPP_INFO(get_logger(),
                "yaw_target: global_frame=%s base_frame=%s expected_yaw=%.3f rad smoothing_alpha=%.3f",
                global_frame_.c_str(), base_frame_.c_str(), expected_yaw_, yaw_smoothing_alpha_);
  }

  void RMSerialDriver::reopenPort()
  {
    // 尝试打开端口
    RCLCPP_WARN(get_logger(), "Attempting to reopen port");
    try
    {
      // 如果端口已打开，则先关闭端口
      if (serial_driver_->port()->is_open())
      {
        serial_driver_->port()->close();
      }
      // 打开端口
      serial_driver_->port()->open();
      RCLCPP_INFO(get_logger(), "Successfully reopened port");
    }
    catch (const std::exception &ex)
    {
      // 获取重新打开端口是发生的错误信息
      RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
      // 如果系统状态正常（rclcpp::ok()返回true）
      if (rclcpp::ok())
      {
        // 等待1秒钟后再次尝试重新打开端口
        rclcpp::sleep_for(std::chrono::seconds(1));
        reopenPort();
      }
    }
  }

  void RMSerialDriver::setParam(const rclcpp::Parameter &param)
  {
    // 检查detector_param_client_的服务是否就绪
    if (!detector_param_client_->service_is_ready())
    {
      RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
      return;
    }

    // 判断set_param_future_是否有效以及其状态是否已变为“ready”。
    if (!set_param_future_.valid() || set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      // 输出表示将要设置detect_color参数的值为param.as_int()
      RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());

      // 设置detect_color参数的异步请求
      set_param_future_ = detector_param_client_->set_parameters(
          {param}, [this, param](const ResultFuturePtr &results)
          {
        // 遍历所有参数设置的结果
        for (const auto & result : results.get())
        {
          // 如果参数设置失败，则输出错误信息
          if (!result.successful)
          {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }

        // 输出参数设置成功的消息
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());

        // 标记初始参数设置已完成
        initial_set_param_ = true; });
    }
  }

  void RMSerialDriver::resetTracker()
  {
    // 检查重置追踪器服务是否就绪
    if (!reset_tracker_client_->service_is_ready())
    {
      // 如果服务未准备就绪，则输出警告信息并跳过重置操作
      RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
      return;
    }

    // 创建一个指向Trigger服务请求的智能指针
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    // 异步发送重置追踪器的请求
    reset_tracker_client_->async_send_request(request);
    // 记录追踪器重置成功的日志信息
    RCLCPP_INFO(get_logger(), "Reset tracker!");
  }

} // namespace rm_serial_driver_nav2

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver_nav2::RMSerialDriver)
