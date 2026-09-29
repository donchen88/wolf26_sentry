// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

// C++ system
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "roborts_msgs/msg/chassis_cmd.hpp"
#include "robot_msgs/msg/all_robot_hp.hpp"
#include "robot_msgs/msg/robot_status.hpp"
#include "robot_msgs/msg/game_status.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "robot_msgs/msg/remote_status.hpp"
#include "robot_msgs/msg/auto_aim_status.hpp"
#include "robot_msgs/msg/auto_aim_mode.hpp"
#include "robot_msgs/msg/chassis_mode.hpp"
#include "robot_msgs/msg/rfid_status.hpp"
#include "robot_msgs/msg/super_capacitor.hpp"
#include "robot_msgs/msg/nav_data.hpp"
#include "robot_msgs/msg/gimbal_target.hpp"
#include "robot_msgs/msg/chassis_energy.hpp"
#include "robot_msgs/msg/rune_status.hpp"


// #include "auto_aim_interfaces/msg/armors.hpp"
struct CHASSISCMD
{
  float lx{0.f};
  float ly{0.f};
  float az{0.f};
  float forward{0.f};
};

using namespace std;
namespace rm_serial_driver_nav2
{
  class RMSerialDriver : public rclcpp::Node
  {
  public:
    explicit RMSerialDriver(const rclcpp::NodeOptions &options);

    ~RMSerialDriver() override;

    float pitch_;
    float yaw_;

    void receiveData();
    void chassisCmd(const roborts_msgs::msg::ChassisCmd::SharedPtr cmd);
    void robotStateCmd(const robot_msgs::msg::RobotState::SharedPtr state);
    void autoAimModeCmd(const robot_msgs::msg::AutoAimMode::SharedPtr mode);
    void chassisModeCmd(const robot_msgs::msg::ChassisMode::SharedPtr mode);
    void superCapacitorCmd(const robot_msgs::msg::SuperCapacitor::SharedPtr state);
    void heartbeatCallback();
    void yawTargetTimer();
    void slowdownFlagCallback(const std_msgs::msg::Bool::SharedPtr msg);
    bool computeYawTarget();

    // ---- 显式订阅 tf / tf_static 的回调 ----
    // 不使用 tf2_ros::TransformListener 的隐式订阅，因为它的内部 subscription
    // 在 namespace + remap 路径下不保证生效；改成自己 create_subscription，
    // launch 里的 ('tf', '/red_standard_robot1/tf') remap 就能 100% 生效，
    // 且 node info 会显示这两个订阅，方便以后排查。
    void tfMessageCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void tfStaticMessageCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

  private:

    void getParams();

    void reopenPort();

    void setParam(const rclcpp::Parameter &param);

    void resetTracker();

    // Serial port
    void sendSerial();
    std::unique_ptr<IoContext> owned_ctx_;
    std::string device_name_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

    int armor_dit{10};
    // Param client to set detect_colr
    using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
    bool initial_set_param_ = false;
    uint8_t previous_receive_color_ = 0;
    rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
    ResultFuturePtr set_param_future_;

    // Service client to reset tracker
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

    rclcpp::Subscription<roborts_msgs::msg::ChassisCmd>::SharedPtr chassis_sub;
    rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr robot_state_sub;
    rclcpp::Subscription<robot_msgs::msg::AutoAimMode>::SharedPtr auto_aim_mode_sub;
    rclcpp::Subscription<robot_msgs::msg::ChassisMode>::SharedPtr chassis_mode_sub;
    rclcpp::Subscription<robot_msgs::msg::SuperCapacitor>::SharedPtr super_capacitor_sub;
    CHASSISCMD cmd_;

    SendPacket robot_cmd_buff_;
    std::mutex cmd_mtx_;
    std::mutex serial_port_mtx_;  // 串口访问互斥锁，保护发送和接收操作

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    // 显式订阅 tf / tf_static（不使用 TransformListener），以保证 launch 文件中的
    // ('tf', '/red_standard_robot1/tf') / ('tf_static', '/red_standard_robot1/tf_static')
    // remap 一定生效。
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
    rclcpp::TimerBase::SharedPtr yaw_target_timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_target_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_current_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr slowdown_flag_sub_;

    std::string global_frame_;
    std::string base_frame_;
    // 期望的 map 系下绝对朝向（弧度）。dyaw = expected_yaw_ - yaw_current
    double expected_yaw_{0.0};
    // dyaw 指数平滑系数，0 表示完全跟新值；越大越平滑。范围 [0, 1]
    double yaw_smoothing_alpha_{0.7};

    rclcpp::Publisher<robot_msgs::msg::AllRobotHP>::SharedPtr refree_hp;
    rclcpp::Publisher<robot_msgs::msg::RobotStatus>::SharedPtr refree_status;
    rclcpp::Publisher<robot_msgs::msg::GameStatus>::SharedPtr refree_game_status;
    rclcpp::Publisher<robot_msgs::msg::RemoteStatus>::SharedPtr refree_remote_status;
    rclcpp::Publisher<robot_msgs::msg::AutoAimStatus>::SharedPtr refree_auto_aim_status;
    rclcpp::Publisher<robot_msgs::msg::RfidStatus>::SharedPtr refree_rfid_status;
    rclcpp::Publisher<robot_msgs::msg::NavData>::SharedPtr refree_nav_data;
    rclcpp::Publisher<robot_msgs::msg::GimbalTarget>::SharedPtr refree_gimbal_target;
    rclcpp::Publisher<robot_msgs::msg::ChassisEnergy>::SharedPtr refree_chassis_energy;
    rclcpp::Publisher<robot_msgs::msg::RuneStatus>::SharedPtr refree_rune_status;


    ReceivePacket robot_inf_buf_;
    std::mutex inf_mtx_;

    std::thread receive_thread_;
    bool auto_shoot{false};
    int fire_count{15};

    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  };
} // namespace rm_serial_driver_nav2

#endif // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
