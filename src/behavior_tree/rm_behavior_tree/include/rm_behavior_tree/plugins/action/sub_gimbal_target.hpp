#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GIMBAL_TARGET_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GIMBAL_TARGET_HPP_

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robot_msgs/msg/gimbal_target.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace rm_behavior_tree
{

/**
 * @brief 订阅云台手通过裁判系统发送的目标点，使用TF进行坐标系转换后写入黑板
 *
 * 节点内部会发布 gimbal_frame -> map 的静态TF，
 * 你可以在XML中配置云台坐标系原点在map中的位置。
 *
 * 云台手发送的坐标是在gimbal_frame坐标系下的坐标（建议第一象限），
 * 会自动转换到map坐标系。
 *
 * 注意：超时控制由 IsGimbalTargetValid 节点负责，不要在此节点设置超时。
 *
 * @param[in] topic_name 话题名称，用于接收云台手的目标点
 * @param[in] gimbal_frame 云台坐标系名称（TF frame ID），默认 "gimbal_field"
 * @param[in] gimbal_origin_x gimbal_frame原点在map中的X坐标
 * @param[in] gimbal_origin_y gimbal_frame原点在map中的Y坐标
 * @param[in] gimbal_origin_z gimbal_frame原点在map中的Z坐标
 * @param[in] gimbal_yaw gimbal_frame在map中的偏航角（弧度）
 * @param[out] target_pose 转换后的目标点 (PoseStamped格式)
 * @param[out] target_valid 标记是否有有效的云台手目标点
 * @param[out] target_x 原始云台目标点x坐标
 * @param[out] target_y 原始云台目标点y坐标
 *
 * 使用示例：
 * <SubGimbalTarget topic_name="gimbal_target_point"
 *                  gimbal_frame="gimbal_field"
 *                  gimbal_origin_x="-0.527905"
 *                  gimbal_origin_y="-7.621"
 *                  gimbal_origin_z="0.0"
 *                  gimbal_yaw="0.0"
 *                  target_pose="{gimbal_target_pose}"
 *                  target_valid="{gimbal_target_valid}"
 *                  target_x="{gimbal_target_x}"
 *                  target_y="{gimbal_target_y}"/>
 */
class SubGimbalTargetAction : public BT::SyncActionNode
{
public:
  SubGimbalTargetAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name", "gimbal_target_point",
          "Topic name for gimbal target position"),
      BT::InputPort<std::string>("gimbal_frame", "gimbal_field",
          "TF frame ID of gimbal coordinate system (e.g., 'gimbal_field')"),
      BT::InputPort<double>("gimbal_origin_x", "0.0",
          "X coordinate of gimbal_frame origin in map frame"),
      BT::InputPort<double>("gimbal_origin_y", "0.0",
          "Y coordinate of gimbal_frame origin in map frame"),
      BT::InputPort<double>("gimbal_origin_z", "0.0",
          "Z coordinate of gimbal_frame origin in map frame"),
      BT::InputPort<double>("gimbal_yaw", "0.0",
          "Yaw angle of gimbal_frame in map frame (radians)"),
      BT::OutputPort<std::string>("target_pose",
          "Converted target pose in map frame as string: x;y;z;qx;qy;qz;qw"),
      BT::OutputPort<bool>("target_valid", "Whether gimbal target is valid"),
      BT::OutputPort<double>("target_x", "Raw gimbal target x in gimbal_frame (before TF transform)"),
      BT::OutputPort<double>("target_y", "Raw gimbal target y in gimbal_frame (before TF transform)")};
  }

  BT::NodeStatus tick() override;

private:
  /**
   * @brief 使用TF将点从gimbal_frame转换到map_frame
   * @param gimbal_x gimbal坐标系下的x坐标
   * @param gimbal_y gimbal坐标系下的y坐标
   * @param map_pose 输出转换后的PoseStamped（map坐标系）
   * @return 是否转换成功
   */
  bool transformPointToMap(double gimbal_x, double gimbal_y, geometry_msgs::msg::PoseStamped & map_pose);

  /**
   * @brief 使用TF将点从gimbal_frame转换到map_frame，输出字符串格式
   * @param gimbal_x gimbal坐标系下的x坐标
   * @param gimbal_y gimbal坐标系下的y坐标
   * @return 字符串格式的pose: x;y;z;qx;qy;qz;qw
   */
  std::string transformPointToMapString(double gimbal_x, double gimbal_y);

  // 回调函数
  void gimbalTargetCallback(const robot_msgs::msg::GimbalTarget::SharedPtr msg);

  // 发布静态TF
  void publishStaticTf();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<robot_msgs::msg::GimbalTarget>::SharedPtr sub_;

  // TF相关
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  bool tf_published_;  // 标记静态TF是否已发布

  // 配置参数
  std::string gimbal_frame_;       // 云台坐标系名称
  std::string target_frame_;       // 目标坐标系（通常是map）
  double gimbal_origin_x_;         // gimbal_frame原点在map中的X
  double gimbal_origin_y_;         // gimbal_frame原点在map中的Y
  double gimbal_origin_z_;         // gimbal_frame原点在map中的Z
  double gimbal_yaw_;              // gimbal_frame在map中的偏航角

  // 最新收到的有效目标点（原始gimbal坐标）
  double last_gimbal_x_;
  double last_gimbal_y_;
  bool has_valid_target_;
  bool subscription_created_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GIMBAL_TARGET_HPP_
