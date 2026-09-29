#include "rm_behavior_tree/rm_behavior_tree.h"
#include "rm_behavior_tree/bt_conversions.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/plugins.hpp"
#include <rclcpp/executors/single_threaded_executor.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  BT::BehaviorTreeFactory factory;

  std::string bt_xml_path;
  auto node = std::make_shared<rclcpp::Node>("rm_behavior_tree");
  node->declare_parameter<std::string>("bt_xml", "");
  node->get_parameter_or<std::string>("bt_xml", bt_xml_path, "");

  std::cout << "Start RM_Behavior_Tree" << '\n';
  RCLCPP_INFO(node->get_logger(), "Load bt_xml: \e[1;42m %s \e[0m", bt_xml_path.c_str());

  BT::RosNodeParams params_update_msg;
  params_update_msg.nh = std::make_shared<rclcpp::Node>("update_msg");

  BT::RosNodeParams params_robot_control;
  params_robot_control.nh = std::make_shared<rclcpp::Node>("robot_control");
  params_robot_control.default_port_value = "robot_control";

  BT::RosNodeParams params_robot_state;
  params_robot_state.nh = std::make_shared<rclcpp::Node>("robot_state");
  params_robot_state.default_port_value = "robot_state";

  BT::RosNodeParams params_send_goal;
  params_send_goal.nh = std::make_shared<rclcpp::Node>("send_goal");
  params_send_goal.default_port_value = "goal_pose";

  // clang-format off
  const std::vector<std::string> msg_update_plugin_libs = {
    "sub_all_robot_hp",
    "sub_robot_status",
    "sub_game_status",
    // "sub_armors",
    "sub_decision_num",
    "sub_target_pos",
    "sub_gimbal_target",
    "sub_remote_status",
    "sub_auto_aim_status",
    "sub_chassis_energy",
    "sub_rune_status",
  };

  const std::vector<std::string> bt_plugin_libs = {
    "rate_controller",
    "decision_switch",
    "is_friend_ok",
    "is_detect_enemy",
    "is_rfid_detected",
    "is_target_in_zone",
    "is_gimbal_target_in_zone",
    "move_around",     // 暂时禁用，构造函数中有问题
    "print_message",
    "is_time_to_bullet",
    "is_gimbal_target_valid",
    "wait_duration",
    "rm_set_blackboard",
    "get_blackboard",
  };
  // clang-format on

  for (const auto & p : msg_update_plugin_libs) {
    RegisterRosNode(factory, BT::SharedLibrary::getOSName(p), params_update_msg);
  }

  // 注册需要 ROS2 节点参数的节点
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_game_time"), params_update_msg);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_status_ok"), params_update_msg);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_remote_status"), params_update_msg);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_auto_aim_ok"), params_update_msg);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_outpost_ok"), params_update_msg);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_enemy_outpost_ok"), params_update_msg);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_attacked"), params_update_msg);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_base_attacked"), params_update_msg);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_base_status_ok"), params_update_msg);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_chassis_energy_ok"), params_update_msg);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_rune_status"), params_update_msg);

  // 注册 PursueEnemy 节点
  BT::RosNodeParams params_pursue_enemy;
  params_pursue_enemy.nh = std::make_shared<rclcpp::Node>("pursue_enemy");
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pursue_enemy"), params_pursue_enemy);

  // 注册 GetCurrentLocation 节点
  BT::RosNodeParams params_get_current_location;
  params_get_current_location.nh = std::make_shared<rclcpp::Node>("get_current_location");
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("get_current_location"), params_get_current_location);

  for (const auto & p : bt_plugin_libs) {
    factory.registerFromPlugin(BT::SharedLibrary::getOSName(p));
  }

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("send_goal"), params_send_goal);

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("robot_control"), params_robot_control);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pub_robot_state"), params_robot_state);
  
  // 注册发布节点（使用默认参数，不需要特殊的 RosNodeParams）
  BT::RosNodeParams params_pub_default;
  params_pub_default.nh = std::make_shared<rclcpp::Node>("pub_default");
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pub_auto_aim_mode"), params_pub_default);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pub_chassis_mode"), params_pub_default);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pub_super_capacitor"), params_pub_default);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pub_enemy_status"), params_pub_default);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pub_autoaim_target"), params_pub_default);

  auto tree = factory.createTreeFromFile(bt_xml_path);

  // Connect the Groot2Publisher. This will allow Groot2 to get the tree and poll status updates.
  const unsigned port = 1667;
  BT::Groot2Publisher publisher(tree, port);

  // 创建一个 executor 来处理主节点回调
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(params_update_msg.nh);
  executor.add_node(params_get_current_location.nh);
  executor.add_node(params_send_goal.nh);
  executor.add_node(params_pub_default.nh);

  while (rclcpp::ok()) {
    // 处理 ROS2 回调（非阻塞）
    executor.spin_some(std::chrono::milliseconds(0));
    // 执行行为树
    tree.tickWhileRunning(std::chrono::milliseconds(10));
  }

  rclcpp::shutdown();
  return 0;
}