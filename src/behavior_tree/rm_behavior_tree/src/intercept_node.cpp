#include "rm_behavior_tree/graph_builder.hpp"
#include "rm_behavior_tree/key_point_loader.hpp"
#include "rm_behavior_tree/map_accessor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <mutex>
#include <limits>
#include <cmath>

using namespace std::chrono_literals;

namespace rm_behavior_tree
{

class InterceptNode : public rclcpp::Node
{
public:
  InterceptNode()
  : Node("intercept_node")
  {
    declare_parameter<std::string>("keypoint_yaml", "");
    declare_parameter<std::string>("map_topic", "/map");
    declare_parameter<std::string>("enemy_state_topic", "/enemy_state");
    declare_parameter<std::string>("enemy_velocity_topic", "/enemy_velocity");
    declare_parameter<std::string>("robot_odom_topic", "/red_standard_robot1/chassis_odometry_gt");
    declare_parameter<std::string>("intercept_topic", "/intercept_point");
    declare_parameter<double>("robot_speed", 1.0); // m/s
    declare_parameter<double>("max_considered_enemy_speed", 0.1); // avoid div0

    yaml_path_ = get_parameter("keypoint_yaml").as_string();
    map_topic_ = get_parameter("map_topic").as_string();
    enemy_state_topic_ = get_parameter("enemy_state_topic").as_string();
    enemy_velocity_topic_ = get_parameter("enemy_velocity_topic").as_string();
    robot_odom_topic_ = get_parameter("robot_odom_topic").as_string();
    intercept_topic_ = get_parameter("intercept_topic").as_string();
    robot_speed_ = get_parameter("robot_speed").as_double();
    min_enemy_speed_ = get_parameter("max_considered_enemy_speed").as_double();

    if (yaml_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "keypoint_yaml param required");
      return;
    }

    if (!kp_loader_.loadFromFile(yaml_path_)) {
      RCLCPP_ERROR(get_logger(), "Failed to load keypoints");
      return;
    }

    // subscribe map to build graph
    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic_, map_qos,
      std::bind(&InterceptNode::mapCallback, this, std::placeholders::_1));

    enemy_state_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      enemy_state_topic_, 10, std::bind(&InterceptNode::enemyStateCb, this, std::placeholders::_1));
    enemy_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      enemy_velocity_topic_, 10, std::bind(&InterceptNode::enemyVelCb, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      robot_odom_topic_, 10, std::bind(&InterceptNode::odomCb, this, std::placeholders::_1));

    intercept_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(intercept_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/graph_intercept_point", 10);

    RCLCPP_INFO(get_logger(), "InterceptNode created, waiting for map on %s", map_topic_.c_str());
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (graph_built_) return;
    MapAccessor map(*msg);
    graph_ = GraphBuilder::buildGraph(kp_loader_.getKeyPoints(), map, GraphBuildOptions());
    graph_built_ = true;
    RCLCPP_INFO(get_logger(), "Graph built in intercept node: nodes=%zu edges=%zu", graph_.getNodeCount(), graph_.getEdgeCount()/2);
  }

  void enemyStateCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_enemy_x_ = msg->pose.position.x;
    last_enemy_y_ = msg->pose.position.y;
    enemy_time_ = msg->header.stamp;
    have_enemy_ = true;
    computeInterceptLocked();
  }

  void enemyVelCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_enemy_vx_ = msg->twist.linear.x;
    last_enemy_vy_ = msg->twist.linear.y;
    have_enemy_vel_ = true;
  }

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    have_robot_ = true;
  }

  void computeInterceptLocked()
  {
    if (!graph_built_ || !have_enemy_ || !have_robot_) return;
    // find robot closest keypoint index
    int start_idx = findNearestKeypointIndex(robot_x_, robot_y_);
    if (start_idx < 0) return;

    double evx = have_enemy_vel_ ? last_enemy_vx_ : 0.0;
    double evy = have_enemy_vel_ ? last_enemy_vy_ : 0.0;
    double enemy_speed = std::hypot(evx, evy);
    if (enemy_speed < min_enemy_speed_) enemy_speed = min_enemy_speed_;

    int best_kp_idx = -1;
    double best_score = 1e9;
    double best_t_self = 0.0, best_t_enemy = 0.0;

    for (int i = 0; i < static_cast<int>(graph_.getNodeCount()); ++i) {
      const auto &kp = graph_.nodes[i];
      double dx = kp.x - last_enemy_x_;
      double dy = kp.y - last_enemy_y_;
      double dist_enemy = std::hypot(dx, dy);
      double t_enemy = dist_enemy / enemy_speed;

      // path from robot start_idx to i
      auto res = GraphBuilder::AStarSearch(graph_, start_idx, i);
      if (res.first.empty()) continue;
      double cost = res.second;
      double t_self = cost / robot_speed_;

      double diff = t_enemy - t_self; // positive means enemy arrives later -> good
      if (diff >= 0) {
        // prefer smallest enemy arrival time (so we intercept early)
        if (t_enemy < best_score) {
          best_score = t_enemy;
          best_kp_idx = i;
          best_t_self = t_self;
          best_t_enemy = t_enemy;
        }
      } else {
        // if none positive, choose minimal negative (closest)
        if (best_kp_idx == -1 && std::abs(diff) < best_score) {
          best_score = std::abs(diff);
          best_kp_idx = i;
          best_t_self = t_self;
          best_t_enemy = t_enemy;
        }
      }
    }

    if (best_kp_idx >= 0) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp = this->now();
      ps.header.frame_id = "map";
      ps.pose.position.x = graph_.nodes[best_kp_idx].x;
      ps.pose.position.y = graph_.nodes[best_kp_idx].y;
      ps.pose.position.z = 0.0;
      ps.pose.orientation.w = 1.0;
      intercept_pub_->publish(ps);

      visualization_msgs::msg::Marker m;
      m.header = ps.header;
      m.ns = "intercept";
      m.id = 0;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = ps.pose;
      m.scale.x = 0.4; m.scale.y = 0.4; m.scale.z = 0.4;
      m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2; m.color.a = 0.9;
      marker_pub_->publish(m);

      RCLCPP_INFO(this->get_logger(), "Selected intercept KP id=%d (%.2f,%.2f) t_self=%.3f t_enemy=%.3f",
        graph_.nodes[best_kp_idx].id, ps.pose.position.x, ps.pose.position.y, best_t_self, best_t_enemy);
    }
  }

  int findNearestKeypointIndex(double x, double y)
  {
    int best = -1;
    double bestd = 1e9;
    for (int i = 0; i < static_cast<int>(graph_.getNodeCount()); ++i) {
      double dx = graph_.nodes[i].x - x;
      double dy = graph_.nodes[i].y - y;
      double d = std::hypot(dx, dy);
      if (d < bestd) { bestd = d; best = i; }
    }
    return best;
  }

  // members
  KeyPointLoader kp_loader_;
  Graph graph_;
  bool graph_built_ = false;
  std::mutex mutex_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr enemy_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr enemy_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr intercept_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::string yaml_path_, map_topic_, enemy_state_topic_, enemy_velocity_topic_, robot_odom_topic_, intercept_topic_;
  double robot_speed_ = 1.0;
  double min_enemy_speed_ = 0.1;

  // state
  double last_enemy_x_ = 0.0, last_enemy_y_ = 0.0;
  double last_enemy_vx_ = 0.0, last_enemy_vy_ = 0.0;
  rclcpp::Time enemy_time_;
  double robot_x_ = 0.0, robot_y_ = 0.0;
  bool have_enemy_ = false, have_enemy_vel_ = false, have_robot_ = false;
};

} // namespace rm_behavior_tree

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rm_behavior_tree::InterceptNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


