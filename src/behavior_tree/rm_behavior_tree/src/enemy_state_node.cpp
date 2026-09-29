#include "rm_behavior_tree/key_point_loader.hpp"
#include "rm_behavior_tree/map_accessor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <auto_aim_interfaces/msg/armors.hpp>
#include <deque>
#include <mutex>
#include <numeric>

using namespace std::chrono_literals;

namespace rm_behavior_tree
{

struct EnemySample
{
  rclcpp::Time stamp;
  double x;
  double y;
};

class EnemyStateNode : public rclcpp::Node
{
public:
  EnemyStateNode()
  : Node("enemy_state_node")
  {
    // parameters
    this->declare_parameter<std::string>("armors_topic", "/detector/armors");
    this->declare_parameter<std::string>("enemy_pose_topic", "/enemy_pose");
    this->declare_parameter<std::string>("enemy_state_topic", "/enemy_state");
    this->declare_parameter<std::string>("enemy_velocity_topic", "/enemy_velocity");
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<int>("window_size", 5);
    this->declare_parameter<double>("smoothing_alpha", 0.6);
    this->declare_parameter<double>("min_velocity_threshold", 0.02);

    armors_topic_ = this->get_parameter("armors_topic").as_string();
    enemy_pose_topic_ = this->get_parameter("enemy_pose_topic").as_string();
    enemy_state_topic_ = this->get_parameter("enemy_state_topic").as_string();
    enemy_velocity_topic_ = this->get_parameter("enemy_velocity_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    window_size_ = this->get_parameter("window_size").as_int();
    smoothing_alpha_ = this->get_parameter("smoothing_alpha").as_double();
    min_velocity_threshold_ = this->get_parameter("min_velocity_threshold").as_double();

    RCLCPP_INFO(this->get_logger(), "EnemyStateNode params: armors_topic=%s, enemy_pose_topic=%s, window=%d, alpha=%.2f",
      armors_topic_.c_str(), enemy_pose_topic_.c_str(), window_size_, smoothing_alpha_);

    // publishers
    enemy_state_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(enemy_state_topic_, 10);
    enemy_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(enemy_velocity_topic_, 10);
    enemy_conf_pub_ = this->create_publisher<std_msgs::msg::Float64>("enemy_confidence", 10);

    // subscribers
    armors_sub_ = this->create_subscription<auto_aim_interfaces::msg::Armors>(
      armors_topic_, 10,
      std::bind(&EnemyStateNode::armorsCallback, this, std::placeholders::_1));

    enemy_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      enemy_pose_topic_, 10,
      std::bind(&EnemyStateNode::poseCallback, this, std::placeholders::_1));
  }

private:
  void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr msg)
  {
    if (!msg) return;
    if (msg->armors.empty()) return;

    // choose first armor (could select by confidence or distance)
    const auto & armor = msg->armors[0];
    double x = armor.pose.position.x;
    double y = armor.pose.position.y;
    rclcpp::Time t;
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
      t = this->now();
    } else {
      t = rclcpp::Time(msg->header.stamp);
    }

    pushSample(t, x, y);
    publishState(t, x, y);
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg) return;
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    rclcpp::Time t;
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
      t = this->now();
    } else {
      t = rclcpp::Time(msg->header.stamp);
    }

    pushSample(t, x, y);
    publishState(t, x, y);
  }

  void pushSample(const rclcpp::Time & t, double x, double y)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EnemySample s; s.stamp = t; s.x = x; s.y = y;
    samples_.push_back(s);
    while (static_cast<int>(samples_.size()) > window_size_) samples_.pop_front();
    computeVelocityLocked();
  }

  void computeVelocityLocked()
  {
    // need at least 2 samples
    if (samples_.size() < 2) {
      last_vel_x_ = 0.0;
      last_vel_y_ = 0.0;
      last_confidence_ = 0.0;
      return;
    }
    // linear regression over time for x and y
    double t0 = samples_.front().stamp.seconds(); // double seconds
    int n = static_cast<int>(samples_.size());
    std::vector<double> ts(n), xs(n), ys(n);
    for (int i = 0; i < n; ++i) {
      ts[i] = samples_[i].stamp.seconds() - t0;
      xs[i] = samples_[i].x;
      ys[i] = samples_[i].y;
    }
    double sum_t = std::accumulate(ts.begin(), ts.end(), 0.0);
    double sum_t2 = 0.0;
    for (double v : ts) sum_t2 += v*v;
    double sum_x = std::accumulate(xs.begin(), xs.end(), 0.0);
    double sum_y = std::accumulate(ys.begin(), ys.end(), 0.0);
    double sum_tx = 0.0, sum_ty = 0.0;
    for (int i = 0; i < n; ++i) { sum_tx += ts[i]*xs[i]; sum_ty += ts[i]*ys[i]; }
    double denom = n * sum_t2 - sum_t * sum_t;
    double vx = 0.0, vy = 0.0;
    if (std::abs(denom) > 1e-6) {
      vx = (n * sum_tx - sum_t * sum_x) / denom;
      vy = (n * sum_ty - sum_t * sum_y) / denom;
    } else {
      // fallback to difference of last two
      const auto &a = samples_[n-1];
      const auto &b = samples_[n-2];
      double dt = a.stamp.seconds() - b.stamp.seconds();
      if (dt <= 0) { vx = 0.0; vy = 0.0; }
      else { vx = (a.x - b.x)/dt; vy = (a.y - b.y)/dt; }
    }
    // smoothing
    last_vel_x_ = smoothing_alpha_ * vx + (1.0 - smoothing_alpha_) * last_vel_x_;
    last_vel_y_ = smoothing_alpha_ * vy + (1.0 - smoothing_alpha_) * last_vel_y_;
    double speed = std::hypot(last_vel_x_, last_vel_y_);
    last_confidence_ = (speed >= min_velocity_threshold_) ? 1.0 : 0.5;
  }

  void publishState(const rclcpp::Time & t, double x, double y)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = t;
    ps.header.frame_id = frame_id_;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.w = 1.0;
    enemy_state_pub_->publish(ps);

    geometry_msgs::msg::TwistStamped ts;
    ts.header = ps.header;
    ts.twist.linear.x = last_vel_x_;
    ts.twist.linear.y = last_vel_y_;
    ts.twist.linear.z = 0.0;
    enemy_vel_pub_->publish(ts);

    std_msgs::msg::Float64 conf;
    conf.data = last_confidence_;
    enemy_conf_pub_->publish(conf);
  }

  // members
  rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr enemy_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr enemy_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr enemy_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr enemy_conf_pub_;

  std::deque<EnemySample> samples_;
  std::mutex mutex_;
  int window_size_ = 5;
  double smoothing_alpha_ = 0.6;
  double min_velocity_threshold_ = 0.02;
  double last_vel_x_ = 0.0;
  double last_vel_y_ = 0.0;
  double last_confidence_ = 0.0;
  std::string armors_topic_, enemy_pose_topic_, enemy_state_topic_, enemy_velocity_topic_, frame_id_;
};

} // namespace rm_behavior_tree

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rm_behavior_tree::EnemyStateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


