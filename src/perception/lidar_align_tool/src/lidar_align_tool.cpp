// lidar_align_tool.cpp
//
// 一个独立的辅助节点：订阅两路 Livox 原始点云（CustomMsg），
// 把它们从各自的雷达坐标系变换到统一坐标系 "camera_init" 后发布，
// 并发布合并点云，便于在 RViz 中实时调节外参做对齐验证。
//
// 外参通过 ROS 2 参数声明，配合 rqt_reconfigure 使用滑动条实时调节，
// 无需重启节点。TF 也使用 dynamic_transform_publisher 实时广播，RViz
// 会同时把两个雷达坐标系拉到正确位置，无需手动配置 TF。
//
// 发布话题：
//   /lidar1_raw   (livox1 原始，frame = livox_frame1)
//   /lidar2_raw   (livox2 原始，frame = livox_frame2)
//   /lidar1_aligned (变换后, frame = camera_init)
//   /lidar2_aligned (变换后, frame = camera_init)
//   /merged_aligned (合并点云, frame = camera_init, intensity 用于区分雷达：88/89)

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class LidarAlignTool : public rclcpp::Node
{
public:
  LidarAlignTool() : Node("lidar_align_tool")
  {
    // ========== 订阅原始 Livox 话题 ==========
    cloud1_sub_.subscribe(this, "livox/lidar_192_168_1_150");
    cloud2_sub_.subscribe(this, "livox/lidar_192_168_1_149");

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), cloud1_sub_, cloud2_sub_);
    sync_->registerCallback(
      std::bind(&LidarAlignTool::syncCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    // ========== 发布器 ==========
    raw1_pub_       = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar1_raw",       10);
    raw2_pub_       = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar2_raw",       10);
    aligned1_pub_   = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar1_aligned",  10);
    aligned2_pub_   = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar2_aligned",  10);
    merged_pub_     = this->create_publisher<sensor_msgs::msg::PointCloud2>("merged_aligned",  10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // ========== 外参参数（带滑动条范围）==========
    declareExtrinsicParams();

    // 基础参数
    this->declare_parameter<std::string>("frame_target", "camera_init");
    this->declare_parameter<std::string>("frame_lidar1", "livox_frame1");
    this->declare_parameter<std::string>("frame_lidar2", "livox_frame2");
    this->declare_parameter<int64_t>("max_time_diff_ms", 50);

    updateTransformParams();

    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&LidarAlignTool::onParameterChanged, this, std::placeholders::_1));

    // 周期广播 TF（让 RViz 随时看到坐标系的当前位置）
    tf_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LidarAlignTool::broadcastTf, this));

    RCLCPP_INFO(this->get_logger(),
      "lidar_align_tool ready. Open rqt_reconfigure to tune extrinsics live.");
  }

private:
  // 工具：根据 (default, range) 构造可显示为滑动条的参数描述符
  static rcl_interfaces::msg::ParameterDescriptor makeFloatDescriptor(
    double min_val, double max_val, double step, const std::string & description)
  {
    auto d = rcl_interfaces::msg::ParameterDescriptor();
    d.description = description;
    d.read_only = false;
    d.floating_point_range.resize(1);
    d.floating_point_range[0].from_value = min_val;
    d.floating_point_range[0].to_value   = max_val;
    d.floating_point_range[0].step       = step;
    return d;
  }

  void declareExtrinsicParams()
  {
    // ===== Lidar1 → camera_init =====
    this->declare_parameter<double>("roll1",  0.0,
      makeFloatDescriptor(-180.0, 180.0, 0.01, "Lidar1 roll (deg)"));
    this->declare_parameter<double>("pitch1", 0.0,
      makeFloatDescriptor(-180.0, 180.0, 0.01, "Lidar1 pitch (deg)"));
    this->declare_parameter<double>("yaw1",   0.0,
      makeFloatDescriptor(-360.0, 360.0, 0.01, "Lidar1 yaw (deg)"));
    this->declare_parameter<double>("tx1",    0.2310,
      makeFloatDescriptor(-2.0, 2.0, 0.0001, "Lidar1 X translation (m)"));
    this->declare_parameter<double>("ty1",    0.0,
      makeFloatDescriptor(-2.0, 2.0, 0.0001, "Lidar1 Y translation (m)"));
    this->declare_parameter<double>("tz1",    0.2867,
      makeFloatDescriptor(-2.0, 2.0, 0.0001, "Lidar1 Z translation (m)"));

    // ===== Lidar2 → camera_init =====
    this->declare_parameter<double>("roll2",  1.70,
      makeFloatDescriptor(-180.0, 180.0, 0.01, "Lidar2 roll (deg)"));
    this->declare_parameter<double>("pitch2", 76.70,
      makeFloatDescriptor(-90.0, 90.0, 0.01, "Lidar2 pitch (deg)"));
    this->declare_parameter<double>("yaw2",   182.0,
      makeFloatDescriptor(-360.0, 360.0, 0.01, "Lidar2 yaw (deg)"));
    this->declare_parameter<double>("tx2",    -0.2201,
      makeFloatDescriptor(-2.0, 2.0, 0.0001, "Lidar2 X translation (m)"));
    this->declare_parameter<double>("ty2",    0.0,
      makeFloatDescriptor(-2.0, 2.0, 0.0001, "Lidar2 Y translation (m)"));
    this->declare_parameter<double>("tz2",    0.1360,
      makeFloatDescriptor(-2.0, 2.0, 0.0001, "Lidar2 Z translation (m)"));
  }

  static double deg2rad(double d) { return d * M_PI / 180.0; }
  static double rad2deg(double r) { return r * 180.0 / M_PI; }

  void updateTransformParams()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    roll1_  = deg2rad(this->get_parameter("roll1").as_double());
    pitch1_ = deg2rad(this->get_parameter("pitch1").as_double());
    yaw1_   = deg2rad(this->get_parameter("yaw1").as_double());
    tx1_    = static_cast<float>(this->get_parameter("tx1").as_double());
    ty1_    = static_cast<float>(this->get_parameter("ty1").as_double());
    tz1_    = static_cast<float>(this->get_parameter("tz1").as_double());

    roll2_  = deg2rad(this->get_parameter("roll2").as_double());
    pitch2_ = deg2rad(this->get_parameter("pitch2").as_double());
    yaw2_   = deg2rad(this->get_parameter("yaw2").as_double());
    tx2_    = static_cast<float>(this->get_parameter("tx2").as_double());
    ty2_    = static_cast<float>(this->get_parameter("ty2").as_double());
    tz2_    = static_cast<float>(this->get_parameter("tz2").as_double());

    transform1_ = Eigen::Matrix4f::Identity();
    transform2_ = Eigen::Matrix4f::Identity();
    setTransformMatrix(transform1_, roll1_, pitch1_, yaw1_, tx1_, ty1_, tz1_);
    setTransformMatrix(transform2_, roll2_, pitch2_, yaw2_, tx2_, ty2_, tz2_);

    RCLCPP_INFO(this->get_logger(),
      "Lidar1 -> %s: rpy=(%.2f, %.2f, %.2f) deg, t=(%.4f, %.4f, %.4f) m",
      this->get_parameter("frame_target").as_string().c_str(),
      rad2deg(roll1_), rad2deg(pitch1_), rad2deg(yaw1_), tx1_, ty1_, tz1_);
    RCLCPP_INFO(this->get_logger(),
      "Lidar2 -> %s: rpy=(%.2f, %.2f, %.2f) deg, t=(%.4f, %.4f, %.4f) m",
      this->get_parameter("frame_target").as_string().c_str(),
      rad2deg(roll2_), rad2deg(pitch2_), rad2deg(yaw2_), tx2_, ty2_, tz2_);
  }

  rcl_interfaces::msg::SetParametersResult onParameterChanged(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "ok";

    static const std::vector<std::string> extrinsic_names = {
      "roll1", "pitch1", "yaw1", "tx1", "ty1", "tz1",
      "roll2", "pitch2", "yaw2", "tx2", "ty2", "tz2",
    };
    for (const auto & p : params) {
      for (const auto & n : extrinsic_names) {
        if (p.get_name() == n) {
          updateTransformParams();
          break;
        }
      }
    }
    return result;
  }

  void setTransformMatrix(Eigen::Matrix4f & transform,
                          float roll, float pitch, float yaw,
                          float tx, float ty, float tz)
  {
    Eigen::AngleAxisf roll_angle(roll,  Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitch_angle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yaw_angle(yaw,    Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf q = yaw_angle * pitch_angle * roll_angle;
    transform.block<3,3>(0,0) = q.matrix();
    transform(0,3) = tx;
    transform(1,3) = ty;
    transform(2,3) = tz;
  }

  // 把矩阵转换为 tf2 的四元数
  static void matrixToQuaternion(const Eigen::Matrix4f & m,
                                  geometry_msgs::msg::Quaternion & q,
                                  double & tx, double & ty, double & tz)
  {
    Eigen::Matrix3f R = m.block<3,3>(0,0);
    Eigen::Quaternionf qf(R);
    q.x = qf.x(); q.y = qf.y(); q.z = qf.z(); q.w = qf.w();
    tx = m(0,3); ty = m(1,3); tz = m(2,3);
  }

  void broadcastTf()
  {
    Eigen::Matrix4f T1, T2;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      T1 = transform1_;
      T2 = transform2_;
    }

    const auto stamp = this->now();

    // Lidar1 frame → camera_init
    {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp    = stamp;
      t.header.frame_id = this->get_parameter("frame_target").as_string();
      t.child_frame_id  = this->get_parameter("frame_lidar1").as_string();
      matrixToQuaternion(T1, t.transform.rotation,
                         t.transform.translation.x,
                         t.transform.translation.y,
                         t.transform.translation.z);
      tf_broadcaster_->sendTransform(t);
    }
    // Lidar2 frame → camera_init
    {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp    = stamp;
      t.header.frame_id = this->get_parameter("frame_target").as_string();
      t.child_frame_id  = this->get_parameter("frame_lidar2").as_string();
      matrixToQuaternion(T2, t.transform.rotation,
                         t.transform.translation.x,
                         t.transform.translation.y,
                         t.transform.translation.z);
      tf_broadcaster_->sendTransform(t);
    }
  }

  void syncCallback(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg1,
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg2)
  {
    const int64_t max_diff_ns =
      this->get_parameter("max_time_diff_ms").as_int() * 1000000LL;

    const auto t1_ns = msg1->header.stamp.sec * 1000000000LL + msg1->header.stamp.nanosec;
    const auto t2_ns = msg2->header.stamp.sec * 1000000000LL + msg2->header.stamp.nanosec;
    if (std::abs(t1_ns - t2_ns) > max_diff_ns) {
      return;  // 时间差过大，跳过
    }

    // 复制外参（避免长时间锁等待）
    Eigen::Matrix4f T1, T2;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      T1 = transform1_;
      T2 = transform2_;
    }

    // 把 CustomMsg 转换成 PCL PointXYZI
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1_raw(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2_raw(new pcl::PointCloud<pcl::PointXYZI>);
    convertCustomToPcl(msg1, cloud1_raw, /*tag=*/88);  // 用 intensity 区分：88 -> lidar1
    convertCustomToPcl(msg2, cloud2_raw, /*tag=*/89);  //                       89 -> lidar2

    // 发布原始点云（在各自传感器坐标系里）
    publishPointCloud2(cloud1_raw, msg1->header.stamp,
                       this->get_parameter("frame_lidar1").as_string(), raw1_pub_);
    publishPointCloud2(cloud2_raw, msg2->header.stamp,
                       this->get_parameter("frame_lidar2").as_string(), raw2_pub_);

    // 应用实时外参
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1_aligned(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2_aligned(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*cloud1_raw, *cloud1_aligned, T1);
    pcl::transformPointCloud(*cloud2_raw, *cloud2_aligned, T2);

    const std::string target_frame = this->get_parameter("frame_target").as_string();
    publishPointCloud2(cloud1_aligned, msg1->header.stamp, target_frame, aligned1_pub_);
    publishPointCloud2(cloud2_aligned, msg2->header.stamp, target_frame, aligned2_pub_);

    // 合并对齐后的点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>);
    *merged = *cloud1_aligned + *cloud2_aligned;
    publishPointCloud2(merged, msg1->header.stamp, target_frame, merged_pub_);
  }

  static void convertCustomToPcl(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg,
    pcl::PointCloud<pcl::PointXYZI>::Ptr & out,
    uint8_t tag_intensity)
  {
    out->clear();
    out->reserve(msg->points.size());
    for (const auto & p : msg->points) {
      pcl::PointXYZI pt;
      pt.x = p.x; pt.y = p.y; pt.z = p.z;
      pt.intensity = tag_intensity;  // 用 intensity 标记点来自哪个雷达
      out->push_back(pt);
    }
    out->width  = out->size();
    out->height = 1;
    out->is_dense = true;
  }

  void publishPointCloud2(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & frame_id,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub)
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    pub->publish(msg);
  }

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    livox_ros_driver2::msg::CustomMsg, livox_ros_driver2::msg::CustomMsg>;
  message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> cloud1_sub_;
  message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> cloud2_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::mutex mutex_;
  Eigen::Matrix4f transform1_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f transform2_{Eigen::Matrix4f::Identity()};
  float roll1_, pitch1_, yaw1_, tx1_, ty1_, tz1_;
  float roll2_, pitch2_, yaw2_, tx2_, ty2_, tz2_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarAlignTool>());
  rclcpp::shutdown();
  return 0;
}
