#ifndef SYNC_NODE_PKG__SYNC_NODE_V2_HPP_
#define SYNC_NODE_PKG__SYNC_NODE_V2_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <deque>
#include <mutex>
#include <atomic>
#include <chrono>
#include <string>

namespace sync_node_pkg
{

class SyncNodeV2 : public rclcpp::Node
{
public:
  SyncNodeV2();

private:
  using ImageMsg      = sensor_msgs::msg::Image;
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;
  using NavSatFixMsg  = sensor_msgs::msg::NavSatFix;
  using ImuMsg        = sensor_msgs::msg::Imu;
  using HeadingMsg    = geometry_msgs::msg::QuaternionStamped;
  using OdometryMsg   = nav_msgs::msg::Odometry;

  // ---- Callbacks ----
  void camera_callback(const ImageMsg::SharedPtr msg);
  void lidar_callback(const PointCloudMsg::SharedPtr msg);
  void gps_callback(const NavSatFixMsg::SharedPtr msg);
  void heading_callback(const HeadingMsg::SharedPtr msg);
  void imu_callback(const ImuMsg::SharedPtr msg);
  void depth_callback(const ImageMsg::SharedPtr msg);
  void odom_callback(const OdometryMsg::SharedPtr msg);

  // ---- Fusion Trigger ----
  void trigger_fusion(const rclcpp::Time & trigger_timestamp, const std::string & trigger_source);

  // ---- Closest Frame Matchers (Zero Loss) ----
  bool find_closest_camera(const rclcpp::Time & target, ImageMsg::ConstSharedPtr & result);
  bool find_closest_lidar(const rclcpp::Time & target, PointCloudMsg::ConstSharedPtr & result);
  bool find_closest_imu(const rclcpp::Time & target, ImuMsg::ConstSharedPtr & result);
  bool find_closest_depth(const rclcpp::Time & target, ImageMsg::ConstSharedPtr & result);
  bool find_closest_odom(const rclcpp::Time & target, OdometryMsg::ConstSharedPtr & result);
  
  // ---- GPS Interpolation ----
  bool get_gps_estimate(const rclcpp::Time & target, NavSatFixMsg::ConstSharedPtr & result, bool & is_interpolated);

  // ---- Publishers ----
  void publish_synced_output(
    const ImageMsg::ConstSharedPtr & camera_msg, bool camera_found,
    const PointCloudMsg::ConstSharedPtr & lidar_msg, bool lidar_found,
    const NavSatFixMsg::ConstSharedPtr & gps_msg, bool gps_found, bool gps_interpolated,
    const ImuMsg::ConstSharedPtr & imu_msg, bool imu_found,
    const HeadingMsg::ConstSharedPtr & heading_msg, bool heading_found,
    const ImageMsg::ConstSharedPtr & depth_msg, bool depth_found,
    const OdometryMsg::ConstSharedPtr & odom_msg, bool odom_found,
    const std::string & trigger_source,
    double fusion_latency_ms);

  void log_stats();

  // ---- Subscriptions ----
  rclcpp::Subscription<ImageMsg>::SharedPtr camera_sub_;
  rclcpp::Subscription<PointCloudMsg>::SharedPtr lidar_sub_;
  rclcpp::Subscription<NavSatFixMsg>::SharedPtr gps_sub_;
  rclcpp::Subscription<HeadingMsg>::SharedPtr heading_sub_;
  rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr depth_sub_;
  rclcpp::Subscription<OdometryMsg>::SharedPtr odom_sub_;

  // ---- Callback Groups ----
  rclcpp::CallbackGroup::SharedPtr camera_cb_group_;
  rclcpp::CallbackGroup::SharedPtr lidar_cb_group_;
  rclcpp::CallbackGroup::SharedPtr gps_cb_group_;
  rclcpp::CallbackGroup::SharedPtr imu_cb_group_;
  rclcpp::CallbackGroup::SharedPtr depth_cb_group_;
  rclcpp::CallbackGroup::SharedPtr odom_cb_group_;
  rclcpp::CallbackGroup::SharedPtr heading_cb_group_;

  // ---- Publishers ----
  rclcpp::Publisher<ImageMsg>::SharedPtr synced_camera_pub_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr synced_lidar_pub_;
  rclcpp::Publisher<NavSatFixMsg>::SharedPtr synced_gps_pub_;
  rclcpp::Publisher<ImuMsg>::SharedPtr synced_imu_pub_;
  rclcpp::Publisher<HeadingMsg>::SharedPtr synced_heading_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr synced_depth_pub_;
  rclcpp::Publisher<OdometryMsg>::SharedPtr synced_odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr loss_stats_pub_;

  // ---- Buffers ----
  std::deque<ImageMsg::ConstSharedPtr> camera_buffer_;
  std::deque<PointCloudMsg::ConstSharedPtr> lidar_buffer_;
  std::deque<NavSatFixMsg::ConstSharedPtr> gps_buffer_;
  std::deque<ImuMsg::ConstSharedPtr> imu_buffer_;
  std::deque<ImageMsg::ConstSharedPtr> depth_buffer_;
  std::deque<OdometryMsg::ConstSharedPtr> odom_buffer_;

  // ---- Heading (latest value) ----
  HeadingMsg::ConstSharedPtr latest_heading_;
  bool heading_received_{false};

  // ---- Mutexes ----
  std::mutex camera_mutex_;
  std::mutex lidar_mutex_;
  std::mutex gps_mutex_;
  std::mutex imu_mutex_;
  std::mutex depth_mutex_;
  std::mutex odom_mutex_;
  std::mutex heading_mutex_;
  std::mutex latency_stats_mutex_;
  std::mutex fusion_mutex_;

  // ---- Monotonic Guards (Camera & LiDAR only) ----
  rclcpp::Time last_published_camera_stamp_{0, 0, RCL_ROS_TIME};
  bool has_published_camera_{false};
  
  rclcpp::Time last_published_lidar_stamp_{0, 0, RCL_ROS_TIME};
  bool has_published_lidar_{false};

  // ---- Parameters ----
  double camera_stale_threshold_sec_;
  double lidar_stale_threshold_sec_;
  double imu_stale_threshold_sec_;
  double depth_stale_threshold_sec_;
  double odom_stale_threshold_sec_;
  size_t max_buffer_size_;
  size_t gps_buffer_size_;

  // ---- Metrics ----
  std::atomic<uint64_t> camera_received_{0};
  std::atomic<uint64_t> camera_published_{0};
  std::atomic<uint64_t> lidar_received_{0};
  std::atomic<uint64_t> lidar_published_{0};
  std::atomic<uint64_t> gps_received_{0};
  std::atomic<uint64_t> gps_published_{0};
  std::atomic<uint64_t> imu_received_{0};
  std::atomic<uint64_t> imu_published_{0};
  std::atomic<uint64_t> depth_received_{0};
  std::atomic<uint64_t> depth_published_{0};
  std::atomic<uint64_t> odom_received_{0};
  std::atomic<uint64_t> odom_published_{0};

  double latency_sum_ms_ = 0.0;
  int latency_count_ = 0;
  static constexpr int kLatencyLogInterval = 100;
  rclcpp::Time last_trigger_time_{0, 0, RCL_ROS_TIME};

  rclcpp::TimerBase::SharedPtr stats_timer_;
};

}  // namespace sync_node_pkg

#endif  // SYNC_NODE_PKG__SYNC_NODE_V2_HPP_
