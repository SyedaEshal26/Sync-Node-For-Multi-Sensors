#ifndef SYNC_NODE_PKG__SYNC_NODE_HPP_
#define SYNC_NODE_PKG__SYNC_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <deque>
#include <mutex>
#include <memory>
#include <atomic>
#include <string>
#include <fstream>
#include <limits>

namespace sync_node_pkg
{

class SyncNode : public rclcpp::Node
{
public:
  SyncNode();
  ~SyncNode();

private:

  // ============================================================
  // Message aliases
  // ============================================================

  using ImageMsg = sensor_msgs::msg::Image;
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;
  using NavSatFixMsg = sensor_msgs::msg::NavSatFix;
  using ImuMsg = sensor_msgs::msg::Imu;
  using OdometryMsg = nav_msgs::msg::Odometry;
  using HeadingMsg = geometry_msgs::msg::QuaternionStamped;

  // ============================================================
  // Callbacks
  // ============================================================

  void camera_callback(const ImageMsg::SharedPtr msg);
  void lidar_callback(const PointCloudMsg::SharedPtr msg);
  void gps_callback(const NavSatFixMsg::SharedPtr msg);
  void imu_callback(const ImuMsg::SharedPtr msg);
  void depth_callback(const ImageMsg::SharedPtr msg);
  void odom_callback(const OdometryMsg::SharedPtr msg);
  void heading_callback(const HeadingMsg::SharedPtr msg);

  // ============================================================
  // Fusion
  // ============================================================

  void trigger_fusion(
    const rclcpp::Time & timestamp,
    const std::string & trigger_source);

  // ============================================================
  // Matching
  // ============================================================

  bool find_closest_camera(
    const rclcpp::Time & target,
    ImageMsg::ConstSharedPtr & result);

  bool find_closest_lidar(
    const rclcpp::Time & target,
    PointCloudMsg::ConstSharedPtr & result);

  bool find_closest_imu(
    const rclcpp::Time & target,
    ImuMsg::ConstSharedPtr & result);

  bool find_closest_depth(
    const rclcpp::Time & target,
    ImageMsg::ConstSharedPtr & result);

  bool find_closest_odom(
    const rclcpp::Time & target,
    OdometryMsg::ConstSharedPtr & result);

  bool get_gps_estimate(
    const rclcpp::Time & target,
    NavSatFixMsg::ConstSharedPtr & result,
    bool & interpolated);

  // ============================================================
  // Statistics
  // ============================================================

  void log_stats();

  double timestamp_difference(
    const rclcpp::Time & a,
    const rclcpp::Time & b) const;

  // ============================================================
  // Subscriptions
  // ============================================================

  rclcpp::Subscription<ImageMsg>::SharedPtr camera_sub_;
  rclcpp::Subscription<PointCloudMsg>::SharedPtr lidar_sub_;
  rclcpp::Subscription<NavSatFixMsg>::SharedPtr gps_sub_;
  rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr depth_sub_;
  rclcpp::Subscription<OdometryMsg>::SharedPtr odom_sub_;
  rclcpp::Subscription<HeadingMsg>::SharedPtr heading_sub_;

  // ============================================================
  // Publishers
  // ============================================================

  rclcpp::Publisher<ImageMsg>::SharedPtr synced_camera_pub_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr synced_lidar_pub_;
  rclcpp::Publisher<NavSatFixMsg>::SharedPtr synced_gps_pub_;
  rclcpp::Publisher<ImuMsg>::SharedPtr synced_imu_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr synced_depth_pub_;
  rclcpp::Publisher<OdometryMsg>::SharedPtr synced_odom_pub_;
  rclcpp::Publisher<HeadingMsg>::SharedPtr synced_heading_pub_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr
    status_pub_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr
    loss_stats_pub_;

  // ============================================================
  // Callback groups
  // ============================================================

  rclcpp::CallbackGroup::SharedPtr camera_cb_group_;
  rclcpp::CallbackGroup::SharedPtr lidar_cb_group_;
  rclcpp::CallbackGroup::SharedPtr gps_cb_group_;
  rclcpp::CallbackGroup::SharedPtr imu_cb_group_;
  rclcpp::CallbackGroup::SharedPtr depth_cb_group_;
  rclcpp::CallbackGroup::SharedPtr odom_cb_group_;
  rclcpp::CallbackGroup::SharedPtr heading_cb_group_;

  // ============================================================
  // Buffers
  // ============================================================

  std::deque<ImageMsg::ConstSharedPtr> camera_buffer_;
  std::deque<PointCloudMsg::ConstSharedPtr> lidar_buffer_;
  std::deque<NavSatFixMsg::ConstSharedPtr> gps_buffer_;
  std::deque<ImuMsg::ConstSharedPtr> imu_buffer_;
  std::deque<ImageMsg::ConstSharedPtr> depth_buffer_;
  std::deque<OdometryMsg::ConstSharedPtr> odom_buffer_;

  std::mutex camera_mutex_;
  std::mutex lidar_mutex_;
  std::mutex gps_mutex_;
  std::mutex imu_mutex_;
  std::mutex depth_mutex_;
  std::mutex odom_mutex_;
  std::mutex heading_mutex_;
  std::mutex fusion_mutex_;

  // ============================================================
  // Latest heading
  // ============================================================

  HeadingMsg::ConstSharedPtr latest_heading_;
  bool heading_received_{false};

  // ============================================================
  // Parameters
  // ============================================================

  size_t max_buffer_size_;
  size_t gps_buffer_size_;

  double max_camera_delta_sec_;
  double max_lidar_delta_sec_;
  double max_imu_delta_sec_;
  double max_depth_delta_sec_;
  double max_odom_delta_sec_;
  double max_gps_delta_sec_;

  std::string output_directory_;

  // ============================================================
  // Trigger protection
  // ============================================================

  rclcpp::Time last_trigger_time_;
  bool has_last_trigger_{false};

  // ============================================================
  // Statistics
  //
  // IMPORTANT:
  // matched != sensor loss.
  //
  // IMU/Odom run faster than camera/LiDAR, therefore many received
  // messages will naturally not be selected for a fusion cycle.
  // ============================================================

  std::atomic<uint64_t> camera_received_{0};
  std::atomic<uint64_t> lidar_received_{0};
  std::atomic<uint64_t> gps_received_{0};
  std::atomic<uint64_t> imu_received_{0};
  std::atomic<uint64_t> depth_received_{0};
  std::atomic<uint64_t> odom_received_{0};
  std::atomic<uint64_t> heading_received_count_{0};

  std::atomic<uint64_t> camera_matched_{0};
  std::atomic<uint64_t> lidar_matched_{0};
  std::atomic<uint64_t> gps_matched_{0};
  std::atomic<uint64_t> imu_matched_{0};
  std::atomic<uint64_t> depth_matched_{0};
  std::atomic<uint64_t> odom_matched_{0};

  std::atomic<uint64_t> camera_buffer_drops_{0};
  std::atomic<uint64_t> lidar_buffer_drops_{0};
  std::atomic<uint64_t> gps_buffer_drops_{0};
  std::atomic<uint64_t> imu_buffer_drops_{0};
  std::atomic<uint64_t> depth_buffer_drops_{0};
  std::atomic<uint64_t> odom_buffer_drops_{0};

  std::atomic<uint64_t> total_fusions_{0};

  std::atomic<uint64_t> gps_direct_count_{0};
  std::atomic<uint64_t> gps_interpolated_count_{0};

  // ============================================================
  // Latency
  // ============================================================

  std::mutex latency_mutex_;

  double latency_sum_ms_{0.0};
  double latency_min_ms_{std::numeric_limits<double>::max()};
  double latency_max_ms_{0.0};
  uint64_t latency_count_{0};

  // ============================================================
  // CSV
  // ============================================================

  std::ofstream csv_file_;
  std::mutex csv_mutex_;

  rclcpp::TimerBase::SharedPtr stats_timer_;
};

}  // namespace sync_node_pkg

#endif  // SYNC_NODE_PKG__SYNC_NODE_HPP_
