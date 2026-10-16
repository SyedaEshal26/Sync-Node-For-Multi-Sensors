#ifndef SYNC_NODE_PKG__SYNC_NODE_HPP_
#define SYNC_NODE_PKG__SYNC_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <deque>
#include <mutex>
#include <memory>
#include <atomic>
#include <string>
#include <fstream>
#include <limits>
#include <cmath>
#include <cstdint>

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
  using VelMsg = geometry_msgs::msg::TwistStamped;
  using CameraInfoMsg = sensor_msgs::msg::CameraInfo;

  // ============================================================
  // LiDAR buffer entry. The lidar stamp is already in the same
  // time domain as the ZED, so no clock correction is needed.
  // ============================================================

  struct LidarEntry
  {
    PointCloudMsg::ConstSharedPtr msg;
    bool used{false};          // already matched to a fusion at least once
  };

  // ============================================================
  // Callbacks - NO triggers except depth
  // ============================================================

  void camera_callback(const ImageMsg::SharedPtr msg);
  void lidar_callback(const PointCloudMsg::SharedPtr msg);
  void gps_callback(const NavSatFixMsg::SharedPtr msg);
  void imu_callback(const ImuMsg::SharedPtr msg);
  void depth_callback(const ImageMsg::SharedPtr msg);
  void odom_callback(const OdometryMsg::SharedPtr msg);
  void vel_callback(const VelMsg::SharedPtr msg);
  void rgb_raw_camera_info_callback(const CameraInfoMsg::SharedPtr msg);
  void stereo_callback(const ImageMsg::SharedPtr msg);

  // ============================================================
  // Fusion - Depth ONLY trigger
  // ============================================================

  void trigger_fusion(
    const rclcpp::Time & timestamp,
    const std::string & trigger_source);

  // ============================================================
  // Matching - All sensors match to depth timestamp
  // (GPS and vel use HOLD mode: latest received value,
  //  no time-delta cutoff - see trigger_fusion())
  // ============================================================

  bool find_closest_camera(
    const rclcpp::Time & target,
    ImageMsg::ConstSharedPtr & result);

  // With lidar_consume_on_match=true (default) only the matched cloud is
  // removed (other unused clouds stay for the next fusion), so no scan is
  // ever published twice. With lidar_consume_on_match=false the nearest cloud
  // is kept in the buffer, so two depth frames closer together than the lidar
  // period still both get a cloud (counted in lidar_reused_). Clouds too old
  // to match any future depth frame are pruned; the ones never matched are
  // counted in lidar_skipped_.
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

  bool find_closest_rgb_raw_camera_info(
    const rclcpp::Time & target,
    CameraInfoMsg::ConstSharedPtr & result);

  bool find_closest_stereo(
    const rclcpp::Time & target,
    ImageMsg::ConstSharedPtr & result);

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
  rclcpp::Subscription<VelMsg>::SharedPtr vel_sub_;
  rclcpp::Subscription<CameraInfoMsg>::SharedPtr rgb_raw_camera_info_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr stereo_sub_;

  // ============================================================
  // Publishers
  // ============================================================

  rclcpp::Publisher<ImageMsg>::SharedPtr synced_camera_pub_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr synced_lidar_pub_;
  rclcpp::Publisher<NavSatFixMsg>::SharedPtr synced_gps_pub_;
  rclcpp::Publisher<ImuMsg>::SharedPtr synced_imu_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr synced_depth_pub_;
  rclcpp::Publisher<OdometryMsg>::SharedPtr synced_odom_pub_;
  rclcpp::Publisher<VelMsg>::SharedPtr synced_vel_pub_;
  rclcpp::Publisher<CameraInfoMsg>::SharedPtr synced_rgb_raw_camera_info_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr synced_stereo_pub_;

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
  rclcpp::CallbackGroup::SharedPtr vel_cb_group_;
  rclcpp::CallbackGroup::SharedPtr rgb_raw_camera_info_cb_group_;
  rclcpp::CallbackGroup::SharedPtr stereo_cb_group_;

  // ============================================================
  // Buffers
  // (GPS and vel are NOT buffered - they use single
  //  "latest value" hold-mode slots instead, see below)
  // ============================================================

  std::deque<ImageMsg::ConstSharedPtr> camera_buffer_;
  std::deque<LidarEntry> lidar_buffer_;
  std::deque<ImuMsg::ConstSharedPtr> imu_buffer_;
  std::deque<ImageMsg::ConstSharedPtr> depth_buffer_;
  std::deque<OdometryMsg::ConstSharedPtr> odom_buffer_;
  std::deque<CameraInfoMsg::ConstSharedPtr> rgb_raw_camera_info_buffer_;
  std::deque<ImageMsg::ConstSharedPtr> stereo_buffer_;

  std::mutex camera_mutex_;
  std::mutex lidar_mutex_;
  std::mutex gps_mutex_;
  std::mutex imu_mutex_;
  std::mutex depth_mutex_;
  std::mutex odom_mutex_;
  std::mutex vel_mutex_;
  std::mutex rgb_raw_camera_info_mutex_;
  std::mutex stereo_mutex_;
  std::mutex fusion_mutex_;

  // ============================================================
  // Hold-mode latest values (GPS + vel)
  //
  // Both repeat their most recently received value on every
  // depth-triggered fusion until a new message arrives. No
  // time-delta cutoff is applied - staleness is only reported
  // via *_delta_ms in the CSV / diagnostics, never used to
  // drop the value from the fused output.
  // ============================================================

  NavSatFixMsg::ConstSharedPtr latest_gps_;
  bool gps_received_flag_{false};

  // Last GPS message actually consumed by a fusion cycle - used to
  // detect whether a given fusion got a fresh fix or reused (held) one
  NavSatFixMsg::ConstSharedPtr last_fused_gps_;

  VelMsg::ConstSharedPtr latest_vel_;
  bool vel_received_{false};

  // ============================================================
  // Parameters
  // ============================================================

  size_t max_buffer_size_;

  double max_camera_delta_sec_;
  double max_lidar_delta_sec_;
  double max_imu_delta_sec_;
  double max_depth_delta_sec_;
  double max_odom_delta_sec_;
  double max_rgb_raw_camera_info_delta_sec_;
  double max_stereo_delta_sec_;

  // true (default): each lidar cloud is matched at most once.
  bool lidar_consume_on_match_;

  std::string output_directory_;

  // ============================================================
  // Trigger protection
  // ============================================================

  rclcpp::Time last_trigger_time_;
  bool has_last_trigger_{false};

  // ============================================================
  // Statistics
  // ============================================================

  std::atomic<uint64_t> camera_received_{0};
  std::atomic<uint64_t> lidar_received_{0};
  std::atomic<uint64_t> gps_received_{0};
  std::atomic<uint64_t> imu_received_{0};
  std::atomic<uint64_t> depth_received_{0};
  std::atomic<uint64_t> odom_received_{0};
  std::atomic<uint64_t> vel_received_count_{0};
  std::atomic<uint64_t> rgb_raw_camera_info_received_{0};
  std::atomic<uint64_t> stereo_received_{0};

  std::atomic<uint64_t> camera_matched_{0};
  std::atomic<uint64_t> lidar_matched_{0};
  std::atomic<uint64_t> gps_matched_{0};
  std::atomic<uint64_t> gps_reused_{0};
  std::atomic<uint64_t> imu_matched_{0};
  std::atomic<uint64_t> depth_matched_{0};
  std::atomic<uint64_t> odom_matched_{0};
  std::atomic<uint64_t> vel_matched_{0};
  std::atomic<uint64_t> rgb_raw_camera_info_matched_{0};
  std::atomic<uint64_t> stereo_matched_{0};

  std::atomic<uint64_t> camera_buffer_drops_{0};
  std::atomic<uint64_t> lidar_buffer_drops_{0};
  std::atomic<uint64_t> lidar_skipped_{0};      // clouds that were never matched to any fusion
  std::atomic<uint64_t> lidar_reused_{0};       // fusions that got a cloud an earlier fusion already used
  std::atomic<uint64_t> lidar_miss_empty_{0};   // fusion found the lidar buffer empty
  std::atomic<uint64_t> lidar_miss_far_{0};     // closest cloud was beyond max_lidar_delta_sec
  std::atomic<int64_t> lidar_worst_miss_ns_{0}; // largest 'too far' delta seen
  std::atomic<uint64_t> imu_buffer_drops_{0};
  std::atomic<uint64_t> depth_buffer_drops_{0};
  std::atomic<uint64_t> odom_buffer_drops_{0};
  std::atomic<uint64_t> rgb_raw_camera_info_buffer_drops_{0};
  std::atomic<uint64_t> stereo_buffer_drops_{0};

  std::atomic<uint64_t> total_fusions_{0};

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
