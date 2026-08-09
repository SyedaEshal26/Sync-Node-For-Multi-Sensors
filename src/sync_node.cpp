#include "sync_node_pkg/sync_node.hpp"

#include <algorithm>
#include <cmath>

namespace sync_node_pkg
{

SyncNode::SyncNode()
: Node("sync_node")
{
  this->declare_parameter<double>("camera_stale_threshold_sec", 0.1);
  this->declare_parameter<double>("lidar_stale_threshold_sec", 0.1);
  this->declare_parameter<int>("max_buffer_size", 30);
  this->declare_parameter<int>("gps_buffer_size", 20);

  camera_stale_threshold_sec_ = this->get_parameter("camera_stale_threshold_sec").as_double();
  lidar_stale_threshold_sec_  = this->get_parameter("lidar_stale_threshold_sec").as_double();
  max_buffer_size_ = static_cast<size_t>(this->get_parameter("max_buffer_size").as_int());
  gps_buffer_size_ = static_cast<size_t>(this->get_parameter("gps_buffer_size").as_int());

  camera_cb_group_   = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  lidar_cb_group_    = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  gps_cb_group_      = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  zed_core_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions camera_opts; camera_opts.callback_group = camera_cb_group_;
  rclcpp::SubscriptionOptions lidar_opts;  lidar_opts.callback_group  = lidar_cb_group_;
  rclcpp::SubscriptionOptions gps_opts;    gps_opts.callback_group    = gps_cb_group_;
  rclcpp::SubscriptionOptions zed_core_opts; zed_core_opts.callback_group = zed_core_cb_group_;

  camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/zed/zed_node/rgb/image_rect_color", rclcpp::SensorDataQoS(),
    std::bind(&SyncNode::camera_callback, this, std::placeholders::_1), camera_opts);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/rslidar_points", rclcpp::SensorDataQoS(),
    std::bind(&SyncNode::lidar_callback, this, std::placeholders::_1), lidar_opts);

  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/fix", rclcpp::SensorDataQoS(),
    std::bind(&SyncNode::gps_callback, this, std::placeholders::_1), gps_opts);

  heading_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
    "/heading", rclcpp::SensorDataQoS(),
    std::bind(&SyncNode::heading_callback, this, std::placeholders::_1), gps_opts);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/zed/zed_node/imu/data", rclcpp::SensorDataQoS(),
    std::bind(&SyncNode::imu_callback, this, std::placeholders::_1), zed_core_opts);

  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/zed/zed_node/depth/depth_registered", rclcpp::SensorDataQoS(),
    std::bind(&SyncNode::depth_callback, this, std::placeholders::_1), zed_core_opts);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/zed/zed_node/odom", rclcpp::SensorDataQoS(),
    std::bind(&SyncNode::odom_callback, this, std::placeholders::_1), zed_core_opts);

  synced_camera_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/synced/camera", 10);
  synced_lidar_pub_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("/synced/lidar", 10);
  synced_gps_pub_    = this->create_publisher<sensor_msgs::msg::NavSatFix>("/synced/gps", 10);
  synced_imu_pub_    = this->create_publisher<sensor_msgs::msg::Imu>("/synced/zed/imu", 10);
  synced_heading_pub_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>("/synced/heading", 10);
  synced_depth_pub_  = this->create_publisher<sensor_msgs::msg::Image>("/synced/zed/depth_registered", 10);
  synced_odom_pub_   = this->create_publisher<nav_msgs::msg::Odometry>("/synced/zed/odom", 10);
  status_pub_        = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/synced/status", 10);
  loss_stats_pub_    = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/synced/loss_stats", 1);

  stats_timer_ = this->create_wall_timer(
    std::chrono::seconds(5), std::bind(&SyncNode::log_stats, this));

  RCLCPP_INFO(this->get_logger(),
    "SyncNode Active: single camera-driven trigger, zero-copy buffers, per-modality monotonic guard.");
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

void SyncNode::camera_callback(const ImageMsg::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    camera_buffer_.push_back(msg);  // shared_ptr copy only — no payload copy
    if (camera_buffer_.size() > max_buffer_size_) camera_buffer_.pop_front();
  }
  camera_received_++;

  // Camera is the sole fusion trigger. camera_callback() is serialized
  // against itself by its own MutuallyExclusive callback group, so
  // trigger_fusion() can never run concurrently with itself and needs no
  // extra global lock.
  trigger_fusion(msg->header.stamp, "camera");
}

void SyncNode::lidar_callback(const PointCloudMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(lidar_mutex_);
  lidar_buffer_.push_back(msg);
  if (lidar_buffer_.size() > max_buffer_size_) lidar_buffer_.pop_front();
  lidar_received_++;
  // No trigger_fusion() call here anymore — lidar only feeds the buffer that
  // the camera-driven trigger reads from.
}

void SyncNode::gps_callback(const NavSatFixMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(gps_mutex_);
  gps_buffer_.push_back(msg);
  if (gps_buffer_.size() > gps_buffer_size_) gps_buffer_.pop_front();
  gps_received_++;
  // No trigger_fusion() call here either — same reasoning as lidar.
}

void SyncNode::heading_callback(const HeadingMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(heading_mutex_);
  latest_heading_ = msg; heading_received_ = true;
}

void SyncNode::imu_callback(const ImuMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  latest_imu_ = msg; imu_received_ = true;
}

void SyncNode::depth_callback(const ImageMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(depth_mutex_);
  latest_depth_ = msg; depth_received_ = true;
}

void SyncNode::odom_callback(const OdometryMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_ = msg; odom_received_ = true;
}

// ---------------------------------------------------------------------------
// Robust Anti-Jitter Lookups (With Pruning & Monotonic Checks)
// ---------------------------------------------------------------------------

bool SyncNode::find_closest_camera(
  const rclcpp::Time & target, ImageMsg::ConstSharedPtr & result, bool & is_stale)
{
  std::lock_guard<std::mutex> lock(camera_mutex_);
  if (camera_buffer_.empty()) return false;

  // 1. Drop outdated frames older than target window
  while (camera_buffer_.size() > 1) {
    double age = (target - rclcpp::Time(camera_buffer_.front()->header.stamp)).seconds();
    if (age > camera_stale_threshold_sec_) {
      camera_buffer_.pop_front();
    } else {
      break;
    }
  }

  // 2. Find closest frame
  auto closest = std::min_element(
    camera_buffer_.begin(), camera_buffer_.end(),
    [&](const ImageMsg::ConstSharedPtr & a, const ImageMsg::ConstSharedPtr & b) {
      return std::abs((rclcpp::Time(a->header.stamp) - target).seconds()) <
             std::abs((rclcpp::Time(b->header.stamp) - target).seconds());
    });

  rclcpp::Time match_stamp((*closest)->header.stamp);

  // 3. Monotonicity Guard: NEVER publish an older/equal frame than what we
  //    already published. Checked AND updated here, atomically, under
  //    camera_mutex_ — no separate lock or race window against publish.
  if (has_published_camera_ && match_stamp <= last_published_camera_stamp_) {
    return false; // Skip old frames completely to avoid stream glitching
  }

  double diff = std::abs((match_stamp - target).seconds());
  is_stale = (diff > camera_stale_threshold_sec_);
  result = *closest;  // shared_ptr copy — no payload copy

  last_published_camera_stamp_ = match_stamp;
  has_published_camera_ = true;

  // 4. Prune Buffer: Remove all frames up to matched frame so buffer stays fresh
  while (!camera_buffer_.empty() && camera_buffer_.begin() != closest) {
    camera_buffer_.pop_front();
  }

  return true;
}

bool SyncNode::find_closest_lidar(
  const rclcpp::Time & target, PointCloudMsg::ConstSharedPtr & result, bool & is_stale)
{
  std::lock_guard<std::mutex> lock(lidar_mutex_);
  if (lidar_buffer_.empty()) return false;

  // 1. Drop outdated frames
  while (lidar_buffer_.size() > 1) {
    double age = (target - rclcpp::Time(lidar_buffer_.front()->header.stamp)).seconds();
    if (age > lidar_stale_threshold_sec_) {
      lidar_buffer_.pop_front();
    } else {
      break;
    }
  }

  // 2. Find closest frame
  auto closest = std::min_element(
    lidar_buffer_.begin(), lidar_buffer_.end(),
    [&](const PointCloudMsg::ConstSharedPtr & a, const PointCloudMsg::ConstSharedPtr & b) {
      return std::abs((rclcpp::Time(a->header.stamp) - target).seconds()) <
             std::abs((rclcpp::Time(b->header.stamp) - target).seconds());
    });

  rclcpp::Time match_stamp((*closest)->header.stamp);

  // 3. Monotonicity Guard (checked + updated atomically under lidar_mutex_)
  if (has_published_lidar_ && match_stamp <= last_published_lidar_stamp_) {
    return false;
  }

  double diff = std::abs((match_stamp - target).seconds());
  is_stale = (diff > lidar_stale_threshold_sec_);
  result = *closest;  // shared_ptr copy — no payload copy (avoids copying a full PointCloud2)

  last_published_lidar_stamp_ = match_stamp;
  has_published_lidar_ = true;

  // 4. Prune Buffer
  while (!lidar_buffer_.empty() && lidar_buffer_.begin() != closest) {
    lidar_buffer_.pop_front();
  }

  return true;
}

bool SyncNode::get_gps_estimate(
  const rclcpp::Time & target, NavSatFixMsg::ConstSharedPtr & result, bool & is_interpolated)
{
  std::lock_guard<std::mutex> lock(gps_mutex_);
  if (gps_buffer_.empty()) return false;

  // Prune old GPS points older than 5 sec
  while (gps_buffer_.size() > 2) {
    if ((target - rclcpp::Time(gps_buffer_.front()->header.stamp)).seconds() > 5.0) {
      gps_buffer_.pop_front();
    } else {
      break;
    }
  }

  if (gps_buffer_.size() < 2) {
    result = gps_buffer_.back();  // shared_ptr copy, no data copy
    is_interpolated = false;
    return true;
  }

  const auto & older = gps_buffer_[gps_buffer_.size() - 2];
  const auto & newer = gps_buffer_.back();

  double t_old = rclcpp::Time(older->header.stamp).seconds();
  double t_new = rclcpp::Time(newer->header.stamp).seconds();
  double t_tgt = target.seconds();

  if (t_tgt >= t_new) {
    result = newer;  // no interpolation needed — no data copy
    is_interpolated = false;
    return true;
  }

  double ratio = (t_new > t_old) ? (t_tgt - t_old) / (t_new - t_old) : 0.0;
  ratio = std::clamp(ratio, 0.0, 1.0);

  // Interpolation genuinely needs a new message, so this is the one place we
  // pay for a copy — unavoidable since we're synthesizing new data.
  auto interpolated = std::make_shared<NavSatFixMsg>(*newer);
  interpolated->latitude  = older->latitude  + ratio * (newer->latitude  - older->latitude);
  interpolated->longitude = older->longitude + ratio * (newer->longitude - older->longitude);
  interpolated->altitude  = older->altitude  + ratio * (newer->altitude  - older->altitude);
  result = interpolated;
  is_interpolated = (ratio > 0.0 && ratio < 1.0);
  return true;
}

// ---------------------------------------------------------------------------
// Fusion Execution
// ---------------------------------------------------------------------------

void SyncNode::trigger_fusion(const rclcpp::Time & trigger_timestamp, const std::string & trigger_source)
{
  // No fusion_mutex_ here — this function is only called from
  // camera_callback(), which is already serialized against itself by its
  // own MutuallyExclusive callback group. lidar/gps callbacks no longer
  // call this function at all, so there is nothing left to serialize
  // against.
  auto t_start = std::chrono::steady_clock::now();

  ImageMsg::ConstSharedPtr matched_camera;
  bool camera_stale = false;
  bool camera_found = find_closest_camera(trigger_timestamp, matched_camera, camera_stale);

  PointCloudMsg::ConstSharedPtr matched_lidar;
  bool lidar_stale = false;
  bool lidar_found = find_closest_lidar(trigger_timestamp, matched_lidar, lidar_stale);

  NavSatFixMsg::ConstSharedPtr gps_output;
  bool gps_interpolated = false;
  bool gps_found = get_gps_estimate(trigger_timestamp, gps_output, gps_interpolated);

  ImuMsg::ConstSharedPtr imu_copy;
  bool imu_found = false;
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (imu_received_) { imu_copy = latest_imu_; imu_found = true; }
  }

  HeadingMsg::ConstSharedPtr heading_copy;
  bool heading_found = false;
  {
    std::lock_guard<std::mutex> lock(heading_mutex_);
    if (heading_received_) { heading_copy = latest_heading_; heading_found = true; }
  }

  ImageMsg::ConstSharedPtr depth_copy;
  bool depth_found = false;
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    if (depth_received_) { depth_copy = latest_depth_; depth_found = true; }
  }

  OdometryMsg::ConstSharedPtr odom_copy;
  bool odom_found = false;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (odom_received_) { odom_copy = latest_odom_; odom_found = true; }
  }

  auto t_end = std::chrono::steady_clock::now();
  double latency_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  publish_synced_output(
    matched_camera, camera_found, camera_stale,
    matched_lidar, lidar_found, lidar_stale,
    gps_output, gps_found, gps_interpolated,
    imu_copy, imu_found,
    heading_copy, heading_found,
    depth_copy, depth_found,
    odom_copy, odom_found,
    trigger_source, latency_ms);

  latency_sum_ms_ += latency_ms;
  latency_count_++;
  if (latency_count_ >= kLatencyLogInterval) {
    RCLCPP_INFO(this->get_logger(), "Avg fusion latency over last %d cycles: %.3f ms",
      latency_count_, (latency_sum_ms_ / latency_count_));
    latency_sum_ms_ = 0.0;
    latency_count_ = 0;
  }
}

// ---------------------------------------------------------------------------
// Publish Outputs
// ---------------------------------------------------------------------------
// Monotonic timestamps are now updated inside find_closest_camera() /
// find_closest_lidar() themselves (atomically, under their own mutex), so
// this function only needs to publish and bump counters.

void SyncNode::publish_synced_output(
  const ImageMsg::ConstSharedPtr & camera_msg, bool camera_found, bool camera_stale,
  const PointCloudMsg::ConstSharedPtr & lidar_msg, bool lidar_found, bool lidar_stale,
  const NavSatFixMsg::ConstSharedPtr & gps_msg, bool gps_found, bool gps_interpolated,
  const ImuMsg::ConstSharedPtr & imu_msg, bool imu_found,
  const HeadingMsg::ConstSharedPtr & heading_msg, bool heading_found,
  const ImageMsg::ConstSharedPtr & depth_msg, bool depth_found,
  const OdometryMsg::ConstSharedPtr & odom_msg, bool odom_found,
  const std::string & trigger_source,
  double fusion_latency_ms)
{
  if (camera_found && camera_msg) {
    synced_camera_pub_->publish(*camera_msg);
    camera_published_++;
  }

  if (lidar_found && lidar_msg) {
    synced_lidar_pub_->publish(*lidar_msg);
    lidar_published_++;
  }

  if (gps_found && gps_msg) {
    synced_gps_pub_->publish(*gps_msg);
    gps_published_++;
  }

  if (imu_found && imu_msg)         synced_imu_pub_->publish(*imu_msg);
  if (heading_found && heading_msg) synced_heading_pub_->publish(*heading_msg);
  if (depth_found && depth_msg)     synced_depth_pub_->publish(*depth_msg);
  if (odom_found && odom_msg)   synced_odom_pub_->publish(*odom_msg);

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "sync_node/fusion_status";
  status.level = (camera_found && gps_found && lidar_found && !camera_stale && !lidar_stale)
    ? diagnostic_msgs::msg::DiagnosticStatus::OK
    : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = "Triggered by [" + trigger_source + "]";

  auto add_kv = [&](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue kv; kv.key = key; kv.value = value;
    status.values.push_back(kv);
  };
  add_kv("trigger_source", trigger_source);
  add_kv("camera_found", camera_found ? "true" : "false");
  add_kv("camera_stale", camera_stale ? "true" : "false");
  add_kv("lidar_found", lidar_found ? "true" : "false");
  add_kv("lidar_stale", lidar_stale ? "true" : "false");
  add_kv("gps_found", gps_found ? "true" : "false");
  add_kv("gps_interpolated", gps_interpolated ? "true" : "false");
  add_kv("fusion_latency_ms", std::to_string(fusion_latency_ms));

  status_pub_->publish(status);
}

void SyncNode::log_stats()
{
  uint64_t cr = camera_received_.load(), cp = camera_published_.load();
  uint64_t lr = lidar_received_.load(),  lp = lidar_published_.load();
  uint64_t gr = gps_received_.load(),    gp = gps_published_.load();

  RCLCPP_INFO(this->get_logger(),
    "[NO-JITTER STATS] Camera R/P: %lu/%lu | LiDAR R/P: %lu/%lu | GPS R/P: %lu/%lu",
    cr, cp, lr, lp, gr, gp);

  diagnostic_msgs::msg::DiagnosticStatus stats_msg;
  stats_msg.name = "sync_node/loss_stats";
  stats_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;

  auto add_kv = [&](const std::string & key, uint64_t value) {
    diagnostic_msgs::msg::KeyValue kv; kv.key = key; kv.value = std::to_string(value);
    stats_msg.values.push_back(kv);
  };
  add_kv("camera_recv", cr); add_kv("camera_pub", cp);
  add_kv("lidar_recv", lr);  add_kv("lidar_pub", lp);
  add_kv("gps_recv", gr);    add_kv("gps_pub", gp);

  loss_stats_pub_->publish(stats_msg);
}

}  // namespace sync_node_pkg
