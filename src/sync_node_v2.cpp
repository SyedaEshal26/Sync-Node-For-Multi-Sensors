#include "sync_node_pkg/sync_node_v2.hpp"

#include <algorithm>
#include <cmath>

namespace sync_node_pkg
{

SyncNodeV2::SyncNodeV2()
: Node("sync_node_v2")
{
  // Parameters for zero-loss synchronization
  this->declare_parameter<double>("camera_stale_threshold_sec", 0.2);
  this->declare_parameter<double>("lidar_stale_threshold_sec", 0.2);
  this->declare_parameter<double>("imu_stale_threshold_sec", 0.1);
  this->declare_parameter<double>("depth_stale_threshold_sec", 0.2);
  this->declare_parameter<double>("odom_stale_threshold_sec", 0.1);
  this->declare_parameter<int>("max_buffer_size", 200);
  this->declare_parameter<int>("gps_buffer_size", 50);

  camera_stale_threshold_sec_ = this->get_parameter("camera_stale_threshold_sec").as_double();
  lidar_stale_threshold_sec_  = this->get_parameter("lidar_stale_threshold_sec").as_double();
  imu_stale_threshold_sec_    = this->get_parameter("imu_stale_threshold_sec").as_double();
  depth_stale_threshold_sec_  = this->get_parameter("depth_stale_threshold_sec").as_double();
  odom_stale_threshold_sec_   = this->get_parameter("odom_stale_threshold_sec").as_double();
  max_buffer_size_ = static_cast<size_t>(this->get_parameter("max_buffer_size").as_int());
  gps_buffer_size_ = static_cast<size_t>(this->get_parameter("gps_buffer_size").as_int());

  // Create callback groups
  camera_cb_group_   = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  lidar_cb_group_    = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  gps_cb_group_      = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  imu_cb_group_      = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  depth_cb_group_    = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  odom_cb_group_     = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  heading_cb_group_  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions camera_opts; camera_opts.callback_group = camera_cb_group_;
  rclcpp::SubscriptionOptions lidar_opts;  lidar_opts.callback_group  = lidar_cb_group_;
  rclcpp::SubscriptionOptions gps_opts;    gps_opts.callback_group    = gps_cb_group_;
  rclcpp::SubscriptionOptions imu_opts;    imu_opts.callback_group    = imu_cb_group_;
  rclcpp::SubscriptionOptions depth_opts;  depth_opts.callback_group  = depth_cb_group_;
  rclcpp::SubscriptionOptions odom_opts;   odom_opts.callback_group   = odom_cb_group_;
  rclcpp::SubscriptionOptions heading_opts; heading_opts.callback_group = heading_cb_group_;

  // Subscriptions
  camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/zed/zed_node/rgb/image_rect_color", rclcpp::SensorDataQoS(),
    std::bind(&SyncNodeV2::camera_callback, this, std::placeholders::_1), camera_opts);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/rslidar_points", rclcpp::SensorDataQoS(),
    std::bind(&SyncNodeV2::lidar_callback, this, std::placeholders::_1), lidar_opts);

  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/fix", rclcpp::SensorDataQoS(),
    std::bind(&SyncNodeV2::gps_callback, this, std::placeholders::_1), gps_opts);

  heading_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
    "/heading", rclcpp::SensorDataQoS(),
    std::bind(&SyncNodeV2::heading_callback, this, std::placeholders::_1), heading_opts);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/zed/zed_node/imu/data", rclcpp::SensorDataQoS(),
    std::bind(&SyncNodeV2::imu_callback, this, std::placeholders::_1), imu_opts);

  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/zed/zed_node/depth/depth_registered", rclcpp::SensorDataQoS(),
    std::bind(&SyncNodeV2::depth_callback, this, std::placeholders::_1), depth_opts);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/zed/zed_node/odom", rclcpp::SensorDataQoS(),
    std::bind(&SyncNodeV2::odom_callback, this, std::placeholders::_1), odom_opts);

  // Publishers
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
    std::chrono::seconds(5), std::bind(&SyncNodeV2::log_stats, this));

  RCLCPP_INFO(this->get_logger(), 
    "🚀 SyncNodeV2: Zero-loss sync enabled | All sensors matched to camera/LiDAR triggers | GPS interpolated");
}

// ---------------------------------------------------------------------------
// Callbacks - Store all data, trigger from Camera & LiDAR only
// ---------------------------------------------------------------------------

void SyncNodeV2::camera_callback(const ImageMsg::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    camera_buffer_.push_back(msg);
    if (camera_buffer_.size() > max_buffer_size_) camera_buffer_.pop_front();
  }
  camera_received_++;
  trigger_fusion(msg->header.stamp, "camera");
}

void SyncNodeV2::lidar_callback(const PointCloudMsg::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    lidar_buffer_.push_back(msg);
    if (lidar_buffer_.size() > max_buffer_size_) lidar_buffer_.pop_front();
  }
  lidar_received_++;
  trigger_fusion(msg->header.stamp, "lidar");
}

void SyncNodeV2::gps_callback(const NavSatFixMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(gps_mutex_);
  gps_buffer_.push_back(msg);
  if (gps_buffer_.size() > gps_buffer_size_) gps_buffer_.pop_front();
  gps_received_++;
}

void SyncNodeV2::imu_callback(const ImuMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  imu_buffer_.push_back(msg);
  if (imu_buffer_.size() > max_buffer_size_) imu_buffer_.pop_front();
  imu_received_++;
}

void SyncNodeV2::depth_callback(const ImageMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(depth_mutex_);
  depth_buffer_.push_back(msg);
  if (depth_buffer_.size() > max_buffer_size_) depth_buffer_.pop_front();
  depth_received_++;
}

void SyncNodeV2::odom_callback(const OdometryMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  odom_buffer_.push_back(msg);
  if (odom_buffer_.size() > max_buffer_size_) odom_buffer_.pop_front();
  odom_received_++;
}

void SyncNodeV2::heading_callback(const HeadingMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(heading_mutex_);
  latest_heading_ = msg;
  heading_received_ = true;
}

// ---------------------------------------------------------------------------
// Closest Frame Matchers - ZERO LOSS (remove ONLY consumed frame)
// ---------------------------------------------------------------------------

bool SyncNodeV2::find_closest_camera(
  const rclcpp::Time & target, ImageMsg::ConstSharedPtr & result)
{
  std::lock_guard<std::mutex> lock(camera_mutex_);
  if (camera_buffer_.empty()) return false;

  // Prune very old frames
  while (camera_buffer_.size() > 1) {
    double age = (target - rclcpp::Time(camera_buffer_.front()->header.stamp)).seconds();
    if (age > camera_stale_threshold_sec_ * 5) {
      camera_buffer_.pop_front();
    } else {
      break;
    }
  }

  auto closest = std::min_element(
    camera_buffer_.begin(), camera_buffer_.end(),
    [&](const ImageMsg::ConstSharedPtr & a, const ImageMsg::ConstSharedPtr & b) {
      return std::abs((rclcpp::Time(a->header.stamp) - target).seconds()) <
             std::abs((rclcpp::Time(b->header.stamp) - target).seconds());
    });

  if (closest == camera_buffer_.end()) return false;

  rclcpp::Time match_stamp((*closest)->header.stamp);

  // Monotonic guard with 1ms tolerance
  if (has_published_camera_ && 
      (match_stamp - last_published_camera_stamp_).seconds() < -0.001) {
    camera_buffer_.erase(camera_buffer_.begin(), std::next(closest));
    return false;
  }

  result = *closest;

  if (!has_published_camera_ || match_stamp > last_published_camera_stamp_) {
    last_published_camera_stamp_ = match_stamp;
    has_published_camera_ = true;
  }

  // Remove ONLY consumed frame
  camera_buffer_.erase(closest);
  return true;
}

bool SyncNodeV2::find_closest_lidar(
  const rclcpp::Time & target, PointCloudMsg::ConstSharedPtr & result)
{
  std::lock_guard<std::mutex> lock(lidar_mutex_);
  if (lidar_buffer_.empty()) return false;

  while (lidar_buffer_.size() > 1) {
    double age = (target - rclcpp::Time(lidar_buffer_.front()->header.stamp)).seconds();
    if (age > lidar_stale_threshold_sec_ * 5) {
      lidar_buffer_.pop_front();
    } else {
      break;
    }
  }

  auto closest = std::min_element(
    lidar_buffer_.begin(), lidar_buffer_.end(),
    [&](const PointCloudMsg::ConstSharedPtr & a, const PointCloudMsg::ConstSharedPtr & b) {
      return std::abs((rclcpp::Time(a->header.stamp) - target).seconds()) <
             std::abs((rclcpp::Time(b->header.stamp) - target).seconds());
    });

  if (closest == lidar_buffer_.end()) return false;

  rclcpp::Time match_stamp((*closest)->header.stamp);

  if (has_published_lidar_ && 
      (match_stamp - last_published_lidar_stamp_).seconds() < -0.001) {
    lidar_buffer_.erase(lidar_buffer_.begin(), std::next(closest));
    return false;
  }

  result = *closest;

  if (!has_published_lidar_ || match_stamp > last_published_lidar_stamp_) {
    last_published_lidar_stamp_ = match_stamp;
    has_published_lidar_ = true;
  }

  // Remove ONLY consumed frame
  lidar_buffer_.erase(closest);
  return true;
}

// ============================================================================
// ZERO-LOSS Matchers - NO MONOTONIC GUARD, remove ONLY consumed frame
// ============================================================================

bool SyncNodeV2::find_closest_imu(
  const rclcpp::Time & target, ImuMsg::ConstSharedPtr & result)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  if (imu_buffer_.empty()) return false;

  // Keep ALL frames - only remove extremely old ones
  while (imu_buffer_.size() > 1) {
    double age = (target - rclcpp::Time(imu_buffer_.front()->header.stamp)).seconds();
    if (age > imu_stale_threshold_sec_ * 10) {
      imu_buffer_.pop_front();
    } else {
      break;
    }
  }

  auto closest = std::min_element(
    imu_buffer_.begin(), imu_buffer_.end(),
    [&](const ImuMsg::ConstSharedPtr & a, const ImuMsg::ConstSharedPtr & b) {
      return std::abs((rclcpp::Time(a->header.stamp) - target).seconds()) <
             std::abs((rclcpp::Time(b->header.stamp) - target).seconds());
    });

  if (closest == imu_buffer_.end()) return false;

  result = *closest;

  // Remove ONLY consumed frame - ZERO LOSS
  imu_buffer_.erase(closest);
  return true;
}

bool SyncNodeV2::find_closest_depth(
  const rclcpp::Time & target, ImageMsg::ConstSharedPtr & result)
{
  std::lock_guard<std::mutex> lock(depth_mutex_);
  if (depth_buffer_.empty()) return false;

  while (depth_buffer_.size() > 1) {
    double age = (target - rclcpp::Time(depth_buffer_.front()->header.stamp)).seconds();
    if (age > depth_stale_threshold_sec_ * 10) {
      depth_buffer_.pop_front();
    } else {
      break;
    }
  }

  auto closest = std::min_element(
    depth_buffer_.begin(), depth_buffer_.end(),
    [&](const ImageMsg::ConstSharedPtr & a, const ImageMsg::ConstSharedPtr & b) {
      return std::abs((rclcpp::Time(a->header.stamp) - target).seconds()) <
             std::abs((rclcpp::Time(b->header.stamp) - target).seconds());
    });

  if (closest == depth_buffer_.end()) return false;

  result = *closest;

  // Remove ONLY consumed frame - ZERO LOSS
  depth_buffer_.erase(closest);
  return true;
}

bool SyncNodeV2::find_closest_odom(
  const rclcpp::Time & target, OdometryMsg::ConstSharedPtr & result)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if (odom_buffer_.empty()) return false;

  while (odom_buffer_.size() > 1) {
    double age = (target - rclcpp::Time(odom_buffer_.front()->header.stamp)).seconds();
    if (age > odom_stale_threshold_sec_ * 10) {
      odom_buffer_.pop_front();
    } else {
      break;
    }
  }

  auto closest = std::min_element(
    odom_buffer_.begin(), odom_buffer_.end(),
    [&](const OdometryMsg::ConstSharedPtr & a, const OdometryMsg::ConstSharedPtr & b) {
      return std::abs((rclcpp::Time(a->header.stamp) - target).seconds()) <
             std::abs((rclcpp::Time(b->header.stamp) - target).seconds());
    });

  if (closest == odom_buffer_.end()) return false;

  result = *closest;

  // Remove ONLY consumed frame - ZERO LOSS
  odom_buffer_.erase(closest);
  return true;
}

// ---------------------------------------------------------------------------
// GPS Interpolation
// ---------------------------------------------------------------------------

bool SyncNodeV2::get_gps_estimate(
  const rclcpp::Time & target, NavSatFixMsg::ConstSharedPtr & result, bool & is_interpolated)
{
  std::lock_guard<std::mutex> lock(gps_mutex_);
  if (gps_buffer_.empty()) return false;

  while (gps_buffer_.size() > 2) {
    if ((target - rclcpp::Time(gps_buffer_.front()->header.stamp)).seconds() > 5.0) {
      gps_buffer_.pop_front();
    } else {
      break;
    }
  }

  if (gps_buffer_.size() < 2) {
    result = gps_buffer_.back();
    is_interpolated = false;
    return true;
  }

  const auto & older = gps_buffer_[gps_buffer_.size() - 2];
  const auto & newer = gps_buffer_.back();

  double t_old = rclcpp::Time(older->header.stamp).seconds();
  double t_new = rclcpp::Time(newer->header.stamp).seconds();
  double t_tgt = target.seconds();

  if (t_tgt >= t_new) {
    result = newer;
    is_interpolated = false;
    return true;
  }

  double ratio = (t_new > t_old) ? (t_tgt - t_old) / (t_new - t_old) : 0.0;
  ratio = std::clamp(ratio, 0.0, 1.0);

  auto interpolated = std::make_shared<NavSatFixMsg>(*newer);
  interpolated->latitude  = older->latitude  + ratio * (newer->latitude  - older->latitude);
  interpolated->longitude = older->longitude + ratio * (newer->longitude - older->longitude);
  interpolated->altitude  = older->altitude  + ratio * (newer->altitude  - older->altitude);
  
  interpolated->header.stamp = target;  // Set to trigger time
  
  result = interpolated;
  is_interpolated = (ratio > 0.0 && ratio < 1.0);
  return true;
}

// ---------------------------------------------------------------------------
// Fusion Execution
// ---------------------------------------------------------------------------

void SyncNodeV2::trigger_fusion(const rclcpp::Time & trigger_timestamp, const std::string & trigger_source)
{
  std::lock_guard<std::mutex> fusion_lock(fusion_mutex_);
  
  // 1ms cooldown to prevent double-triggering
  if ((trigger_timestamp - last_trigger_time_).seconds() < 0.001) {
    return;
  }
  last_trigger_time_ = trigger_timestamp;

  auto t_start = std::chrono::steady_clock::now();

  // Find closest frame for ALL sensors
  ImageMsg::ConstSharedPtr matched_camera;
  bool camera_found = find_closest_camera(trigger_timestamp, matched_camera);

  PointCloudMsg::ConstSharedPtr matched_lidar;
  bool lidar_found = find_closest_lidar(trigger_timestamp, matched_lidar);

  NavSatFixMsg::ConstSharedPtr gps_output;
  bool gps_interpolated = false;
  bool gps_found = get_gps_estimate(trigger_timestamp, gps_output, gps_interpolated);

  ImuMsg::ConstSharedPtr matched_imu;
  bool imu_found = find_closest_imu(trigger_timestamp, matched_imu);

  ImageMsg::ConstSharedPtr matched_depth;
  bool depth_found = find_closest_depth(trigger_timestamp, matched_depth);

  OdometryMsg::ConstSharedPtr matched_odom;
  bool odom_found = find_closest_odom(trigger_timestamp, matched_odom);

  HeadingMsg::ConstSharedPtr heading_copy;
  bool heading_found = false;
  {
    std::lock_guard<std::mutex> lock(heading_mutex_);
    if (heading_received_) {
      heading_copy = latest_heading_;
      heading_found = true;
    }
  }

  auto t_end = std::chrono::steady_clock::now();
  double latency_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  // Publish ALL synchronized data
  if (camera_found && matched_camera) {
    synced_camera_pub_->publish(*matched_camera);
    camera_published_++;
  }

  if (lidar_found && matched_lidar) {
    synced_lidar_pub_->publish(*matched_lidar);
    lidar_published_++;
  }

  if (gps_found && gps_output) {
    synced_gps_pub_->publish(*gps_output);
    gps_published_++;
  }

  if (imu_found && matched_imu) {
    synced_imu_pub_->publish(*matched_imu);
    imu_published_++;
  }

  if (depth_found && matched_depth) {
    synced_depth_pub_->publish(*matched_depth);
    depth_published_++;
  }

  if (odom_found && matched_odom) {
    synced_odom_pub_->publish(*matched_odom);
    odom_published_++;
  }

  if (heading_found && heading_copy) {
    synced_heading_pub_->publish(*heading_copy);
  }

  // Status
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "sync_node/fusion_status";
  status.level = (camera_found && lidar_found && gps_found && imu_found && depth_found && odom_found)
    ? diagnostic_msgs::msg::DiagnosticStatus::OK
    : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = "Triggered by [" + trigger_source + "]";

  auto add_kv = [&](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue kv; kv.key = key; kv.value = value;
    status.values.push_back(kv);
  };
  add_kv("trigger_source", trigger_source);
  add_kv("camera_found", camera_found ? "true" : "false");
  add_kv("lidar_found", lidar_found ? "true" : "false");
  add_kv("gps_found", gps_found ? "true" : "false");
  add_kv("gps_interpolated", gps_interpolated ? "true" : "false");
  add_kv("imu_found", imu_found ? "true" : "false");
  add_kv("depth_found", depth_found ? "true" : "false");
  add_kv("odom_found", odom_found ? "true" : "false");
  add_kv("fusion_latency_ms", std::to_string(latency_ms));

  status_pub_->publish(status);

  {
    std::lock_guard<std::mutex> lock(latency_stats_mutex_);
    latency_sum_ms_ += latency_ms;
    latency_count_++;
    if (latency_count_ >= kLatencyLogInterval) {
      RCLCPP_DEBUG(this->get_logger(), "Avg fusion latency: %.3f ms",
        latency_sum_ms_ / latency_count_);
      latency_sum_ms_ = 0.0;
      latency_count_ = 0;
    }
  }
}

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------

void SyncNodeV2::log_stats()
{
  uint64_t cr = camera_received_.load(), cp = camera_published_.load();
  uint64_t lr = lidar_received_.load(), lp = lidar_published_.load();
  uint64_t gr = gps_received_.load(), gp = gps_published_.load();
  uint64_t ir = imu_received_.load(), ip = imu_published_.load();
  uint64_t dr = depth_received_.load(), dp = depth_published_.load();
  uint64_t orr = odom_received_.load(), op = odom_published_.load();

  auto loss_pct = [](uint64_t recv, uint64_t pub) -> double {
    return recv > 0 ? (1.0 - static_cast<double>(pub) / recv) * 100.0 : 0.0;
  };

  RCLCPP_INFO(this->get_logger(),
    "📊 SYNC STATS: Camera %lu/%lu (%.1f%%) | LiDAR %lu/%lu (%.1f%%) | "
    "GPS %lu/%lu (%.1f%%) | IMU %lu/%lu (%.1f%%) | "
    "Depth %lu/%lu (%.1f%%) | Odom %lu/%lu (%.1f%%)",
    cr, cp, loss_pct(cr, cp),
    lr, lp, loss_pct(lr, lp),
    gr, gp, loss_pct(gr, gp),
    ir, ip, loss_pct(ir, ip),
    dr, dp, loss_pct(dr, dp),
    orr, op, loss_pct(orr, op));

  diagnostic_msgs::msg::DiagnosticStatus stats_msg;
  stats_msg.name = "sync_node/loss_stats";
  stats_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;

  auto add_kv = [&](const std::string & key, uint64_t value) {
    diagnostic_msgs::msg::KeyValue kv; kv.key = key; kv.value = std::to_string(value);
    stats_msg.values.push_back(kv);
  };
  add_kv("camera_recv", cr); add_kv("camera_pub", cp);
  add_kv("lidar_recv", lr); add_kv("lidar_pub", lp);
  add_kv("gps_recv", gr); add_kv("gps_pub", gp);
  add_kv("imu_recv", ir); add_kv("imu_pub", ip);
  add_kv("depth_recv", dr); add_kv("depth_pub", dp);
  add_kv("odom_recv", orr); add_kv("odom_pub", op);

  loss_stats_pub_->publish(stats_msg);
}

}  // namespace sync_node_pkg
