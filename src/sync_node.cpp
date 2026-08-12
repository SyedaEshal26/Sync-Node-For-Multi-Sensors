#include "sync_node_pkg/sync_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <filesystem>

namespace sync_node_pkg
{

SyncNode::SyncNode()
: Node("sync_node")
{
  // ============================================================
  // Parameters
  // ============================================================

  declare_parameter<int>("max_buffer_size", 300);
  declare_parameter<int>("gps_buffer_size", 100);

  declare_parameter<double>("max_camera_delta_sec", 0.10);
  declare_parameter<double>("max_lidar_delta_sec", 1.50);
  declare_parameter<double>("max_imu_delta_sec", 0.05);
  declare_parameter<double>("max_depth_delta_sec", 0.20);
  declare_parameter<double>("max_odom_delta_sec", 0.10);
  declare_parameter<double>("max_gps_delta_sec", 5.0);

  declare_parameter<std::string>(
    "output_directory",
    "/home/eshal/ros2_ws/src/sync_node_pkg/sync_data");

  max_buffer_size_ =
    static_cast<size_t>(get_parameter("max_buffer_size").as_int());

  gps_buffer_size_ =
    static_cast<size_t>(get_parameter("gps_buffer_size").as_int());

  max_camera_delta_sec_ =
    get_parameter("max_camera_delta_sec").as_double();

  max_lidar_delta_sec_ =
    get_parameter("max_lidar_delta_sec").as_double();

  max_imu_delta_sec_ =
    get_parameter("max_imu_delta_sec").as_double();

  max_depth_delta_sec_ =
    get_parameter("max_depth_delta_sec").as_double();

  max_odom_delta_sec_ =
    get_parameter("max_odom_delta_sec").as_double();

  max_gps_delta_sec_ =
    get_parameter("max_gps_delta_sec").as_double();

  output_directory_ =
    get_parameter("output_directory").as_string();

  // ============================================================
  // Create output directory
  // ============================================================

  try {
    std::filesystem::create_directories(output_directory_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(),
      "Could not create output directory: %s",
      e.what());
  }

  // ============================================================
  // CSV
  // ============================================================

  const std::string csv_path =
    output_directory_ + "/sync_statistics.csv";

  csv_file_.open(csv_path, std::ios::out | std::ios::trunc);

  if (!csv_file_.is_open()) {
    RCLCPP_ERROR(
      get_logger(),
      "Could not open CSV: %s",
      csv_path.c_str());
  } else {

    csv_file_
      << "wall_time,"
      << "fusion_id,"
      << "trigger_source,"
      << "trigger_stamp,"
      << "camera_found,"
      << "lidar_found,"
      << "gps_found,"
      << "gps_interpolated,"
      << "imu_found,"
      << "depth_found,"
      << "odom_found,"
      << "camera_delta_ms,"
      << "lidar_delta_ms,"
      << "imu_delta_ms,"
      << "depth_delta_ms,"
      << "odom_delta_ms,"
      << "gps_delta_ms,"
      << "fusion_latency_ms\n";

    csv_file_.flush();
  }

  // ============================================================
  // Callback groups
  // ============================================================

  camera_cb_group_ =
    create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  lidar_cb_group_ =
    create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  gps_cb_group_ =
    create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  imu_cb_group_ =
    create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  depth_cb_group_ =
    create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  odom_cb_group_ =
    create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  heading_cb_group_ =
    create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions camera_opts;
  camera_opts.callback_group = camera_cb_group_;

  rclcpp::SubscriptionOptions lidar_opts;
  lidar_opts.callback_group = lidar_cb_group_;

  rclcpp::SubscriptionOptions gps_opts;
  gps_opts.callback_group = gps_cb_group_;

  rclcpp::SubscriptionOptions imu_opts;
  imu_opts.callback_group = imu_cb_group_;

  rclcpp::SubscriptionOptions depth_opts;
  depth_opts.callback_group = depth_cb_group_;

  rclcpp::SubscriptionOptions odom_opts;
  odom_opts.callback_group = odom_cb_group_;

  rclcpp::SubscriptionOptions heading_opts;
  heading_opts.callback_group = heading_cb_group_;

  // ============================================================
  // Subscriptions
  // ============================================================

  camera_sub_ =
    create_subscription<ImageMsg>(
      "/zed/zed_node/rgb/image_rect_color",
      rclcpp::SensorDataQoS(),
      std::bind(
        &SyncNode::camera_callback,
        this,
        std::placeholders::_1),
      camera_opts);

  lidar_sub_ =
    create_subscription<PointCloudMsg>(
      "/rslidar_points",
      rclcpp::SensorDataQoS(),
      std::bind(
        &SyncNode::lidar_callback,
        this,
        std::placeholders::_1),
      lidar_opts);

  gps_sub_ =
    create_subscription<NavSatFixMsg>(
      "/fix",
      rclcpp::SensorDataQoS(),
      std::bind(
        &SyncNode::gps_callback,
        this,
        std::placeholders::_1),
      gps_opts);

  imu_sub_ =
    create_subscription<ImuMsg>(
      "/zed/zed_node/imu/data",
      rclcpp::SensorDataQoS(),
      std::bind(
        &SyncNode::imu_callback,
        this,
        std::placeholders::_1),
      imu_opts);

  depth_sub_ =
    create_subscription<ImageMsg>(
      "/zed/zed_node/depth/depth_registered",
      rclcpp::SensorDataQoS(),
      std::bind(
        &SyncNode::depth_callback,
        this,
        std::placeholders::_1),
      depth_opts);

  odom_sub_ =
    create_subscription<OdometryMsg>(
      "/zed/zed_node/odom",
      rclcpp::SensorDataQoS(),
      std::bind(
        &SyncNode::odom_callback,
        this,
        std::placeholders::_1),
      odom_opts);

  heading_sub_ =
    create_subscription<HeadingMsg>(
      "/heading",
      rclcpp::SensorDataQoS(),
      std::bind(
        &SyncNode::heading_callback,
        this,
        std::placeholders::_1),
      heading_opts);

  // ============================================================
  // Publishers
  // ============================================================

  synced_camera_pub_ =
    create_publisher<ImageMsg>("/synced/camera", 10);

  synced_lidar_pub_ =
    create_publisher<PointCloudMsg>("/synced/lidar", 10);

  synced_gps_pub_ =
    create_publisher<NavSatFixMsg>("/synced/gps", 10);

  synced_imu_pub_ =
    create_publisher<ImuMsg>("/synced/zed/imu", 10);

  synced_depth_pub_ =
    create_publisher<ImageMsg>(
      "/synced/zed/depth_registered", 10);

  synced_odom_pub_ =
    create_publisher<OdometryMsg>(
      "/synced/zed/odom", 10);

  synced_heading_pub_ =
    create_publisher<HeadingMsg>(
      "/synced/heading", 10);

  status_pub_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
      "/synced/status", 10);

  loss_stats_pub_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
      "/synced/loss_stats", 10);

  // ============================================================
  // Statistics timer
  // ============================================================

  stats_timer_ =
    create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&SyncNode::log_stats, this));

  // ============================================================
  // Startup
  // ============================================================

  RCLCPP_INFO(
    get_logger(),
    "==============================================");

  RCLCPP_INFO(
    get_logger(),
    "SyncNode started");

  RCLCPP_INFO(
    get_logger(),
    "Camera + LiDAR trigger fusion");

  RCLCPP_INFO(
    get_logger(),
    "Buffered nearest-timestamp synchronization");

  RCLCPP_INFO(
    get_logger(),
    "GPS interpolation enabled");

  RCLCPP_INFO(
    get_logger(),
    "CSV output: %s",
    csv_path.c_str());

  RCLCPP_INFO(
    get_logger(),
    "==============================================");
}

// ================================================================
// Destructor
// ================================================================

SyncNode::~SyncNode()
{
  std::lock_guard<std::mutex> lock(csv_mutex_);

  if (csv_file_.is_open()) {
    csv_file_.flush();
    csv_file_.close();
  }
}

// ================================================================
// Timestamp difference
// ================================================================

double SyncNode::timestamp_difference(
  const rclcpp::Time & a,
  const rclcpp::Time & b) const
{
  return std::abs((a - b).seconds());
}

// ================================================================
// Camera callback
// ================================================================

void SyncNode::camera_callback(
  const ImageMsg::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);

    camera_buffer_.push_back(msg);

    if (camera_buffer_.size() > max_buffer_size_) {
      camera_buffer_.pop_front();
      camera_buffer_drops_++;
    }
  }

  camera_received_++;

  trigger_fusion(
    rclcpp::Time(msg->header.stamp),
    "camera");
}

// ================================================================
// LiDAR callback
// ================================================================

void SyncNode::lidar_callback(
  const PointCloudMsg::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(lidar_mutex_);

    lidar_buffer_.push_back(msg);

    if (lidar_buffer_.size() > max_buffer_size_) {
      lidar_buffer_.pop_front();
      lidar_buffer_drops_++;
    }
  }

  lidar_received_++;

  trigger_fusion(
    rclcpp::Time(msg->header.stamp),
    "lidar");
}

// ================================================================
// GPS callback
// ================================================================

void SyncNode::gps_callback(
  const NavSatFixMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(gps_mutex_);

  gps_buffer_.push_back(msg);

  if (gps_buffer_.size() > gps_buffer_size_) {
    gps_buffer_.pop_front();
    gps_buffer_drops_++;
  }

  gps_received_++;
}

// ================================================================
// IMU callback
// ================================================================

void SyncNode::imu_callback(
  const ImuMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);

  imu_buffer_.push_back(msg);

  if (imu_buffer_.size() > max_buffer_size_) {
    imu_buffer_.pop_front();
    imu_buffer_drops_++;
  }

  imu_received_++;
}

// ================================================================
// Depth callback
// ================================================================

void SyncNode::depth_callback(
  const ImageMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(depth_mutex_);

  depth_buffer_.push_back(msg);

  if (depth_buffer_.size() > max_buffer_size_) {
    depth_buffer_.pop_front();
    depth_buffer_drops_++;
  }

  depth_received_++;
}

// ================================================================
// Odom callback
// ================================================================

void SyncNode::odom_callback(
  const OdometryMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);

  odom_buffer_.push_back(msg);

  if (odom_buffer_.size() > max_buffer_size_) {
    odom_buffer_.pop_front();
    odom_buffer_drops_++;
  }

  odom_received_++;
}

// ================================================================
// Heading callback
// ================================================================

void SyncNode::heading_callback(
  const HeadingMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(heading_mutex_);

  latest_heading_ = msg;
  heading_received_ = true;
  heading_received_count_++;
}

// ================================================================
// Camera matching
// ================================================================

bool SyncNode::find_closest_camera(
  const rclcpp::Time & target,
  ImageMsg::ConstSharedPtr & result)
{
  std::lock_guard<std::mutex> lock(camera_mutex_);

  if (camera_buffer_.empty()) {
    return false;
  }

  auto closest =
    std::min_element(
      camera_buffer_.begin(),
      camera_buffer_.end(),
      [&](const auto & a, const auto & b)
      {
        return timestamp_difference(
                 rclcpp::Time(a->header.stamp), target)
             <
               timestamp_difference(
                 rclcpp::Time(b->header.stamp), target);
      });

  double delta =
    timestamp_difference(
      rclcpp::Time((*closest)->header.stamp),
      target);

  if (delta > max_camera_delta_sec_) {
    return false;
  }

  result = *closest;

  camera_buffer_.erase(closest);

  camera_matched_++;

  return true;
}

// ================================================================
// LiDAR matching
// ================================================================

bool SyncNode::find_closest_lidar(
  const rclcpp::Time & target,
  PointCloudMsg::ConstSharedPtr & result)
{
  std::lock_guard<std::mutex> lock(lidar_mutex_);

  if (lidar_buffer_.empty()) {
    return false;
  }

  auto closest =
    std::min_element(
      lidar_buffer_.begin(),
      lidar_buffer_.end(),
      [&](const auto & a, const auto & b)
      {
        return timestamp_difference(
                 rclcpp::Time(a->header.stamp), target)
             <
               timestamp_difference(
                 rclcpp::Time(b->header.stamp), target);
      });

  double delta =
    timestamp_difference(
      rclcpp::Time((*closest)->header.stamp),
      target);

  if (delta > max_lidar_delta_sec_) {
    return false;
  }

  result = *closest;

  lidar_buffer_.erase(closest);

  lidar_matched_++;

  return true;
}

// ================================================================
// IMU matching
// ================================================================

bool SyncNode::find_closest_imu(
  const rclcpp::Time & target,
  ImuMsg::ConstSharedPtr & result)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);

  if (imu_buffer_.empty()) {
    return false;
  }

  auto closest =
    std::min_element(
      imu_buffer_.begin(),
      imu_buffer_.end(),
      [&](const auto & a, const auto & b)
      {
        return timestamp_difference(
                 rclcpp::Time(a->header.stamp), target)
             <
               timestamp_difference(
                 rclcpp::Time(b->header.stamp), target);
      });

  double delta =
    timestamp_difference(
      rclcpp::Time((*closest)->header.stamp),
      target);

  if (delta > max_imu_delta_sec_) {
    return false;
  }

  result = *closest;

  imu_buffer_.erase(closest);

  imu_matched_++;

  return true;
}

// ================================================================
// Depth matching
// ================================================================

bool SyncNode::find_closest_depth(
  const rclcpp::Time & target,
  ImageMsg::ConstSharedPtr & result)
{
  std::lock_guard<std::mutex> lock(depth_mutex_);

  if (depth_buffer_.empty()) {
    return false;
  }

  auto closest =
    std::min_element(
      depth_buffer_.begin(),
      depth_buffer_.end(),
      [&](const auto & a, const auto & b)
      {
        return timestamp_difference(
                 rclcpp::Time(a->header.stamp), target)
             <
               timestamp_difference(
                 rclcpp::Time(b->header.stamp), target);
      });

  double delta =
    timestamp_difference(
      rclcpp::Time((*closest)->header.stamp),
      target);

  if (delta > max_depth_delta_sec_) {
    return false;
  }

  result = *closest;

  depth_buffer_.erase(closest);

  depth_matched_++;

  return true;
}

// ================================================================
// Odom matching
// ================================================================

bool SyncNode::find_closest_odom(
  const rclcpp::Time & target,
  OdometryMsg::ConstSharedPtr & result)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);

  if (odom_buffer_.empty()) {
    return false;
  }

  auto closest =
    std::min_element(
      odom_buffer_.begin(),
      odom_buffer_.end(),
      [&](const auto & a, const auto & b)
      {
        return timestamp_difference(
                 rclcpp::Time(a->header.stamp), target)
             <
               timestamp_difference(
                 rclcpp::Time(b->header.stamp), target);
      });

  double delta =
    timestamp_difference(
      rclcpp::Time((*closest)->header.stamp),
      target);

  if (delta > max_odom_delta_sec_) {
    return false;
  }

  result = *closest;

  odom_buffer_.erase(closest);

  odom_matched_++;

  return true;
}

// ================================================================
// GPS
// ================================================================

bool SyncNode::get_gps_estimate(
  const rclcpp::Time & target,
  NavSatFixMsg::ConstSharedPtr & result,
  bool & interpolated)
{
  std::lock_guard<std::mutex> lock(gps_mutex_);

  interpolated = false;

  if (gps_buffer_.empty()) {
    return false;
  }

  // --------------------------------------------------------------
  // One GPS sample
  // --------------------------------------------------------------

  if (gps_buffer_.size() == 1) {

    const auto & msg = gps_buffer_.front();

    double delta =
      timestamp_difference(
        rclcpp::Time(msg->header.stamp),
        target);

    if (delta > max_gps_delta_sec_) {
      return false;
    }

    result = msg;

    gps_matched_++;
    gps_direct_count_++;

    return true;
  }

  // --------------------------------------------------------------
  // Find closest GPS sample
  // --------------------------------------------------------------

  auto closest =
    std::min_element(
      gps_buffer_.begin(),
      gps_buffer_.end(),
      [&](const auto & a, const auto & b)
      {
        return timestamp_difference(
                 rclcpp::Time(a->header.stamp), target)
             <
               timestamp_difference(
                 rclcpp::Time(b->header.stamp), target);
      });

  double closest_delta =
    timestamp_difference(
      rclcpp::Time((*closest)->header.stamp),
      target);

  // --------------------------------------------------------------
  // Interpolation requires bracketing samples
  // --------------------------------------------------------------

  for (size_t i = 0; i + 1 < gps_buffer_.size(); ++i) {

    auto older = gps_buffer_[i];
    auto newer = gps_buffer_[i + 1];

    rclcpp::Time t_old(older->header.stamp);
    rclcpp::Time t_new(newer->header.stamp);

    if (t_old <= target && target <= t_new) {

      double dt =
        (t_new - t_old).seconds();

      if (dt <= 0.0) {
        break;
      }

      double ratio =
        (target - t_old).seconds() / dt;

      if (ratio < 0.0 || ratio > 1.0) {
        break;
      }

      auto gps =
        std::make_shared<NavSatFixMsg>(*newer);

      gps->latitude =
        older->latitude +
        ratio * (newer->latitude - older->latitude);

      gps->longitude =
        older->longitude +
        ratio * (newer->longitude - older->longitude);

      gps->altitude =
        older->altitude +
        ratio * (newer->altitude - older->altitude);

      gps->header.stamp = target;

      result = gps;

      interpolated = true;

      gps_matched_++;
      gps_interpolated_count_++;

      return true;
    }
  }

  // --------------------------------------------------------------
  // Otherwise use closest direct GPS
  // --------------------------------------------------------------

  if (closest_delta > max_gps_delta_sec_) {
    return false;
  }

  result = *closest;

  gps_matched_++;
  gps_direct_count_++;

  return true;
}

// ================================================================
// Fusion
// ================================================================

void SyncNode::trigger_fusion(
  const rclcpp::Time & timestamp,
  const std::string & trigger_source)
{
  std::lock_guard<std::mutex> fusion_lock(fusion_mutex_);

  // Prevent duplicate trigger at essentially same timestamp
  if (has_last_trigger_) {

    double dt =
      std::abs(
        (timestamp - last_trigger_time_).seconds());

    if (dt < 0.001) {
      return;
    }
  }

  last_trigger_time_ = timestamp;
  has_last_trigger_ = true;

  auto start =
    std::chrono::steady_clock::now();

  // --------------------------------------------------------------
  // Match sensors
  // --------------------------------------------------------------

  ImageMsg::ConstSharedPtr camera;
  PointCloudMsg::ConstSharedPtr lidar;
  NavSatFixMsg::ConstSharedPtr gps;
  ImuMsg::ConstSharedPtr imu;
  ImageMsg::ConstSharedPtr depth;
  OdometryMsg::ConstSharedPtr odom;
  HeadingMsg::ConstSharedPtr heading;

  bool camera_found =
    find_closest_camera(timestamp, camera);

  bool lidar_found =
    find_closest_lidar(timestamp, lidar);

  bool gps_interpolated = false;

  bool gps_found =
    get_gps_estimate(
      timestamp,
      gps,
      gps_interpolated);

  bool imu_found =
    find_closest_imu(timestamp, imu);

  bool depth_found =
    find_closest_depth(timestamp, depth);

  bool odom_found =
    find_closest_odom(timestamp, odom);

  bool heading_found = false;

  {
    std::lock_guard<std::mutex> lock(heading_mutex_);

    if (heading_received_ && latest_heading_) {
      heading = latest_heading_;
      heading_found = true;
    }
  }

  // --------------------------------------------------------------
  // Latency
  // --------------------------------------------------------------

  auto end =
    std::chrono::steady_clock::now();

  double latency_ms =
    std::chrono::duration<double, std::milli>(
      end - start).count();

  total_fusions_++;

  {
    std::lock_guard<std::mutex> lock(latency_mutex_);

    latency_sum_ms_ += latency_ms;

    latency_min_ms_ =
      std::min(latency_min_ms_, latency_ms);

    latency_max_ms_ =
      std::max(latency_max_ms_, latency_ms);

    latency_count_++;
  }

  // --------------------------------------------------------------
  // Publish
  // --------------------------------------------------------------

  if (camera_found && camera) {
    synced_camera_pub_->publish(*camera);
  }

  if (lidar_found && lidar) {
    synced_lidar_pub_->publish(*lidar);
  }

  if (gps_found && gps) {
    synced_gps_pub_->publish(*gps);
  }

  if (imu_found && imu) {
    synced_imu_pub_->publish(*imu);
  }

  if (depth_found && depth) {
    synced_depth_pub_->publish(*depth);
  }

  if (odom_found && odom) {
    synced_odom_pub_->publish(*odom);
  }

  if (heading_found && heading) {
    synced_heading_pub_->publish(*heading);
  }

  // --------------------------------------------------------------
  // Timestamp errors
  // --------------------------------------------------------------

  double camera_delta_ms = -1.0;
  double lidar_delta_ms = -1.0;
  double imu_delta_ms = -1.0;
  double depth_delta_ms = -1.0;
  double odom_delta_ms = -1.0;
  double gps_delta_ms = -1.0;

  if (camera_found) {
    camera_delta_ms =
      timestamp_difference(
        rclcpp::Time(camera->header.stamp),
        timestamp) * 1000.0;
  }

  if (lidar_found) {
    lidar_delta_ms =
      timestamp_difference(
        rclcpp::Time(lidar->header.stamp),
        timestamp) * 1000.0;
  }

  if (imu_found) {
    imu_delta_ms =
      timestamp_difference(
        rclcpp::Time(imu->header.stamp),
        timestamp) * 1000.0;
  }

  if (depth_found) {
    depth_delta_ms =
      timestamp_difference(
        rclcpp::Time(depth->header.stamp),
        timestamp) * 1000.0;
  }

  if (odom_found) {
    odom_delta_ms =
      timestamp_difference(
        rclcpp::Time(odom->header.stamp),
        timestamp) * 1000.0;
  }

  if (gps_found) {
    gps_delta_ms =
      timestamp_difference(
        rclcpp::Time(gps->header.stamp),
        timestamp) * 1000.0;
  }

  // --------------------------------------------------------------
  // CSV
  // --------------------------------------------------------------

  {
    std::lock_guard<std::mutex> lock(csv_mutex_);

    if (csv_file_.is_open()) {

      auto now =
        std::chrono::duration_cast<
          std::chrono::milliseconds>(
            std::chrono::system_clock::now()
              .time_since_epoch())
          .count();

      csv_file_
        << now << ","
        << total_fusions_.load() << ","
        << trigger_source << ","
        << std::fixed
        << std::setprecision(9)
        << timestamp.seconds() << ","

        << camera_found << ","
        << lidar_found << ","
        << gps_found << ","
        << gps_interpolated << ","
        << imu_found << ","
        << depth_found << ","
        << odom_found << ","

        << std::setprecision(4)
        << camera_delta_ms << ","
        << lidar_delta_ms << ","
        << imu_delta_ms << ","
        << depth_delta_ms << ","
        << odom_delta_ms << ","
        << gps_delta_ms << ","
        << latency_ms
        << "\n";

      csv_file_.flush();
    }
  }

  // --------------------------------------------------------------
  // Diagnostic status
  // --------------------------------------------------------------

  diagnostic_msgs::msg::DiagnosticStatus status;

  status.name =
    "sync_node/fusion_status";

  bool core_ok =
    camera_found &&
    lidar_found &&
    imu_found &&
    depth_found &&
    odom_found;

  status.level =
    core_ok
      ? diagnostic_msgs::msg::DiagnosticStatus::OK
      : diagnostic_msgs::msg::DiagnosticStatus::WARN;

  status.message =
    "Fusion triggered by " + trigger_source;

  auto add_status =
    [&](const std::string & key,
        const std::string & value)
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      status.values.push_back(kv);
    };

  add_status(
    "trigger_source",
    trigger_source);

  add_status(
    "camera_found",
    camera_found ? "true" : "false");

  add_status(
    "lidar_found",
    lidar_found ? "true" : "false");

  add_status(
    "gps_found",
    gps_found ? "true" : "false");

  add_status(
    "gps_interpolated",
    gps_interpolated ? "true" : "false");

  add_status(
    "imu_found",
    imu_found ? "true" : "false");

  add_status(
    "depth_found",
    depth_found ? "true" : "false");

  add_status(
    "odom_found",
    odom_found ? "true" : "false");

  add_status(
    "fusion_latency_ms",
    std::to_string(latency_ms));

  status_pub_->publish(status);
}

// ================================================================
// Statistics
// ================================================================

void SyncNode::log_stats()
{
  uint64_t cr = camera_received_.load();
  uint64_t lr = lidar_received_.load();
  uint64_t gr = gps_received_.load();
  uint64_t ir = imu_received_.load();
  uint64_t dr = depth_received_.load();
  uint64_t orr = odom_received_.load();

  uint64_t cm = camera_matched_.load();
  uint64_t lm = lidar_matched_.load();
  uint64_t gm = gps_matched_.load();
  uint64_t im = imu_matched_.load();
  uint64_t dm = depth_matched_.load();
  uint64_t om = odom_matched_.load();

  uint64_t cb = camera_buffer_drops_.load();
  uint64_t lb = lidar_buffer_drops_.load();
  uint64_t gb = gps_buffer_drops_.load();
  uint64_t ib = imu_buffer_drops_.load();
  uint64_t db = depth_buffer_drops_.load();
  uint64_t ob = odom_buffer_drops_.load();

  double avg_latency = 0.0;
  double min_latency = 0.0;
  double max_latency = 0.0;

  {
    std::lock_guard<std::mutex> lock(latency_mutex_);

    if (latency_count_ > 0) {

      avg_latency =
        latency_sum_ms_ /
        static_cast<double>(latency_count_);

      min_latency = latency_min_ms_;
      max_latency = latency_max_ms_;
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "==================================================");

  RCLCPP_INFO(
    get_logger(),
    "SYNC STATISTICS");

  RCLCPP_INFO(
    get_logger(),
    "Fusions: %lu",
    total_fusions_.load());

  RCLCPP_INFO(
    get_logger(),
    "Camera : recv=%lu matched=%lu buffer_drops=%lu",
    cr, cm, cb);

  RCLCPP_INFO(
    get_logger(),
    "LiDAR  : recv=%lu matched=%lu buffer_drops=%lu",
    lr, lm, lb);

  RCLCPP_INFO(
    get_logger(),
    "GPS    : recv=%lu matched=%lu buffer_drops=%lu",
    gr, gm, gb);

  RCLCPP_INFO(
    get_logger(),
    "IMU    : recv=%lu matched=%lu buffer_drops=%lu",
    ir, im, ib);

  RCLCPP_INFO(
    get_logger(),
    "Depth  : recv=%lu matched=%lu buffer_drops=%lu",
    dr, dm, db);

  RCLCPP_INFO(
    get_logger(),
    "Odom   : recv=%lu matched=%lu buffer_drops=%lu",
    orr, om, ob);

  RCLCPP_INFO(
    get_logger(),
    "GPS direct=%lu | interpolated=%lu",
    gps_direct_count_.load(),
    gps_interpolated_count_.load());

  RCLCPP_INFO(
    get_logger(),
    "Fusion latency: avg=%.3f ms | min=%.3f ms | max=%.3f ms",
    avg_latency,
    min_latency,
    max_latency);

  RCLCPP_INFO(
    get_logger(),
    "==================================================");

  // --------------------------------------------------------------
  // Diagnostic statistics
  // --------------------------------------------------------------

  diagnostic_msgs::msg::DiagnosticStatus msg;

  msg.name = "sync_node/statistics";

  msg.level =
    diagnostic_msgs::msg::DiagnosticStatus::OK;

  msg.message =
    "Synchronization statistics";

  auto add =
    [&](const std::string & key,
        uint64_t value)
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = std::to_string(value);
      msg.values.push_back(kv);
    };

  add("camera_received", cr);
  add("camera_matched", cm);
  add("camera_buffer_drops", cb);

  add("lidar_received", lr);
  add("lidar_matched", lm);
  add("lidar_buffer_drops", lb);

  add("gps_received", gr);
  add("gps_matched", gm);
  add("gps_buffer_drops", gb);

  add("imu_received", ir);
  add("imu_matched", im);
  add("imu_buffer_drops", ib);

  add("depth_received", dr);
  add("depth_matched", dm);
  add("depth_buffer_drops", db);

  add("odom_received", orr);
  add("odom_matched", om);
  add("odom_buffer_drops", ob);

  add(
    "gps_direct",
    gps_direct_count_.load());

  add(
    "gps_interpolated",
    gps_interpolated_count_.load());

  loss_stats_pub_->publish(msg);
}

}  // namespace sync_node_pkg
