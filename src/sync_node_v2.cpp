#include "sync_node_pkg/sync_node_v2.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <filesystem>

namespace sync_node_pkg
{

SyncNodeV2::SyncNodeV2()
: Node("sync_node_v2")
{
  // ============================================================
  // Parameters - Generous windows for robust synchronization
  // ============================================================

  declare_parameter<int>("max_buffer_size", 500);
  declare_parameter<int>("gps_buffer_size", 10000);

  declare_parameter<double>("max_camera_delta_sec", 1.0);
  declare_parameter<double>("max_lidar_delta_sec", 1.0);
  declare_parameter<double>("max_imu_delta_sec", 1.0);
  declare_parameter<double>("max_depth_delta_sec", 0.5);
  declare_parameter<double>("max_odom_delta_sec", 1.0);
  declare_parameter<double>("max_gps_delta_sec", 5.0);
  declare_parameter<double>("max_rgb_raw_camera_info_delta_sec", 1.0);
  declare_parameter<double>("max_stereo_delta_sec", 1.0);

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

  max_rgb_raw_camera_info_delta_sec_ =
    get_parameter("max_rgb_raw_camera_info_delta_sec").as_double();

  max_stereo_delta_sec_ =
    get_parameter("max_stereo_delta_sec").as_double();

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
    output_directory_ + "/sync_statistics_v2.csv";

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
      << "imu_found,"
      << "depth_found,"
      << "odom_found,"
      << "heading_found,"
      << "rgb_raw_camera_info_found,"
      << "stereo_found,"
      << "camera_delta_ms,"
      << "lidar_delta_ms,"
      << "imu_delta_ms,"
      << "depth_delta_ms,"
      << "odom_delta_ms,"
      << "gps_delta_ms,"
      << "heading_delta_ms,"
      << "rgb_raw_camera_info_delta_ms,"
      << "stereo_delta_ms,"
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

  rgb_raw_camera_info_cb_group_ =
    create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  stereo_cb_group_ =
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

  rclcpp::SubscriptionOptions rgb_raw_camera_info_opts;
  rgb_raw_camera_info_opts.callback_group = rgb_raw_camera_info_cb_group_;

  rclcpp::SubscriptionOptions stereo_opts;
  stereo_opts.callback_group = stereo_cb_group_;

  // ============================================================
  // Subscriptions
  // ============================================================

  camera_sub_ =
    create_subscription<ImageMsg>(
      "/zed/zed_node/rgb/image_rect_color",
      rclcpp::SensorDataQoS(),
      std::bind(
        &SyncNodeV2::camera_callback,
        this,
        std::placeholders::_1),
      camera_opts);

  lidar_sub_ =
    create_subscription<PointCloudMsg>(
      "/rslidar_points",
      rclcpp::SensorDataQoS(),
      std::bind(
        &SyncNodeV2::lidar_callback,
        this,
        std::placeholders::_1),
      lidar_opts);

  gps_sub_ =
    create_subscription<NavSatFixMsg>(
      "/fix",
      rclcpp::SensorDataQoS(),
      std::bind(
        &SyncNodeV2::gps_callback,
        this,
        std::placeholders::_1),
      gps_opts);

  imu_sub_ =
    create_subscription<ImuMsg>(
      "/zed/zed_node/imu/data",
      rclcpp::SensorDataQoS(),
      std::bind(
        &SyncNodeV2::imu_callback,
        this,
        std::placeholders::_1),
      imu_opts);

  depth_sub_ =
    create_subscription<ImageMsg>(
      "/zed/zed_node/depth/depth_registered",
      rclcpp::SensorDataQoS(),
      std::bind(
        &SyncNodeV2::depth_callback,
        this,
        std::placeholders::_1),
      depth_opts);

  odom_sub_ =
    create_subscription<OdometryMsg>(
      "/zed/zed_node/odom",
      rclcpp::SensorDataQoS(),
      std::bind(
        &SyncNodeV2::odom_callback,
        this,
        std::placeholders::_1),
      odom_opts);

  // NOTE: topic name kept as "/heading" per original file; wire this to
  // your actual "/vel" heading topic name if different.
  heading_sub_ =
    create_subscription<HeadingMsg>(
      "/heading",
      rclcpp::SensorDataQoS(),
      std::bind(
        &SyncNodeV2::heading_callback,
        this,
        std::placeholders::_1),
      heading_opts);

  rgb_raw_camera_info_sub_ =
    create_subscription<CameraInfoMsg>(
      "/zed/zed_node/rgb_raw/camera_info",
      rclcpp::SensorDataQoS(),
      std::bind(
        &SyncNodeV2::rgb_raw_camera_info_callback,
        this,
        std::placeholders::_1),
      rgb_raw_camera_info_opts);

  stereo_sub_ =
    create_subscription<ImageMsg>(
      "/zed/zed_node/stereo/image_rect_color",
      rclcpp::SensorDataQoS(),
      std::bind(
        &SyncNodeV2::stereo_callback,
        this,
        std::placeholders::_1),
      stereo_opts);

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

  synced_rgb_raw_camera_info_pub_ =
    create_publisher<CameraInfoMsg>(
      "/synced/zed/rgb_raw_camera_info", 10);

  synced_stereo_pub_ =
    create_publisher<ImageMsg>(
      "/synced/zed/stereo_image_rect_color", 10);

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
      std::bind(&SyncNodeV2::log_stats, this));

  // ============================================================
  // Startup
  // ============================================================

  RCLCPP_INFO(
    get_logger(),
    "==============================================");

  RCLCPP_INFO(
    get_logger(),
    "SyncNodeV2 started - DEPTH TRIGGERED FUSION");

  RCLCPP_INFO(
    get_logger(),
    "GPS + Heading: hold mode (repeat last value until new sample arrives)");

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

SyncNodeV2::~SyncNodeV2()
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

double SyncNodeV2::timestamp_difference(
  const rclcpp::Time & a,
  const rclcpp::Time & b) const
{
  return std::abs((a - b).seconds());
}

// ================================================================
// Camera callback - STORE ONLY, NO TRIGGER
// ================================================================

void SyncNodeV2::camera_callback(const ImageMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(camera_mutex_);
  camera_buffer_.push_back(msg);

  if (camera_buffer_.size() > max_buffer_size_) {
    camera_buffer_.pop_front();
    camera_buffer_drops_++;
  }

  camera_received_++;
}

// ================================================================
// LiDAR callback - STORE ONLY (NO conversion needed with sim time)
// ================================================================

void SyncNodeV2::lidar_callback(const PointCloudMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(lidar_mutex_);
  lidar_buffer_.push_back(msg);

  if (lidar_buffer_.size() > max_buffer_size_) {
    lidar_buffer_.pop_front();
    lidar_buffer_drops_++;
  }

  lidar_received_++;
}

// ================================================================
// GPS callback - HOLD MODE
// Stores only the single latest fix. No buffer growth, no
// deque search - trigger_fusion() always uses whatever is
// currently held, same pattern as heading_callback() below.
// ================================================================

void SyncNodeV2::gps_callback(const NavSatFixMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(gps_mutex_);
  latest_gps_ = msg;
  gps_received_flag_ = true;
  gps_received_++;
}

// ================================================================
// IMU callback - STORE ONLY
// ================================================================

void SyncNodeV2::imu_callback(const ImuMsg::SharedPtr msg)
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
// DEPTH callback - THE ONLY TRIGGER SOURCE
// ================================================================

void SyncNodeV2::depth_callback(const ImageMsg::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    depth_buffer_.push_back(msg);

    if (depth_buffer_.size() > max_buffer_size_) {
      depth_buffer_.pop_front();
      depth_buffer_drops_++;
    }
  }

  depth_received_++;

  // DEPTH is the ONLY trigger
  trigger_fusion(rclcpp::Time(msg->header.stamp), "depth");
}

// ================================================================
// Odom callback - STORE ONLY
// ================================================================

void SyncNodeV2::odom_callback(const OdometryMsg::SharedPtr msg)
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
// Heading callback - HOLD MODE
// ================================================================

void SyncNodeV2::heading_callback(const HeadingMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(heading_mutex_);
  latest_heading_ = msg;
  heading_received_ = true;
  heading_received_count_++;
}

// ================================================================
// RGB raw camera_info callback - STORE ONLY
// ================================================================

void SyncNodeV2::rgb_raw_camera_info_callback(const CameraInfoMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(rgb_raw_camera_info_mutex_);
  rgb_raw_camera_info_buffer_.push_back(msg);

  if (rgb_raw_camera_info_buffer_.size() > max_buffer_size_) {
    rgb_raw_camera_info_buffer_.pop_front();
    rgb_raw_camera_info_buffer_drops_++;
  }

  rgb_raw_camera_info_received_++;
}

// ================================================================
// Stereo callback - STORE ONLY
// ================================================================

void SyncNodeV2::stereo_callback(const ImageMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(stereo_mutex_);
  stereo_buffer_.push_back(msg);

  if (stereo_buffer_.size() > max_buffer_size_) {
    stereo_buffer_.pop_front();
    stereo_buffer_drops_++;
  }

  stereo_received_++;
}

// ================================================================
// Camera matching
// ================================================================

bool SyncNodeV2::find_closest_camera(
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

bool SyncNodeV2::find_closest_lidar(
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

bool SyncNodeV2::find_closest_imu(
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
// Depth matching - DO NOT ERASE depth messages!
// ================================================================

bool SyncNodeV2::find_closest_depth(
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
  // DO NOT erase depth from buffer - it's the trigger source!
  depth_matched_++;
  return true;
}

// ================================================================
// Odom matching
// ================================================================

bool SyncNodeV2::find_closest_odom(
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
// RGB raw camera_info matching
// ================================================================

bool SyncNodeV2::find_closest_rgb_raw_camera_info(
  const rclcpp::Time & target,
  CameraInfoMsg::ConstSharedPtr & result)
{
  std::lock_guard<std::mutex> lock(rgb_raw_camera_info_mutex_);

  if (rgb_raw_camera_info_buffer_.empty()) {
    return false;
  }

  auto closest =
    std::min_element(
      rgb_raw_camera_info_buffer_.begin(),
      rgb_raw_camera_info_buffer_.end(),
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

  if (delta > max_rgb_raw_camera_info_delta_sec_) {
    return false;
  }

  result = *closest;
  rgb_raw_camera_info_buffer_.erase(closest);
  rgb_raw_camera_info_matched_++;
  return true;
}

// ================================================================
// Stereo matching
// ================================================================

bool SyncNodeV2::find_closest_stereo(
  const rclcpp::Time & target,
  ImageMsg::ConstSharedPtr & result)
{
  std::lock_guard<std::mutex> lock(stereo_mutex_);

  if (stereo_buffer_.empty()) {
    return false;
  }

  auto closest =
    std::min_element(
      stereo_buffer_.begin(),
      stereo_buffer_.end(),
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

  if (delta > max_stereo_delta_sec_) {
    return false;
  }

  result = *closest;
  stereo_buffer_.erase(closest);
  stereo_matched_++;
  return true;
}

// ================================================================
// Fusion - Depth ONLY trigger
// ================================================================

void SyncNodeV2::trigger_fusion(
  const rclcpp::Time & timestamp,
  const std::string & trigger_source)
{
  std::lock_guard<std::mutex> fusion_lock(fusion_mutex_);

  // Only allow "depth" as trigger
  if (trigger_source != "depth") {
    return;
  }

  if (has_last_trigger_) {
    double dt = std::abs((timestamp - last_trigger_time_).seconds());
    if (dt < 0.001) {
      return;
    }
  }

  last_trigger_time_ = timestamp;
  has_last_trigger_ = true;

  auto start = std::chrono::steady_clock::now();

  // Match ALL sensors to depth timestamp
  ImageMsg::ConstSharedPtr camera;
  PointCloudMsg::ConstSharedPtr lidar;
  NavSatFixMsg::ConstSharedPtr gps;
  ImuMsg::ConstSharedPtr imu;
  ImageMsg::ConstSharedPtr depth;
  OdometryMsg::ConstSharedPtr odom;
  HeadingMsg::ConstSharedPtr heading;
  CameraInfoMsg::ConstSharedPtr rgb_raw_camera_info;
  ImageMsg::ConstSharedPtr stereo;

  bool camera_found = find_closest_camera(timestamp, camera);
  bool lidar_found = find_closest_lidar(timestamp, lidar);
  bool imu_found = find_closest_imu(timestamp, imu);
  bool depth_found = find_closest_depth(timestamp, depth);
  bool odom_found = find_closest_odom(timestamp, odom);
  bool rgb_raw_camera_info_found = find_closest_rgb_raw_camera_info(timestamp, rgb_raw_camera_info);
  bool stereo_found = find_closest_stereo(timestamp, stereo);

  // GPS - HOLD MODE: always use the latest received fix, no
  // time-delta cutoff. Repeats previous value until a new
  // /fix message arrives.
  bool gps_found = false;
  {
    std::lock_guard<std::mutex> lock(gps_mutex_);
    if (gps_received_flag_ && latest_gps_) {
      gps = latest_gps_;
      gps_found = true;
      gps_matched_++;
    }
  }

  // Heading - HOLD MODE: same pattern as GPS.
  bool heading_found = false;
  {
    std::lock_guard<std::mutex> lock(heading_mutex_);
    if (heading_received_ && latest_heading_) {
      heading = latest_heading_;
      heading_found = true;
      heading_matched_++;
    }
  }

  auto end = std::chrono::steady_clock::now();
  double latency_ms = std::chrono::duration<double, std::milli>(end - start).count();

  total_fusions_++;

  {
    std::lock_guard<std::mutex> lock(latency_mutex_);
    latency_sum_ms_ += latency_ms;
    latency_min_ms_ = std::min(latency_min_ms_, latency_ms);
    latency_max_ms_ = std::max(latency_max_ms_, latency_ms);
    latency_count_++;
  }

  // Publish ALL synced topics
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

  if (rgb_raw_camera_info_found && rgb_raw_camera_info) {
    synced_rgb_raw_camera_info_pub_->publish(*rgb_raw_camera_info);
  }

  if (stereo_found && stereo) {
    synced_stereo_pub_->publish(*stereo);
  }

  // Timestamp errors (informational only for GPS/heading -
  // these are hold-mode and are published regardless of delta)
  double camera_delta_ms = -1.0;
  double lidar_delta_ms = -1.0;
  double imu_delta_ms = -1.0;
  double depth_delta_ms = -1.0;
  double odom_delta_ms = -1.0;
  double gps_delta_ms = -1.0;
  double heading_delta_ms = -1.0;
  double rgb_raw_camera_info_delta_ms = -1.0;
  double stereo_delta_ms = -1.0;

  if (camera_found) {
    camera_delta_ms = timestamp_difference(
      rclcpp::Time(camera->header.stamp), timestamp) * 1000.0;
  }

  if (lidar_found) {
    lidar_delta_ms = timestamp_difference(
      rclcpp::Time(lidar->header.stamp), timestamp) * 1000.0;
  }

  if (imu_found) {
    imu_delta_ms = timestamp_difference(
      rclcpp::Time(imu->header.stamp), timestamp) * 1000.0;
  }

  if (depth_found) {
    depth_delta_ms = timestamp_difference(
      rclcpp::Time(depth->header.stamp), timestamp) * 1000.0;
  }

  if (odom_found) {
    odom_delta_ms = timestamp_difference(
      rclcpp::Time(odom->header.stamp), timestamp) * 1000.0;
  }

  if (gps_found) {
    gps_delta_ms = timestamp_difference(
      rclcpp::Time(gps->header.stamp), timestamp) * 1000.0;
  }

  if (heading_found) {
    heading_delta_ms = timestamp_difference(
      rclcpp::Time(heading->header.stamp), timestamp) * 1000.0;
  }

  if (rgb_raw_camera_info_found) {
    rgb_raw_camera_info_delta_ms = timestamp_difference(
      rclcpp::Time(rgb_raw_camera_info->header.stamp), timestamp) * 1000.0;
  }

  if (stereo_found) {
    stereo_delta_ms = timestamp_difference(
      rclcpp::Time(stereo->header.stamp), timestamp) * 1000.0;
  }

  // CSV
  {
    std::lock_guard<std::mutex> lock(csv_mutex_);
    if (csv_file_.is_open()) {
      auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now().time_since_epoch()).count();

      csv_file_
        << now << ","
        << total_fusions_.load() << ","
        << trigger_source << ","
        << std::fixed << std::setprecision(9) << timestamp.seconds() << ","
        << camera_found << ","
        << lidar_found << ","
        << gps_found << ","
        << imu_found << ","
        << depth_found << ","
        << odom_found << ","
        << heading_found << ","
        << rgb_raw_camera_info_found << ","
        << stereo_found << ","
        << std::setprecision(4)
        << camera_delta_ms << ","
        << lidar_delta_ms << ","
        << imu_delta_ms << ","
        << depth_delta_ms << ","
        << odom_delta_ms << ","
        << gps_delta_ms << ","
        << heading_delta_ms << ","
        << rgb_raw_camera_info_delta_ms << ","
        << stereo_delta_ms << ","
        << latency_ms << "\n";

      csv_file_.flush();
    }
  }

  // Diagnostic status
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "sync_node_v2/fusion_status";

  bool core_ok = camera_found && lidar_found && imu_found && depth_found && odom_found;

  status.level = core_ok
    ? diagnostic_msgs::msg::DiagnosticStatus::OK
    : diagnostic_msgs::msg::DiagnosticStatus::WARN;

  status.message = "Depth-triggered fusion";

  auto add_status = [&](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  };

  add_status("trigger_source", trigger_source);
  add_status("camera_found", camera_found ? "true" : "false");
  add_status("lidar_found", lidar_found ? "true" : "false");
  add_status("gps_found", gps_found ? "true" : "false");
  add_status("imu_found", imu_found ? "true" : "false");
  add_status("depth_found", depth_found ? "true" : "false");
  add_status("odom_found", odom_found ? "true" : "false");
  add_status("heading_found", heading_found ? "true" : "false");
  add_status("rgb_raw_camera_info_found", rgb_raw_camera_info_found ? "true" : "false");
  add_status("stereo_found", stereo_found ? "true" : "false");
  add_status("gps_delta_ms", std::to_string(gps_delta_ms));
  add_status("heading_delta_ms", std::to_string(heading_delta_ms));
  add_status("fusion_latency_ms", std::to_string(latency_ms));

  status_pub_->publish(status);
}

// ================================================================
// Statistics
// ================================================================

void SyncNodeV2::log_stats()
{
  uint64_t cr = camera_received_.load();
  uint64_t lr = lidar_received_.load();
  uint64_t gr = gps_received_.load();
  uint64_t ir = imu_received_.load();
  uint64_t dr = depth_received_.load();
  uint64_t orr = odom_received_.load();
  uint64_t hr = heading_received_count_.load();
  uint64_t rcr = rgb_raw_camera_info_received_.load();
  uint64_t sr = stereo_received_.load();

  uint64_t cm = camera_matched_.load();
  uint64_t lm = lidar_matched_.load();
  uint64_t gm = gps_matched_.load();
  uint64_t im = imu_matched_.load();
  uint64_t dm = depth_matched_.load();
  uint64_t om = odom_matched_.load();
  uint64_t hm = heading_matched_.load();
  uint64_t rcm = rgb_raw_camera_info_matched_.load();
  uint64_t sm = stereo_matched_.load();

  uint64_t cb = camera_buffer_drops_.load();
  uint64_t lb = lidar_buffer_drops_.load();
  uint64_t ib = imu_buffer_drops_.load();
  uint64_t db = depth_buffer_drops_.load();
  uint64_t ob = odom_buffer_drops_.load();
  uint64_t rcb = rgb_raw_camera_info_buffer_drops_.load();
  uint64_t sb = stereo_buffer_drops_.load();

  double avg_latency = 0.0, min_latency = 0.0, max_latency = 0.0;
  {
    std::lock_guard<std::mutex> lock(latency_mutex_);
    if (latency_count_ > 0) {
      avg_latency = latency_sum_ms_ / static_cast<double>(latency_count_);
      min_latency = latency_min_ms_;
      max_latency = latency_max_ms_;
    }
  }

  double camera_match_pct = cr > 0 ? (100.0 * cm / cr) : 0.0;
  double lidar_match_pct = lr > 0 ? (100.0 * lm / lr) : 0.0;
  double gps_match_pct = gr > 0 ? (100.0 * gm / gr) : 0.0;
  double imu_match_pct = ir > 0 ? (100.0 * im / ir) : 0.0;
  double depth_match_pct = dr > 0 ? (100.0 * dm / dr) : 0.0;
  double odom_match_pct = orr > 0 ? (100.0 * om / orr) : 0.0;
  double heading_match_pct = hr > 0 ? (100.0 * hm / hr) : 0.0;
  double rgb_match_pct = rcr > 0 ? (100.0 * rcm / rcr) : 0.0;
  double stereo_match_pct = sr > 0 ? (100.0 * sm / sr) : 0.0;

  RCLCPP_INFO(get_logger(), "==================================================");
  RCLCPP_INFO(get_logger(), "SYNC STATISTICS (v2) - DEPTH TRIGGERED FUSION");
  RCLCPP_INFO(get_logger(), "Fusions: %lu", total_fusions_.load());
  RCLCPP_INFO(get_logger(), "Camera             : recv=%lu matched=%lu (%.1f%%) drops=%lu", cr, cm, camera_match_pct, cb);
  RCLCPP_INFO(get_logger(), "LiDAR              : recv=%lu matched=%lu (%.1f%%) drops=%lu", lr, lm, lidar_match_pct, lb);
  RCLCPP_INFO(get_logger(), "GPS                : recv=%lu matched=%lu (%.1f%%) [HOLD MODE]", gr, gm, gps_match_pct);
  RCLCPP_INFO(get_logger(), "IMU                : recv=%lu matched=%lu (%.1f%%) drops=%lu", ir, im, imu_match_pct, ib);
  RCLCPP_INFO(get_logger(), "Depth (trigger)    : recv=%lu matched=%lu (%.1f%%) drops=%lu", dr, dm, depth_match_pct, db);
  RCLCPP_INFO(get_logger(), "Odom               : recv=%lu matched=%lu (%.1f%%) drops=%lu", orr, om, odom_match_pct, ob);
  RCLCPP_INFO(get_logger(), "Heading            : recv=%lu matched=%lu (%.1f%%) [HOLD MODE]", hr, hm, heading_match_pct);
  RCLCPP_INFO(get_logger(), "RGB raw camera_info: recv=%lu matched=%lu (%.1f%%) drops=%lu", rcr, rcm, rgb_match_pct, rcb);
  RCLCPP_INFO(get_logger(), "Stereo image       : recv=%lu matched=%lu (%.1f%%) drops=%lu", sr, sm, stereo_match_pct, sb);
  RCLCPP_INFO(get_logger(), "Fusion latency: avg=%.3f ms | min=%.3f ms | max=%.3f ms", avg_latency, min_latency, max_latency);
  RCLCPP_INFO(get_logger(), "==================================================");

  // Diagnostic statistics
  diagnostic_msgs::msg::DiagnosticStatus msg;
  msg.name = "sync_node_v2/statistics";
  msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  msg.message = "Depth-triggered synchronization statistics";

  auto add = [&](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    msg.values.push_back(kv);
  };

  add("camera_match_pct", std::to_string(camera_match_pct));
  add("lidar_match_pct", std::to_string(lidar_match_pct));
  add("gps_match_pct", std::to_string(gps_match_pct));
  add("imu_match_pct", std::to_string(imu_match_pct));
  add("depth_match_pct", std::to_string(depth_match_pct));
  add("odom_match_pct", std::to_string(odom_match_pct));
  add("heading_match_pct", std::to_string(heading_match_pct));
  add("rgb_raw_camera_info_match_pct", std::to_string(rgb_match_pct));
  add("stereo_match_pct", std::to_string(stereo_match_pct));

  add("camera_buffer_drops", std::to_string(cb));
  add("lidar_buffer_drops", std::to_string(lb));
  add("imu_buffer_drops", std::to_string(ib));
  add("depth_buffer_drops", std::to_string(db));
  add("odom_buffer_drops", std::to_string(ob));
  add("rgb_raw_camera_info_buffer_drops", std::to_string(rcb));
  add("stereo_buffer_drops", std::to_string(sb));

  add("fusion_latency_avg_ms", std::to_string(avg_latency));
  add("fusion_latency_min_ms", std::to_string(min_latency));
  add("fusion_latency_max_ms", std::to_string(max_latency));

  loss_stats_pub_->publish(msg);
}

}  // namespace sync_node_pkg
