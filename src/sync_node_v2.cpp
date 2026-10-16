#include "sync_node_pkg/sync_node_v2.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <vector>
#include <filesystem>

namespace sync_node_pkg
{

namespace
{

// Adds offset_sec to a FLOAT64 per-point time field (e.g. RoboSense
// "timestamp", which holds absolute lidar-clock seconds). Returns false and
// sets `why` if the field is missing or not a shiftable absolute FLOAT64.
bool shift_point_time_field(
  sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & name,
  double offset_sec,
  std::string & why)
{
  const sensor_msgs::msg::PointField * field = nullptr;
  for (const auto & f : cloud.fields) {
    if (f.name == name) {
      field = &f;
      break;
    }
  }

  if (field == nullptr) {
    why = "field '" + name + "' not found in cloud";
    return false;
  }

  if (field->datatype != sensor_msgs::msg::PointField::FLOAT64) {
    why = "field '" + name + "' is not FLOAT64 (relative/other time formats are not shifted)";
    return false;
  }

  if (cloud.is_bigendian) {
    why = "big-endian clouds are not supported";
    return false;
  }

  if (field->offset + sizeof(double) > cloud.point_step) {
    why = "field offset is outside point_step";
    return false;
  }

  const size_t n = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
  const size_t step = cloud.point_step;

  for (size_t i = 0; i < n; ++i) {
    const size_t pos = i * step + field->offset;
    if (pos + sizeof(double) > cloud.data.size()) {
      break;
    }
    double t;
    std::memcpy(&t, cloud.data.data() + pos, sizeof(double));
    t += offset_sec;
    std::memcpy(cloud.data.data() + pos, &t, sizeof(double));
  }

  return true;
}

}  // namespace

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

  // LiDAR clock-domain correction
  declare_parameter<std::string>("lidar_time_mode", "auto");
  declare_parameter<double>("lidar_auto_threshold_sec", 5.0);
  declare_parameter<int>("lidar_offset_window", 500);
  declare_parameter<double>("lidar_time_trim_sec", 0.0);
  declare_parameter<double>("lidar_fixed_offset_sec", 0.0);
  declare_parameter<double>("lidar_offset_percentile", 0.9);
  declare_parameter<double>("lidar_live_percentile", 0.05);
  declare_parameter<double>("live_detect_threshold_sec", 5.0);
  // Raised from 1.0: depth-reference samples are noisy by >1 s on replay,
  // and 1.0 caused the offset window to be wiped over and over.
  declare_parameter<double>("lidar_reset_jump_sec", 5.0);
  // Number of CONSECUTIVE disagreeing samples required before the window
  // is cleared (a real power-cycle gives a steady stream of them, a noise
  // spike gives one).
  declare_parameter<int>("lidar_reset_confirm_samples", 10);
  // REPLAY only: once this many samples are collected the offset is locked,
  // so changing delivery lag during replay cannot make it drift. 0 = never lock.
  declare_parameter<int>("lidar_freeze_after_samples", 100);
  // true (default): each lidar cloud is matched at most once - the unused
  // cloud closest to the depth stamp (within max_lidar_delta_sec) is used and
  // removed, so /synced/lidar never repeats a scan. Other clouds stay available
  // for the next fusion. With depth faster than the lidar (e.g. 30 Hz vs 5 Hz)
  // only some fusions carry a cloud.
  // false: the nearest cloud stays in the buffer and may be reused.
  declare_parameter<bool>("lidar_consume_on_match", true);
  declare_parameter<bool>("restamp_lidar", true);

  // Per-point time shift applied together with the header restamp.
  declare_parameter<bool>("shift_point_timestamps", true);
  declare_parameter<std::string>("point_time_field", "timestamp");

  // Full-rate in-order restamped lidar output (replaces a separate restamp node)
  declare_parameter<bool>("publish_restamped_lidar", true);
  // Odometry input topic (default ZED visual odometry; use "/odometry" for lidar odometry)
  declare_parameter<std::string>("odom_topic", "/zed/zed_node/odom");
  declare_parameter<std::string>("restamped_lidar_topic", "/rslidar_points_zed");
  // Reference for the clock-offset estimate: "imu" (default) or "depth"
  declare_parameter<std::string>("lidar_ref_source", "imu");

  declare_parameter<std::string>(
    "output_directory",
    "/home/scl-ncai/ros2_ws/src/Sync-Node-For-Multi-Sensors/sync_data");

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

  lidar_time_mode_ =
    get_parameter("lidar_time_mode").as_string();

  lidar_auto_threshold_sec_ =
    get_parameter("lidar_auto_threshold_sec").as_double();

  {
    const int w = get_parameter("lidar_offset_window").as_int();
    lidar_offset_window_size_ = static_cast<size_t>(w < 1 ? 1 : w);
  }

  lidar_time_trim_sec_ =
    get_parameter("lidar_time_trim_sec").as_double();

  lidar_fixed_offset_sec_ =
    get_parameter("lidar_fixed_offset_sec").as_double();

  lidar_offset_percentile_ =
    std::min(1.0, std::max(0.0, get_parameter("lidar_offset_percentile").as_double()));

  lidar_live_percentile_ =
    std::min(1.0, std::max(0.0, get_parameter("lidar_live_percentile").as_double()));

  live_detect_threshold_sec_ =
    get_parameter("live_detect_threshold_sec").as_double();

  lidar_reset_jump_sec_ =
    get_parameter("lidar_reset_jump_sec").as_double();

  {
    const int n = get_parameter("lidar_reset_confirm_samples").as_int();
    lidar_reset_confirm_samples_ = n < 1 ? 1 : n;
  }

  {
    const int n = get_parameter("lidar_freeze_after_samples").as_int();
    lidar_freeze_after_samples_ = n < 0 ? 0 : n;
  }

  lidar_consume_on_match_ =
    get_parameter("lidar_consume_on_match").as_bool();

  restamp_lidar_ =
    get_parameter("restamp_lidar").as_bool();

  shift_point_timestamps_ =
    get_parameter("shift_point_timestamps").as_bool();

  point_time_field_ =
    get_parameter("point_time_field").as_string();

  publish_restamped_lidar_ =
    get_parameter("publish_restamped_lidar").as_bool();

  restamped_lidar_topic_ =
    get_parameter("restamped_lidar_topic").as_string();

  lidar_ref_source_ =
    get_parameter("lidar_ref_source").as_string();

  if (lidar_ref_source_ != "imu" && lidar_ref_source_ != "depth") {
    RCLCPP_WARN(
      get_logger(),
      "Unknown lidar_ref_source '%s' - falling back to 'imu'",
      lidar_ref_source_.c_str());
    lidar_ref_source_ = "imu";
  }
  lidar_use_imu_ref_ = (lidar_ref_source_ == "imu");

  if (lidar_time_mode_ != "auto" &&
    lidar_time_mode_ != "offset" &&
    lidar_time_mode_ != "fixed" &&
    lidar_time_mode_ != "passthrough")
  {
    RCLCPP_WARN(
      get_logger(),
      "Unknown lidar_time_mode '%s' - falling back to 'auto'",
      lidar_time_mode_.c_str());
    lidar_time_mode_ = "auto";
  }

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
      << "gps_reused,"
      << "imu_found,"
      << "depth_found,"
      << "odom_found,"
      << "vel_found,"
      << "rgb_raw_camera_info_found,"
      << "stereo_found,"
      << "camera_delta_ms,"
      << "lidar_delta_ms,"
      << "imu_delta_ms,"
      << "depth_delta_ms,"
      << "odom_delta_ms,"
      << "gps_delta_ms,"
      << "vel_delta_ms,"
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

  vel_cb_group_ =
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

  rclcpp::SubscriptionOptions vel_opts;
  vel_opts.callback_group = vel_cb_group_;

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
      rclcpp::QoS(rclcpp::KeepLast(100)).best_effort(),
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
      rclcpp::QoS(rclcpp::KeepLast(1000)).best_effort(),
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
      get_parameter("odom_topic").as_string(),
      rclcpp::SensorDataQoS(),
      std::bind(
        &SyncNodeV2::odom_callback,
        this,
        std::placeholders::_1),
      odom_opts);

  // GPS-derived velocity subscription (renamed from "heading" - this
  // topic actually carries velocity data published by the GPS, not heading)
  vel_sub_ =
    create_subscription<VelMsg>(
      "/vel",
      rclcpp::SensorDataQoS(),
      std::bind(
        &SyncNodeV2::vel_callback,
        this,
        std::placeholders::_1),
      vel_opts);

  rgb_raw_camera_info_sub_ =
    create_subscription<CameraInfoMsg>(
      "/zed/zed_node/rgb/camera_info",
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

  if (publish_restamped_lidar_) {
    restamped_lidar_pub_ =
      create_publisher<PointCloudMsg>(
        restamped_lidar_topic_,
        rclcpp::QoS(rclcpp::KeepLast(100)).best_effort());
  }

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

  synced_vel_pub_ =
    create_publisher<VelMsg>(
      "/synced/vel", 10);

  synced_rgb_raw_camera_info_pub_ =
    create_publisher<CameraInfoMsg>(
      "/synced/zed/rgb_camera_info", 10);

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
    "CSV output: %s",
    csv_path.c_str());

  RCLCPP_INFO(
    get_logger(),
    "Odometry input: %s",
    get_parameter("odom_topic").as_string().c_str());

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
// LiDAR callback - STORE + CLOCK-OFFSET ESTIMATION
//
// The lidar stamp may be in a different time domain than the ZED
// (sensor uptime ~20012 s vs. epoch ~1.77e9 s), and that offset is
// different on every power-up, so it is estimated online:
//   LIVE   : sample = wall_arrival - lidar_stamp   (low percentile)
//   REPLAY : sample = latest_depth_stamp - lidar_stamp (high percentile)
// LIVE vs REPLAY is detected by comparing the wall clock with the
// newest ZED stamp. The offset is applied at match time in
// find_closest_lidar().
//
// Outlier handling: a single sample that disagrees with the current
// estimate is IGNORED. Only lidar_reset_confirm_samples_ consecutive
// disagreeing samples (a real clock change) clear the window.
// ================================================================

void SyncNodeV2::lidar_callback(const PointCloudMsg::SharedPtr msg)
{
  const int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  const int64_t trim_ns = static_cast<int64_t>(lidar_time_trim_sec_ * 1e9);

  if (lidar_time_mode_ == "fixed") {
    lidar_offset_ns_.store(
      static_cast<int64_t>(lidar_fixed_offset_sec_ * 1e9) + trim_ns);
    lidar_offset_valid_.store(true);
  } else if (lidar_time_mode_ == "passthrough") {
    lidar_offset_ns_.store(trim_ns);
    lidar_offset_valid_.store(true);
  } else {
    const int64_t wall_ns = system_clock_.now().nanoseconds();
    // Reference stamp: newest ZED IMU stamp (default) or newest depth stamp
    const int64_t depth_ref_ns =
      lidar_use_imu_ref_ ? latest_imu_stamp_ns_.load() : latest_depth_stamp_ns_.load();

    if (depth_ref_ns != 0) {
      // LIVE if the ZED stamps track the host wall clock
      const bool live =
        std::abs(static_cast<double>(wall_ns - depth_ref_ns)) * 1e-9 <
        live_detect_threshold_sec_;

      const int64_t ref_ns = live ? wall_ns : depth_ref_ns;
      const int64_t sample_ns = ref_ns - stamp_ns;

      std::lock_guard<std::mutex> lock(lidar_offset_mutex_);

      bool use_sample = true;

      if (!lidar_offset_window_ns_.empty()) {
        const bool ref_changed = (lidar_window_live_ != live);
        const bool jumped =
          std::abs(static_cast<double>(sample_ns - lidar_last_est_ns_)) * 1e-9 >
          lidar_reset_jump_sec_;

        if (ref_changed) {
          // LIVE <-> REPLAY flip: the old samples use another reference.
          lidar_offset_window_ns_.clear();
          lidar_jump_count_ = 0;
          lidar_offset_frozen_ = false;
        } else if (jumped) {
          if (++lidar_jump_count_ >= lidar_reset_confirm_samples_) {
            // Sustained disagreement -> real clock change (power-cycle,
            // driver restart). Start over with the new domain.
            lidar_offset_window_ns_.clear();
            lidar_jump_count_ = 0;
            lidar_offset_frozen_ = false;
          } else {
            // Probably a noise spike: ignore this sample, keep the offset.
            use_sample = false;
          }
        } else {
          lidar_jump_count_ = 0;
        }
      }

      // REPLAY with a locked offset: keep it, ignore further samples.
      if (use_sample && lidar_offset_frozen_ && !live) {
        use_sample = false;
      }

      if (use_sample) {
        lidar_window_live_ = live;

        lidar_offset_window_ns_.push_back(sample_ns);
        while (lidar_offset_window_ns_.size() > lidar_offset_window_size_) {
          lidar_offset_window_ns_.pop_front();
        }

        std::vector<int64_t> tmp(
          lidar_offset_window_ns_.begin(),
          lidar_offset_window_ns_.end());

        // LIVE: low percentile = least transport delay.
        // REPLAY with DEPTH reference: the reference is "stale" by 0..1 depth
        // period, which only pushes samples DOWN, so use a high percentile.
        // IMU reference (any mode): the reference is only a few ms stale, so
        // the dominant error is lidar delivery lag, which only pushes samples
        // UP (sample = newest IMU stamp - lidar stamp). Use the LOW percentile
        // so the estimate is not biased by clouds that arrived late.
        const double p =
          (live || lidar_use_imu_ref_) ? lidar_live_percentile_ : lidar_offset_percentile_;
        const size_t idx = static_cast<size_t>(
          p * static_cast<double>(tmp.size() - 1));
        auto mid = tmp.begin() + static_cast<std::ptrdiff_t>(idx);
        std::nth_element(tmp.begin(), mid, tmp.end());
        const int64_t est_ns = *mid;
        lidar_last_est_ns_ = est_ns;

        if (lidar_freeze_after_samples_ > 0 && !live &&
          lidar_offset_window_ns_.size() >=
          static_cast<size_t>(lidar_freeze_after_samples_))
        {
          lidar_offset_frozen_ = true;
        }

        int64_t offset_ns = 0;
        if (lidar_time_mode_ == "offset" ||
          std::abs(static_cast<double>(est_ns)) * 1e-9 > lidar_auto_threshold_sec_)
        {
          offset_ns = est_ns;
        }

        lidar_offset_ns_.store(offset_ns + trim_ns);
        lidar_ref_live_.store(live);
        lidar_offset_valid_.store(true);
      }
    }
  }

  // Full-rate, in-order restamped copy of EVERY cloud (header + per-point
  // time on the ZED clock). Clouds that arrive before the first offset exists
  // are dropped so an unshifted cloud is never published.
  if (publish_restamped_lidar_ && restamped_lidar_pub_) {
    if (lidar_offset_valid_.load()) {
      auto out = std::make_unique<PointCloudMsg>(*msg);
      const int64_t shift_ns = lidar_offset_ns_.load();

      out->header.stamp = rclcpp::Time(stamp_ns + shift_ns, RCL_ROS_TIME);

      if (shift_point_timestamps_) {
        std::string why;
        if (!shift_point_time_field(
            *out, point_time_field_, static_cast<double>(shift_ns) * 1e-9, why))
        {
          if (!point_time_warned_.exchange(true)) {
            RCLCPP_WARN(
              get_logger(),
              "Per-point time not shifted: %s (header stamp still restamped)",
              why.c_str());
          }
        }
      }

      restamped_lidar_pub_->publish(std::move(out));
      restamped_published_++;
    } else {
      restamped_dropped_++;
    }
  }

  LidarEntry entry;
  entry.msg = msg;
  entry.raw_stamp_ns = stamp_ns;

  std::lock_guard<std::mutex> lock(lidar_mutex_);
  lidar_buffer_.push_back(std::move(entry));

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
// currently held, same pattern as vel_callback() below.
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
  latest_imu_stamp_ns_.store(rclcpp::Time(msg->header.stamp).nanoseconds());

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
  latest_depth_stamp_ns_.store(rclcpp::Time(msg->header.stamp).nanoseconds());

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
// Vel callback - HOLD MODE
// ================================================================

void SyncNodeV2::vel_callback(const VelMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(vel_mutex_);
  latest_vel_ = msg;
  vel_received_ = true;
  vel_received_count_++;
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
// LiDAR matching - uses the corrected (host-domain) stamp
//
// After a successful match, every cloud that is not newer than the
// matched one is pruned: depth triggers arrive in increasing order,
// so those clouds can never be the closest match again. They are
// counted in lidar_skipped_ (normal when the lidar runs faster than
// the depth trigger) instead of piling up until the buffer overflows.
// ================================================================

bool SyncNodeV2::find_closest_lidar(
  const rclcpp::Time & target,
  PointCloudMsg::ConstSharedPtr & result,
  rclcpp::Time & corrected_stamp)
{
  std::lock_guard<std::mutex> lock(lidar_mutex_);

  if (lidar_buffer_.empty()) {
    lidar_miss_empty_++;
    return false;
  }

  const int64_t offset_ns = lidar_offset_ns_.load();

  auto corrected = [&](const LidarEntry & e) {
      return rclcpp::Time(e.raw_stamp_ns + offset_ns, RCL_ROS_TIME);
    };

  auto closest =
    std::min_element(
      lidar_buffer_.begin(),
      lidar_buffer_.end(),
      [&](const LidarEntry & a, const LidarEntry & b)
      {
        return timestamp_difference(corrected(a), target)
             < timestamp_difference(corrected(b), target);
      });

  const rclcpp::Time closest_stamp = corrected(*closest);
  double delta = timestamp_difference(closest_stamp, target);

  if (delta > max_lidar_delta_sec_) {
    lidar_miss_far_++;
    const int64_t d_ns = static_cast<int64_t>(delta * 1e9);
    if (d_ns > lidar_worst_miss_ns_.load()) {
      lidar_worst_miss_ns_.store(d_ns);
    }
    return false;
  }

  // Copy everything needed BEFORE erasing (iterators get invalidated).
  result = closest->msg;
  corrected_stamp = closest_stamp;

  if (closest->used) {
    lidar_reused_++;
  }
  closest->used = true;

  size_t skipped_now = 0;

  if (lidar_consume_on_match_) {
    // Remove ONLY the matched cloud. Older/newer unused clouds stay, so if two
    // depth frames come in back to back, the second one still finds a cloud
    // (the next-closest unused one within max_lidar_delta_sec) instead of an
    // empty buffer. No scan is ever published twice.
    lidar_buffer_.erase(closest);
  }

  // Prune clouds too old to match any future depth frame (depth stamps only
  // increase). Never-matched ones are counted as skipped.
  {
    const int64_t stale_before_ns =
      target.nanoseconds() - static_cast<int64_t>(max_lidar_delta_sec_ * 1e9);

    lidar_buffer_.erase(
      std::remove_if(
        lidar_buffer_.begin(),
        lidar_buffer_.end(),
        [&](const LidarEntry & e) {
          if (e.raw_stamp_ns + offset_ns < stale_before_ns) {
            if (!e.used) {
              ++skipped_now;
            }
            return true;
          }
          return false;
        }),
      lidar_buffer_.end());
  }

  lidar_skipped_ += skipped_now;
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
  rclcpp::Time lidar_corrected_stamp(0, 0, RCL_ROS_TIME);
  NavSatFixMsg::ConstSharedPtr gps;
  ImuMsg::ConstSharedPtr imu;
  ImageMsg::ConstSharedPtr depth;
  OdometryMsg::ConstSharedPtr odom;
  VelMsg::ConstSharedPtr vel;
  CameraInfoMsg::ConstSharedPtr rgb_raw_camera_info;
  ImageMsg::ConstSharedPtr stereo;

  bool camera_found = find_closest_camera(timestamp, camera);
  bool lidar_found = find_closest_lidar(timestamp, lidar, lidar_corrected_stamp);
  bool imu_found = find_closest_imu(timestamp, imu);
  bool depth_found = find_closest_depth(timestamp, depth);
  bool odom_found = find_closest_odom(timestamp, odom);
  bool rgb_raw_camera_info_found = find_closest_rgb_raw_camera_info(timestamp, rgb_raw_camera_info);
  bool stereo_found = find_closest_stereo(timestamp, stereo);

  // GPS - HOLD MODE: always use the latest received fix, no
  // time-delta cutoff. Repeats previous value until a new
  // /fix message arrives.
  bool gps_found = false;
  bool gps_reused = false;
  {
    std::lock_guard<std::mutex> lock(gps_mutex_);
    if (gps_received_flag_ && latest_gps_) {
      gps = latest_gps_;
      gps_found = true;
      gps_matched_++;

      // A fix counts as "reused" if it's the same message object we
      // already consumed in a previous fusion cycle (i.e. no new /fix
      // arrived since then), not merely because it's outside a delta
      // window - GPS hold mode has no window.
      if (last_fused_gps_ == gps) {
        gps_reused = true;
        gps_reused_++;
      }
      last_fused_gps_ = gps;
    }
  }

  // Vel - HOLD MODE: same pattern as GPS.
  bool vel_found = false;
  {
    std::lock_guard<std::mutex> lock(vel_mutex_);
    if (vel_received_ && latest_vel_) {
      vel = latest_vel_;
      vel_found = true;
      vel_matched_++;
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
    if (restamp_lidar_) {
      // Publish with the stamp translated into the ZED/host time domain.
      // The per-point time field (absolute lidar-clock seconds) is shifted by
      // the SAME amount so header and point times stay consistent.
      auto out = std::make_unique<PointCloudMsg>(*lidar);

      const int64_t raw_ns = rclcpp::Time(lidar->header.stamp).nanoseconds();
      const int64_t shift_ns = lidar_corrected_stamp.nanoseconds() - raw_ns;

      out->header.stamp = lidar_corrected_stamp;

      if (shift_point_timestamps_) {
        std::string why;
        if (!shift_point_time_field(
            *out, point_time_field_, static_cast<double>(shift_ns) * 1e-9, why))
        {
          if (!point_time_warned_.exchange(true)) {
            RCLCPP_WARN(
              get_logger(),
              "Per-point time not shifted: %s (header stamp still restamped)",
              why.c_str());
          }
        }
      }

      synced_lidar_pub_->publish(std::move(out));
    } else {
      synced_lidar_pub_->publish(*lidar);
    }
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

  if (vel_found && vel) {
    synced_vel_pub_->publish(*vel);
  }

  if (rgb_raw_camera_info_found && rgb_raw_camera_info) {
    synced_rgb_raw_camera_info_pub_->publish(*rgb_raw_camera_info);
  }

  if (stereo_found && stereo) {
    synced_stereo_pub_->publish(*stereo);
  }

  // Timestamp errors (informational only for GPS/vel -
  // these are hold-mode and are published regardless of delta)
  double camera_delta_ms = -1.0;
  double lidar_delta_ms = -1.0;
  double imu_delta_ms = -1.0;
  double depth_delta_ms = -1.0;
  double odom_delta_ms = -1.0;
  double gps_delta_ms = -1.0;
  double vel_delta_ms = -1.0;
  double rgb_raw_camera_info_delta_ms = -1.0;
  double stereo_delta_ms = -1.0;

  if (camera_found) {
    camera_delta_ms = timestamp_difference(
      rclcpp::Time(camera->header.stamp), timestamp) * 1000.0;
  }

  if (lidar_found) {
    // Delta measured with the corrected stamp (same domain as depth)
    lidar_delta_ms = timestamp_difference(
      lidar_corrected_stamp, timestamp) * 1000.0;
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

  if (vel_found) {
    vel_delta_ms = timestamp_difference(
      rclcpp::Time(vel->header.stamp), timestamp) * 1000.0;
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
        << gps_reused << ","
        << imu_found << ","
        << depth_found << ","
        << odom_found << ","
        << vel_found << ","
        << rgb_raw_camera_info_found << ","
        << stereo_found << ","
        << std::setprecision(4)
        << camera_delta_ms << ","
        << lidar_delta_ms << ","
        << imu_delta_ms << ","
        << depth_delta_ms << ","
        << odom_delta_ms << ","
        << gps_delta_ms << ","
        << vel_delta_ms << ","
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
  add_status("gps_reused", gps_reused ? "true" : "false");
  add_status("imu_found", imu_found ? "true" : "false");
  add_status("depth_found", depth_found ? "true" : "false");
  add_status("odom_found", odom_found ? "true" : "false");
  add_status("vel_found", vel_found ? "true" : "false");
  add_status("rgb_raw_camera_info_found", rgb_raw_camera_info_found ? "true" : "false");
  add_status("stereo_found", stereo_found ? "true" : "false");
  add_status("lidar_clock_offset_sec", std::to_string(static_cast<double>(lidar_offset_ns_.load()) * 1e-9));
  add_status("gps_delta_ms", std::to_string(gps_delta_ms));
  add_status("vel_delta_ms", std::to_string(vel_delta_ms));
  add_status("fusion_latency_ms", std::to_string(latency_ms));

  status_pub_->publish(status);
}

// ================================================================
// Statistics
// ================================================================

void SyncNodeV2::log_stats()
{
  uint64_t tf = total_fusions_.load();

  uint64_t cr = camera_received_.load();
  uint64_t lr = lidar_received_.load();
  uint64_t gr = gps_received_.load();
  uint64_t ir = imu_received_.load();
  uint64_t dr = depth_received_.load();
  uint64_t orr = odom_received_.load();
  uint64_t hr = vel_received_count_.load();
  uint64_t rcr = rgb_raw_camera_info_received_.load();
  uint64_t sr = stereo_received_.load();

  uint64_t cm = camera_matched_.load();
  uint64_t lm = lidar_matched_.load();
  uint64_t gm = gps_matched_.load();
  uint64_t gru = gps_reused_.load();
  uint64_t im = imu_matched_.load();
  uint64_t dm = depth_matched_.load();
  uint64_t om = odom_matched_.load();
  uint64_t hm = vel_matched_.load();
  uint64_t rcm = rgb_raw_camera_info_matched_.load();
  uint64_t sm = stereo_matched_.load();

  uint64_t cb = camera_buffer_drops_.load();
  uint64_t lb = lidar_buffer_drops_.load();
  uint64_t ls = lidar_skipped_.load();
  uint64_t ib = imu_buffer_drops_.load();
  uint64_t db = depth_buffer_drops_.load();
  uint64_t ob = odom_buffer_drops_.load();
  uint64_t rcb = rgb_raw_camera_info_buffer_drops_.load();
  uint64_t sb = stereo_buffer_drops_.load();

  const double lidar_offset_sec = static_cast<double>(lidar_offset_ns_.load()) * 1e-9;

  double avg_latency = 0.0, min_latency = 0.0, max_latency = 0.0;
  {
    std::lock_guard<std::mutex> lock(latency_mutex_);
    if (latency_count_ > 0) {
      avg_latency = latency_sum_ms_ / static_cast<double>(latency_count_);
      min_latency = latency_min_ms_;
      max_latency = latency_max_ms_;
    }
  }

  // Every percentage below is matched / total fusions (= depth triggers).
  // Counters are read without a common lock, so clamp to 100.
  auto pct_of_fusions = [tf](uint64_t matched) {
      return tf > 0 ? std::min(100.0, 100.0 * static_cast<double>(matched) / static_cast<double>(tf)) : 0.0;
    };

  double camera_match_pct = pct_of_fusions(cm);
  double lidar_match_pct = pct_of_fusions(lm);
  double gps_match_pct = pct_of_fusions(gm);
  double gps_reused_pct = gm > 0 ? (100.0 * gru / gm) : 0.0;
  double imu_match_pct = pct_of_fusions(im);
  double depth_match_pct = pct_of_fusions(dm);
  double odom_match_pct = pct_of_fusions(om);
  double vel_match_pct = pct_of_fusions(hm);
  double rgb_match_pct = pct_of_fusions(rcm);
  double stereo_match_pct = pct_of_fusions(sm);

  RCLCPP_INFO(get_logger(), "==================================================");
  RCLCPP_INFO(get_logger(), "SYNC STATISTICS (v2) - DEPTH TRIGGERED FUSION");
  RCLCPP_INFO(get_logger(), "Fusions: %lu", total_fusions_.load());
  RCLCPP_INFO(get_logger(), "Camera             : recv=%lu matched=%lu (%.1f%% of fusions) drops=%lu", cr, cm, camera_match_pct, cb);
  RCLCPP_INFO(
    get_logger(),
    "LiDAR              : recv=%lu matched=%lu (%.1f%% of fusions)",
    lr, lm, lidar_match_pct);
  RCLCPP_INFO(get_logger(), "GPS                : recv=%lu matched=%lu (%.1f%% of fusions)", gr, gm, gps_match_pct);
  RCLCPP_INFO(get_logger(), "IMU                : recv=%lu matched=%lu (%.1f%% of fusions) drops=%lu", ir, im, imu_match_pct, ib);
  RCLCPP_INFO(get_logger(), "Depth (trigger)    : recv=%lu matched=%lu (%.1f%% of fusions) drops=%lu", dr, dm, depth_match_pct, db);
  RCLCPP_INFO(get_logger(), "Odom               : recv=%lu matched=%lu (%.1f%% of fusions) drops=%lu", orr, om, odom_match_pct, ob);
  RCLCPP_INFO(get_logger(), "Vel                : recv=%lu matched=%lu (%.1f%% of fusions)", hr, hm, vel_match_pct);
  RCLCPP_INFO(get_logger(), "RGB raw camera_info: recv=%lu matched=%lu (%.1f%% of fusions) drops=%lu", rcr, rcm, rgb_match_pct, rcb);
  RCLCPP_INFO(get_logger(), "Stereo image       : recv=%lu matched=%lu (%.1f%% of fusions) drops=%lu", sr, sm, stereo_match_pct, sb);
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
  add("lidar_clock_offset_sec", std::to_string(lidar_offset_sec));
  add("gps_fusion_coverage_pct", std::to_string(gps_match_pct));
  add("gps_reused_pct", std::to_string(gps_reused_pct));
  add("imu_match_pct", std::to_string(imu_match_pct));
  add("depth_match_pct", std::to_string(depth_match_pct));
  add("odom_match_pct", std::to_string(odom_match_pct));
  add("vel_fusion_coverage_pct", std::to_string(vel_match_pct));
  add("rgb_raw_camera_info_match_pct", std::to_string(rgb_match_pct));
  add("stereo_match_pct", std::to_string(stereo_match_pct));

  add("camera_buffer_drops", std::to_string(cb));
  add("lidar_buffer_drops", std::to_string(lb));
  add("lidar_skipped", std::to_string(ls));
  add("lidar_restamped_published", std::to_string(restamped_published_.load()));
  add("lidar_restamped_dropped_no_offset", std::to_string(restamped_dropped_.load()));
  add("lidar_reused", std::to_string(lidar_reused_.load()));
  add("lidar_miss_buffer_empty", std::to_string(lidar_miss_empty_.load()));
  add("lidar_miss_too_far", std::to_string(lidar_miss_far_.load()));
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
