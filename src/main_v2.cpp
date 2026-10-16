#include "sync_node_pkg/sync_node_v2.hpp"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<sync_node_pkg::SyncNodeV2>();

  // Multi-threaded so the per-sensor callback groups really run in parallel:
  // the heavy depth/camera/stereo publishes in trigger_fusion() must not block
  // the lidar and IMU callbacks. Each group is MutuallyExclusive, so messages
  // of one sensor are still handled one at a time and in order.
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 8);

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
