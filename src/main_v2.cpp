#include <rclcpp/rclcpp.hpp>
#include "sync_node_pkg/sync_node_v2.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<sync_node_pkg::SyncNodeV2>();

  // MultiThreadedExecutor required for concurrent callbacks
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
