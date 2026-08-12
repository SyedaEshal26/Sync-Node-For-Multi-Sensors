#include "sync_node_pkg/sync_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<sync_node_pkg::SyncNode>();

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(node);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
