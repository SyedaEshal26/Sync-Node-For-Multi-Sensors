#include "sync_node_pkg/sync_node_v2.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<sync_node_pkg::SyncNodeV2>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
