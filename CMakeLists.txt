cmake_minimum_required(VERSION 3.8)
project(sync_node_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)   # needed for /heading (QuaternionStamped)
find_package(diagnostic_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ---------------------------------------------------------------------------
# Needed by sync_node_v2 only (message_filters::Synchronizer + the
# rslidar_msg/RslidarScan type used for /rslidar_packets). If your RoboSense
# driver publishes /rslidar_packets under a different package name, swap
# "rslidar_msg" below for the correct one -- check with
# `ros2 pkg list | grep rslidar` or your driver's own package.xml.
# ---------------------------------------------------------------------------
find_package(message_filters REQUIRED)
find_package(rslidar_msg REQUIRED)

include_directories(include)

# ---------------------------------------------------------------------------
# Original node: sync_node
# NOTE: this assumes your existing entry-point file is named
# src/sync_node_main.cpp. If your real main() file has a different name,
# just change the source path below to match it — nothing else changes.
# ---------------------------------------------------------------------------
add_executable(sync_node
  src/sync_node.cpp
  src/main.cpp
)
ament_target_dependencies(sync_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  diagnostic_msgs
  builtin_interfaces
)

# ---------------------------------------------------------------------------
# New node: sync_node_v2 (9-topic ApproximateTimeSynchronizer)
# Built as a second, independent executable so it can run side by side with
# the original sync_node without touching it.
# ---------------------------------------------------------------------------
add_executable(sync_node_v2
  src/sync_node_v2.cpp
  src/main_v2.cpp
)
ament_target_dependencies(sync_node_v2
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  diagnostic_msgs
  builtin_interfaces
  message_filters
  rslidar_msg
)

install(TARGETS
  sync_node
  sync_node_v2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
