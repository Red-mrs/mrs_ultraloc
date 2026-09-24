/*
 * Standalone entry point.
 *
 * The node only touches shared state from the two subscriptions and from its own
 * publish timer, all of which are serialised by a mutex, so a single-threaded
 * executor is enough and keeps the ordering of callbacks deterministic.
 */

#include <mrs_ultraloc/uwb_uvdar_fusion_node.h>

#include <memory>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mrs_ultraloc::UwbUvdarFusionNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();

  return 0;
}
