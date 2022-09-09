#include "LidarSlamNode.h"

//------------------------------------------------------------------------------
/*!
 * @brief Main node entry point.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create options for the node to use undeclared parameters
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.allow_undeclared_parameters(true);

  // Create lidar slam node, which subscribes to pointclouds coming from conversion node
  // and to external sensor messages in parallel.
  std::shared_ptr<LidarSlamNode> slamNode = std::make_shared<LidarSlamNode>("lidar_slam", options);

  // Handle callbacks until shut down
  rclcpp::spin(slamNode);

  return 0;
}