#include "LidarSlamNode.h"

//------------------------------------------------------------------------------
/*!
 * @brief Main node entry point.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("lidar_slam");
  rclcpp::Node::SharedPtr priv_nh = rclcpp::Node::make_shared("~");


  // Create lidar slam node, which subscribes to pointclouds coming from conversion node
  // and to external sensor messages in parallel.
  LidarSlamNode slam(nh, priv_nh);

  // Handle callbacks until shut down
  rclcpp::spin(nh);

  return 0;
}