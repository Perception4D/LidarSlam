//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Arthur Bourbousson (Kitware SAS)
// Creation date: 2022-09-02
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//==============================================================================

#include "VelodyneToLidarNode.h"
#include "Utilities.h"
#include <pcl_conversions/pcl_conversions.h>

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_conversions
{

VelodyneToLidarNode::VelodyneToLidarNode(std::string node_name, const rclcpp::NodeOptions options)
  : rclcpp::Node(node_name, options)
{
  // Get laser ID mapping
  this->get_parameter("laser_id_mapping", this->LaserIdMapping);

  //  Get LiDAR id
  this->get_parameter("device_id", this->DeviceId);

  //  Get LiDAR spinning speed and first timestamp option
  this->get_parameter("rpm", this->Rpm);
  this->get_parameter("timestamp_first_packet", this->TimestampFirstPacket);

  // Init ROS publisher
  this->Talker = this->create_publisher<Pcl2_msg>("lidar_points", 1);

  // Init ROS subscriber
  this->Listener = this->create_subscription<Pcl2_msg>("velodyne_points", 1,
                                            std::bind(&VelodyneToLidarNode::Callback, this, std::placeholders::_1));

  RCLCPP_INFO_STREAM(this->get_logger(), BOLD_GREEN( "Velodyne data converter is ready !"));
}

//------------------------------------------------------------------------------
void VelodyneToLidarNode::Callback(const Pcl2_msg& msg_received)
{
  //convertion message to PointCloud
  //doc : https://github.com/mikeferguson/ros2_cookbook/blob/main/rclcpp/pcl.md
  CloudV cloudV;
  pcl::fromROSMsg(msg_received, cloudV);

  // If input cloud is empty, ignore it
  if (cloudV.empty())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Input Velodyne pointcloud is empty : frame ignored.");
    return;
  }

  // Init SLAM pointcloud
  CloudS cloudS;
  cloudS.reserve(cloudV.size());

  // Copy pointcloud metadata
  Utils::CopyPointCloudMetadata(cloudV, cloudS);

  // Check wether to use custom laser ID mapping or leave it untouched
  bool useLaserIdMapping = !this->LaserIdMapping.empty();

  // Check if time field looks properly set
  // If first and last points have same timestamps, this is not normal
  bool isTimeValid = cloudV.back().time - cloudV.front().time > 1e-8;
  if (!isTimeValid)
    RCLCPP_WARN_STREAM(this->get_logger(), "Invalid 'time' field, it will be built from azimuth advancement.");

  // Helper to estimate frameAdvancement in case time field is invalid
  Utils::SpinningFrameAdvancementEstimator frameAdvancementEstimator;

  // Build SLAM pointcloud
  for (const PointV& velodynePoint : cloudV)
  {
    PointS slamPoint;
    slamPoint.x = velodynePoint.x;
    slamPoint.y = velodynePoint.y;
    slamPoint.z = velodynePoint.z;
    slamPoint.intensity = velodynePoint.intensity;
    slamPoint.laser_id = useLaserIdMapping ? this->LaserIdMapping[velodynePoint.ring] : velodynePoint.ring;
    slamPoint.device_id = this->DeviceId;

    // Use time field if available
    // time is the offset to add to header.stamp to get point-wise timestamp
    if (isTimeValid)
      slamPoint.time = velodynePoint.time;

    // Build approximate point-wise timestamp from azimuth angle
    // 'frameAdvancement' is 0 for first point, and should match 1 for last point
    // for a 360 degrees scan at ideal spinning frequency.
    // 'time' is the offset to add to 'header.stamp' to get approximate point-wise timestamp.
    // By default, 'header.stamp' is the timestamp of the last Veloydne packet,
    // but user can choose the first packet timestamp using parameter 'timestamp_first_packet'.
    else
    {
      double frameAdvancement = frameAdvancementEstimator(slamPoint);
      slamPoint.time = (this->TimestampFirstPacket ? frameAdvancement : frameAdvancement - 1) / this->Rpm * 60.;
    }

    cloudS.push_back(slamPoint);
  }

  //convertion PointCloud to msg
  Pcl2_msg msg_sent;
  pcl::toROSMsg(cloudS, msg_sent);

  this->Talker->publish(msg_sent);
}

}  // end of namespace lidar_conversions

//------------------------------------------------------------------------------
/*!
 * @brief Main node entry point.
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Create options for the node to use undeclared parameters
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.allow_undeclared_parameters(true);

  std::shared_ptr<lidar_conversions::VelodyneToLidarNode> v2s
      = std::make_shared<lidar_conversions::VelodyneToLidarNode>("velodyne_conversion", options);

  rclcpp::spin(v2s);

  return 0;
}
