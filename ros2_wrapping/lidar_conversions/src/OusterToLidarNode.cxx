//==============================================================================
// Copyright 2021-2022 Kitware, Inc., Kitware SAS
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

#include "OusterToLidarNode.h"
#include "Utilities.h"
#include <pcl_conversions/pcl_conversions.h>
#include "rmw/qos_profiles.h"

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_conversions
{

OusterToLidarNode::OusterToLidarNode(std::string node_name, const rclcpp::NodeOptions options)
  : Node(node_name, options)
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

  // Create custom QoS
  rclcpp::QoS custom_qos_profile(1); // History mode : only keep last
  // Put reliability to the same mode than Ouster Driver
  custom_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  // Init ROS subscrib
  this->Listener = this->create_subscription<Pcl2_msg>("/ouster/points", custom_qos_profile,
                                        std::bind(&OusterToLidarNode::Callback, this, std::placeholders::_1));

  RCLCPP_INFO_STREAM(this->get_logger(), BOLD_GREEN("Ouster data converter is ready !"));
}

//------------------------------------------------------------------------------
void OusterToLidarNode::Callback(const Pcl2_msg& msg_received)
{
  CloudV cloudO;
  pcl::fromROSMsg(msg_received, cloudO);

  // If input cloud is empty, ignore it
  if (cloudO.empty())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Input Ouster pointcloud is empty : frame ignored.");
    return;
  }

  // Init SLAM pointcloud
  CloudS cloudS;
  cloudS.reserve(cloudO.size());

  // Copy pointcloud metadata
  Utils::CopyPointCloudMetadata(cloudO, cloudS);

  // Check wether to use custom laser ID mapping or leave it untouched
  bool useLaserIdMapping = !this->LaserIdMapping.empty();

  // Helper to estimate frameAdvancement in case time field is invalid
  Utils::SpinningFrameAdvancementEstimator frameAdvancementEstimator;

  // Build SLAM pointcloud
  for (const PointO& ousterPoint : cloudO)
  {
    // Remove no return points by checking unvalid values (NaNs or zeros)
    if (!Utils::IsPointValid(ousterPoint))
      continue;

    PointS slamPoint;
    slamPoint.x = ousterPoint.x;
    slamPoint.y = ousterPoint.y;
    slamPoint.z = ousterPoint.z;
    slamPoint.intensity = ousterPoint.reflectivity;
    slamPoint.laser_id = useLaserIdMapping ? this->LaserIdMapping[ousterPoint.ring] : ousterPoint.ring;
    slamPoint.device_id = this->DeviceId;

    // Build approximate point-wise timestamp from azimuth angle
    // 'frameAdvancement' is 0 for first point, and should match 1 for last point
    // for a 360 degrees scan at ideal spinning frequency.
    // 'time' is the offset to add to 'header.stamp' to get approximate point-wise timestamp.
    // By default, 'header.stamp' is the timestamp of the last Veloydne packet,
    // but user can choose the first packet timestamp using parameter 'timestamp_first_packet'.
    double frameAdvancement = frameAdvancementEstimator(slamPoint);
    slamPoint.time = (this->TimestampFirstPacket ? frameAdvancement : frameAdvancement - 1) / this->Rpm * 60.;

    cloudS.push_back(slamPoint);
  }

  //conversion to msg
  Pcl2_msg msg_sended;
  pcl::toROSMsg(cloudS, msg_sended);

  this->Talker->publish(msg_sended);
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

  std::shared_ptr<lidar_conversions::OusterToLidarNode> v2s
    = std::make_shared<lidar_conversions::OusterToLidarNode>("ouster_conversion", options);

  rclcpp::spin(v2s);

  return 0;
}
