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

#include "RobosenseToLidarNode.h"

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_conversions
{
namespace
{
  // Mapping between RSLidar laser id and vertical laser id
  // TODO add laser ID mappings for RS32, RSBPEARL and RSBPEARL_MINI ?
  const std::array<uint16_t, 16> LASER_ID_MAPPING_RS16 = {0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8};
}

RobosenseToLidarNode::RobosenseToLidarNode(std::string node_name, const rclcpp::NodeOptions options)
  : Node(node_name, options)
{
  // Init ROS publisher
  this->Talker = this->create_publisher<Pcl2_msg>("lidar_points", 1);

  // Init ROS subscriber
  this->Listener = this->create_subscription<Pcl2_msg>("rslidar_points", 1,
                              std::bind(&RobosenseToLidarNode::Callback, this, std::placeholders::_1));

  RCLCPP_INFO_STREAM(this->get_logger(), BOLD_GREEN("RSLidar data converter is ready !"));
}

//------------------------------------------------------------------------------
void RobosenseToLidarNode::Callback(const Pcl2_msg& msg_received)
{
  //convertion to CloudRS
  CloudRS cloudRS = Utils::InitCloudRaw<CloudRS>(msg_received);

  // If input cloud is empty, ignore it
  if (cloudRS.empty())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Input RSLidar pointcloud is empty : frame ignored.");
    return;
  }

  // Fill the map of device_id if the device hasn't already been attributed one
  if (this->DeviceIdMap.count(cloudRS.header.frame_id) == 0)
    this->DeviceIdMap[cloudRS.header.frame_id] = this->DeviceIdMap.size();

  // We compute RPM : to do so, we need to ignore the first frame of the LiDAR, but it doesn't really matter as it is just 100ms ignored
  double currentTimeStamp = cloudRS.header.stamp;
  this->Rpm = Utils::EstimateRpm(currentTimeStamp, this->PreviousTimeStamp, this->Rpm, this->PossibleFrequencies);
  // We ignore first frame because it has no RPM
  if (this->Rpm < 0.)
    return;

  // Init SLAM pointcloud
  CloudS cloudS = Utils::InitCloudS<CloudRS>(cloudRS);

  const unsigned int nLasers = cloudRS.height;

  // Init of parameters useful for laser_id and time estimations
  if (this->InitEstimParamToDo)
  {
    Utils::InitEstimationParameters<PointRS>(cloudRS, nLasers, this->Clusters, this->ClockwiseRotationBool);
    this->InitEstimParamToDo = false;
  }
  double rotationTime = 1. / (this->Rpm * 60.);
  Eigen::Vector2d firstPoint = {cloudRS[0].x, cloudRS[0].y};

  // Build SLAM pointcloud
  for (unsigned int i = 0; i < cloudRS.size(); ++i)
  {
    const PointRS& rsPoint = cloudRS[i];

    // Remove no return points by checking unvalid values (NaNs or zeros)
    if (!Utils::IsPointValid(rsPoint))
      continue;

    // In case of dual returns mode, check that the second return is not identical to the first
    // CHECK this operation for other sensors than RS16
    if (!cloudS.empty() && std::equal(rsPoint.data, rsPoint.data + 3, cloudS.back().data))
      continue;

    // Copy coordinates and intensity
    PointS slamPoint;
    slamPoint.x = rsPoint.x;
    slamPoint.y = rsPoint.y;
    slamPoint.z = rsPoint.z;
    slamPoint.intensity = rsPoint.intensity;
    slamPoint.device_id = this->DeviceIdMap[cloudRS.header.frame_id];
    slamPoint.laser_id = Utils::ComputeLaserId({slamPoint.x, slamPoint.y, slamPoint.z}, nLasers, this->Clusters);
    slamPoint.time = Utils::EstimateTime({slamPoint.x, slamPoint.y}, rotationTime, firstPoint, this->ClockwiseRotationBool);

    cloudS.push_back(slamPoint);
  }

  // Publish pointcloud only if non empty
  if (!cloudS.empty())
  {
    //convertion to msg
    Pcl2_msg msg_sended;
    pcl::toROSMsg(cloudS, msg_sended);

    this->Talker->publish(msg_sended);
  }
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

  std::shared_ptr<lidar_conversions::RobosenseToLidarNode> rs2s
    = std::make_shared<lidar_conversions::RobosenseToLidarNode>("rslidar_conversion", options);

  rclcpp::spin(rs2s);

  return 0;
}
