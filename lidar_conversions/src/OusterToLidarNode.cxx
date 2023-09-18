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
#include "rmw/qos_profiles.h"

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_conversions
{

OusterToLidarNode::OusterToLidarNode(std::string node_name, const rclcpp::NodeOptions options)
  : Node(node_name, options)
{
  // Get laser ID mapping
  this->get_parameter("laser_id_mapping", this->LaserIdMapping);

  // Get number of lasers
  this->get_parameter("nb_lasers", this->NbLasers);

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
  CloudO cloudO = Utils::InitCloudRaw<CloudO>(msg_received);
  // If input cloud is empty, ignore it
  if (cloudO.empty())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Input Ouster pointcloud is empty : frame ignored.");
    return;
  }

  // Fill the map of device_id if the device hasn't already been attributed one
  if (this->DeviceIdMap.count(cloudO.header.frame_id) == 0)
    this->DeviceIdMap[cloudO.header.frame_id] = this->DeviceIdMap.size();

  // We compute RPM : to do so, we need to ignore the first frame of the LiDAR, but it doesn't really matter as it is just 100ms ignored
  // However, we chose to ignore the first frame after having initialized estimation parameters (see below)
  double currentTimeStamp = cloudO.header.stamp;
  this->Rpm = Utils::EstimateRpm(currentTimeStamp, this->PreviousTimeStamp, this->Rpm, this->PossibleFrequencies);
  // We now ignore first frame because it has no RPM
  if (this->Rpm < 0.)
    return;

  // Init SLAM pointcloud
  CloudS cloudS = Utils::InitCloudS<CloudO>(cloudO);

  // Check wether to use custom laser ID mapping or leave it untouched
  bool useLaserIdMapping = !this->LaserIdMapping.empty();

  // Init of parameters useful for laser_id and time estimations
  if (this->InitEstimParamToDo)
  {
    Utils::InitEstimationParameters<PointO>(cloudO, this->NbLasers, this->Clusters, this->ClockwiseRotationBool);
    this->InitEstimParamToDo = false;
  }
  double rotationTime = 1. / (this->Rpm * 60.);
  Eigen::Vector2d firstPoint = {cloudO[0].x, cloudO[0].y};

  // Check if time field looks properly set
  bool isTimeValid = cloudO.back().t - cloudO.front().t > 1e-8;
  if (!isTimeValid)
    RCLCPP_WARN_STREAM(this->get_logger(), "Invalid 'time' field, it will be built from azimuth advancement.");

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
    slamPoint.device_id = this->DeviceIdMap[cloudO.header.frame_id];

    // Use time field if available, else estimate it from azimuth advancement
    if (isTimeValid)
      slamPoint.time = ousterPoint.t;
    else
      slamPoint.time = Utils::EstimateTime({slamPoint.x, slamPoint.y}, rotationTime, firstPoint, this->ClockwiseRotationBool);

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
