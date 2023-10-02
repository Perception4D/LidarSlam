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

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_conversions
{

VelodyneToLidarNode::VelodyneToLidarNode(std::string node_name, const rclcpp::NodeOptions options)
  : rclcpp::Node(node_name, options)
{
  // Get number of lasers
  this->get_parameter("nb_lasers", this->NbLasers);

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
  CloudV cloudV = Utils::InitCloudRaw<CloudV>(msg_received);

  // If input cloud is empty, ignore it
  if (cloudV.empty())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Input Velodyne pointcloud is empty : frame ignored.");
    return;
  }

  // We compute the rotation duration : to do so, we need to ignore the first frame of the LiDAR,
  // but it doesn't really matter as it is just 100ms ignored
  double currentTimeStamp = cloudV.header.stamp;
  this->RotationDuration = Utils::EstimateFrameTime(currentTimeStamp, this->PreviousTimeStamp, this->RotationDuration, this->PossibleFrequencies);
  // We now ignore first frame because it has no rotation duration
  if (this->RotationDuration < 0.)
    return;

  // Init SLAM pointcloud
  CloudS cloudS = Utils::InitCloudS<CloudV>(cloudV);

  const double nLasers = (cloudV.height >= 8 && cloudV.height <=128) ? static_cast<double>(cloudV.height) : this->NbLasers;

  // Init of parameters useful for laser_id and time estimations
  if (this->InitEstimParamToDo)
  {
    Utils::InitEstimationParameters<PointV>(cloudV, nLasers, this->Clusters, this->ClockwiseRotationBool);
    this->InitEstimParamToDo = false;
  }
  Eigen::Vector2d firstPoint = {cloudV[0].x, cloudV[0].y};

  // Check if time field looks properly set
  bool isTimeValid = cloudV.back().time - cloudV.front().time > 1e-8;
  if (!isTimeValid)
    RCLCPP_WARN_STREAM(this->get_logger(), "Invalid 'time' field, it will be built from azimuth advancement.");

  // Build SLAM pointcloud
  for (const PointV& velodynePoint : cloudV)
  {
    // Remove no return points by checking unvalid values (NaNs or zeros)
    if (!Utils::IsPointValid(velodynePoint))
      continue;

    PointS slamPoint;
    slamPoint.x = velodynePoint.x;
    slamPoint.y = velodynePoint.y;
    slamPoint.z = velodynePoint.z;
    slamPoint.intensity = velodynePoint.intensity;
    slamPoint.device_id = this->DeviceIdMap[cloudV.header.frame_id];
    slamPoint.laser_id = velodynePoint.ring;

    // Use time field if available, else estimate it from azimuth advancement
    if (isTimeValid)
      slamPoint.time = velodynePoint.time;
    else
      slamPoint.time = Utils::EstimateTime({slamPoint.x, slamPoint.y}, this->RotationDuration, firstPoint, this->ClockwiseRotationBool);

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
