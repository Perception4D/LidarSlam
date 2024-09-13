//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
// Creation date: 2020-12-10
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

#include "LivoxToLidarNode.h"
#include "Utilities.h"
#include <pcl_conversions/pcl_conversions.h>

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_conversions
{

LivoxToLidarNode::LivoxToLidarNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
  : Nh(nh)
  , PrivNh(priv_nh)
{
  // Init ROS publisher
  this->Talker = nh.advertise<CloudS>("lidar_points", 1);

  // Get LiDAR id
  this->PrivNh.param("pointcloud2", this->IsPcl2, this->IsPcl2);

  // Init ROS subscriber
  if (this->IsPcl2)
    this->Listener = nh.subscribe("livox/lidar", 1, &LivoxToLidarNode::PointCloud2Callback, this);
  else
    this->Listener = nh.subscribe("livox/lidar", 1, &LivoxToLidarNode::LivoxCustomMsgCallback, this);

  ROS_INFO_STREAM(BOLD_GREEN("Livox data converter is ready !"));
}

//------------------------------------------------------------------------------
void LivoxToLidarNode::PointCloud2Callback(const CloudL& cloudL)
{
  // If input cloud is empty, ignore it
  if (cloudL.empty())
  {
    ROS_ERROR_STREAM("Input Livox pointcloud is empty : frame ignored.");
    return;
  }

  // Init SLAM pointcloud
  CloudS cloudS = Utils::InitCloudS<CloudL>(cloudL);

  // Build SLAM pointcloud
  double prevTime = -0.1;
  for (const PointL& livoxPoint : cloudL)
  {
    // Remove no return points by checking unvalid values (NaNs or zeros)
    if (!Utils::IsPointValid(livoxPoint))
      continue;

    PointS slamPoint;
    slamPoint.x = livoxPoint.x;
    slamPoint.y = livoxPoint.y;
    slamPoint.z = livoxPoint.z;
    slamPoint.intensity = livoxPoint.intensity;
    slamPoint.laser_id = 0;

    slamPoint.time = prevTime + 0.1/300000.; // Supposing 10 Hz and 300 000 points
    prevTime = slamPoint.time;

    if (!Utils::HasNanField(slamPoint))
      cloudS.push_back(slamPoint);
  }

  // Publish pointcloud only if non empty
  if (cloudS.empty())
  {
    ROS_ERROR_STREAM("Slam pointcloud is empty : frame ignored.");
    return;
  }
  this->Talker.publish(cloudS);
}

//------------------------------------------------------------------------------
void LivoxToLidarNode::LivoxCustomMsgCallback(const CustomMsg& cloudLmsg)
{
  // Init SLAM pointcloud
  CloudS cloudS;
  cloudS.reserve(cloudLmsg.point_num);
  cloudS.header.stamp = cloudLmsg.timebase * 1e-3; // microseconds
  cloudS.header.frame_id = cloudLmsg.header.frame_id;
  cloudS.header.seq = cloudLmsg.header.seq;

  // Build SLAM pointcloud
  for (int i = 0; i < cloudLmsg.point_num; ++i)
  {
    const LivoxCustomPoint& livoxPoint = cloudLmsg.points[i];

    if (!Utils::IsPointValid(pcl::PointXYZ(livoxPoint.x, livoxPoint.y, livoxPoint.z)))
      continue;

    PointS slamPoint;
    slamPoint.x = livoxPoint.x;
    slamPoint.y = livoxPoint.y;
    slamPoint.z = livoxPoint.z;
    slamPoint.intensity = livoxPoint.reflectivity;
    slamPoint.laser_id = livoxPoint.line;
    slamPoint.time = double(livoxPoint.offset_time) * 1e-9; // seconds

    if (!Utils::HasNanField(slamPoint))
      cloudS.push_back(slamPoint);
  }

  this->Talker.publish(cloudS);
}

}  // end of namespace lidar_conversions

//------------------------------------------------------------------------------
/*!
 * @brief Main node entry point.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "livox_conversion");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  lidar_conversions::LivoxToLidarNode v2s(n, priv_nh);

  ros::spin();

  return 0;
}
