//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
// Creation date: 2023-12-12
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

#pragma once

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <hesai_point.h>
#include <LidarSlam/LidarPoint.h>

namespace lidar_conversions
{

/**
 * @class HesaiToLidarNode aims at converting pointclouds published by ROS
 * Hesai driver to the expected SLAM pointcloud format.
 *
 * The ROS Hesai driver can be found here :
 * https://github.com/HesaiTechnology/HesaiLidar_General_SDK
 */
class HesaiToLidarNode
{
public:
  using PointH = hesai_pcl::PointXYZIT;
  using CloudH = pcl::PointCloud<PointH>;  ///< Pointcloud published by hesai driver
  using PointS = LidarSlam::LidarPoint;
  using CloudS = pcl::PointCloud<PointS>;  ///< Pointcloud needed by SLAM

  //----------------------------------------------------------------------------
  /*!
   * @brief Constructor.
   * @param nh      Public ROS node handle, used to init publisher/subscriber.
   * @param priv_nh Private ROS node handle, used to access parameters.
   */
  HesaiToLidarNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

  //----------------------------------------------------------------------------
  /*!
   * @brief New lidar frame callback, converting and publishing Hesai PointCloud as SLAM LidarPoint.
   */
  void Callback(const CloudH& cloud);

private:

  //----------------------------------------------------------------------------

  // ROS node handles, subscriber and publisher
  ros::NodeHandle &Nh, &PrivNh;
  ros::Subscriber Listener;
  ros::Publisher Talker;

  // Optional mapping used to correct the numeric identifier of the laser ring that shot each point.
  // SLAM expects that the lowest/bottom laser ring is 0, and is increasing upward.
  // If unset, identity mapping (no laser_id change) will be used.
  // NOTE: the Hesai ROS driver should already correctly modify the laser_id,
  // so this shouldn't be needed.
  std::vector<int> LaserIdMapping;

  // Useful variables for approximate point-wise timestamps computation
  // These parameters should be set to the same values as ROS Hesai driver's.
  double Rpm = 600;  ///< Spinning speed of sensor [rpm]
  bool TimestampFirstPacket = false;  ///< Wether timestamping is based on the first or last packet of each scan
};

}  // end of namespace lidar_conversions
