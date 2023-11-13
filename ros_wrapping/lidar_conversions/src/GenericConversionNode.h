//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
//         Fu Tong (Kitware SAS)
// Creation date: 2023-11-03
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
#include <pcl/point_types.h>
#include <LidarSlam/LidarPoint.h>

#include "Utilities.h"

namespace lidar_conversions
{

/**
 * @class GenericConversionNode aims at
 * converting pointclouds published by any ROS driver
 * to the expected SLAM pointcloud format.
 */
class GenericConversionNode
{
public:
  using PointXYZ = pcl::PointXYZ;
  using CloudXYZ = pcl::PointCloud<PointXYZ>;  ///< Pointcloud published by a lidar driver
  using PointS   = LidarSlam::LidarPoint;
  using CloudS   = pcl::PointCloud<PointS>;    ///< Pointcloud needed by SLAM

  //----------------------------------------------------------------------------
  /*!
   * @brief Constructor.
   * @param nh      Public ROS node handle, used to init publisher/subscriber.
   * @param priv_nh Private ROS node handle, used to access parameters.
   */
  GenericConversionNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

  //----------------------------------------------------------------------------
  /*!
   * @brief New lidar frame callback, converting and publishing Lidar PointCloud as SLAM LidarPoint.
   * @param cloud New Lidar Frame, published by lidar_pointcloud/cloud_node.
   */
  void Callback(const sensor_msgs::PointCloud2& msg_received);

private:

  //----------------------------------------------------------------------------

  // ROS node handles, subscriber and publisher
  ros::NodeHandle &Nh, &PrivNh;
  ros::Subscriber Listener;
  ros::Publisher Talker;

  // Number of lasers of the LiDAR.
  int NbLasers = 16.;

  // Useful variable to estimate rotation duration (itself used to estimate time)
  // NOTE: to be precise, this rotation duration estimation requires that each input
  // scan is an entire scan covering excatly 360°
  double RotationDuration = -1.;
  double RotationDurationPrior = -1.;
  double PrevFrameTime = -1.;
  std::vector<double> PossibleFrequencies; ///< Vector of all the possible frequencies of a certain type of LiDAR

  // Useful variable to estimate laser_id
  std::vector<Utils::Cluster> Clusters;

  // Number of threads to use for the conversion
  int NbThreads = 1;
};

}  // end of namespace lidar_conversions
