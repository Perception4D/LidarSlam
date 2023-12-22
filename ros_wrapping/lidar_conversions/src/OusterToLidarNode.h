//==============================================================================
// Copyright 2021-2022 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
// Creation date: 2022-05-10
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
#include <ouster_point.h>
#include <LidarSlam/LidarPoint.h>

#include "Utilities.h"
#include "lidar_conversions/EstimSense.h"
namespace lidar_conversions
{

/**
 * @class OusterToLidarNode aims at converting pointclouds published by ROS
 * Ouster driver to the expected SLAM pointcloud format.
 *
 * The ROS Ouster driver can be found here :
 * https://github.com/ouster-lidar/ouster-ros
 */
class OusterToLidarNode
{
public:
  using PointO = ouster_ros::Point;
  using CloudO = pcl::PointCloud<PointO>;  ///< Pointcloud published by ouster driver
  using PointS = LidarSlam::LidarPoint;
  using CloudS = pcl::PointCloud<PointS>;  ///< Pointcloud needed by SLAM

  //----------------------------------------------------------------------------
  /*!
   * @brief Constructor.
   * @param nh      Public ROS node handle, used to init publisher/subscriber.
   * @param priv_nh Private ROS node handle, used to access parameters.
   */
  OusterToLidarNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

  //----------------------------------------------------------------------------
  /*!
   * @brief New lidar frame callback, converting and publishing Ouster PointCloud as SLAM LidarPoint.
   */
  void Callback(const CloudO& cloud);

  //----------------------------------------------------------------------------
  /*!
   * @brief Service to re-compute the rotation sense of LiDAR.
   * @param request Service request
   * @param response Service response
   */
  bool EstimSenseService(lidar_conversions::EstimSenseRequest& req, lidar_conversions::EstimSenseResponse& res);

private:

  //----------------------------------------------------------------------------

  // ROS node handles, subscriber and publisher
  ros::NodeHandle &Nh, &PrivNh;
  ros::Subscriber Listener;
  ros::Publisher Talker;
  ros::ServiceServer EstimService;

  unsigned int NbLasers = 64; ///< Number of lasers of the LiDAR. Optional as it can be taken from header attribute .height of the PointCloud.
  bool RotationSenseEstimated = false; ///< Flag to initialize the parameters useful for time estimations.
  bool RotationIsClockwise;  ///< True if the LiDAR rotates clockwise, false otherwise.

  // Useful variable to estimate the rotation duration (itself used to estimate time)
  // NOTE: to be precise, this rotation duration estimation requires that each input
  // scan is an entire scan covering excatly 360°
  double RotationDuration = -1.;
  double RotationDurationPrior = -1.;
  double PrevFrameTime = -1.;
  std::vector<double> PossibleFrequencies = {10., 20.}; ///< Vector of all the possible frequencies for Ouster LiDAR

  // Number of threads to use for the conversion
  int NbThreads = 1;
};

}  // end of namespace lidar_conversions
