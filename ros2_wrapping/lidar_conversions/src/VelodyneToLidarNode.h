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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_point.h>
#include <LidarSlam/LidarPoint.h>
#include "Utilities.h"

namespace lidar_conversions
{

/**
 * @class VelodyneToLidarNode aims at converting pointclouds published by ROS
 * Velodyne driver to the expected SLAM pointcloud format.
 *
 * The ROS Velodyne driver can be found here :
 * https://github.com/ros-drivers/velodyne
 */
class VelodyneToLidarNode : public rclcpp::Node
{
public:
  using PointV = velodyne_pcl::PointXYZIRT;
  using CloudV = pcl::PointCloud<PointV>;  ///< Pointcloud published by velodyne driver
  using PointS = LidarSlam::LidarPoint;
  using CloudS = pcl::PointCloud<PointS>;  ///< Pointcloud needed by SLAM
  using Pcl2_msg = sensor_msgs::msg::PointCloud2;

  //----------------------------------------------------------------------------
  /*!
   * @brief Constructor.
   * @param[in] name_node Name of the node used to init publisher/subscribers and log messages
   * @param[in] options Options of the node, default no options
   */
  VelodyneToLidarNode(std::string node_name = "velodyne_conversion",
                      const rclcpp::NodeOptions options = rclcpp::NodeOptions());

  //----------------------------------------------------------------------------
  /*!
   * @brief New lidar frame callback, converting and publishing Velodyne PointCloud as SLAM LidarPoint.
   * @param msg_received New Lidar Frame from a Velodyne lidar
   */
  void Callback(const Pcl2_msg& msg_received);

private:

  //----------------------------------------------------------------------------

  // ROS node handles, subscriber and publisher
  rclcpp::Subscription<Pcl2_msg>::SharedPtr Listener;
  rclcpp::Publisher<Pcl2_msg>::SharedPtr Talker;

  // Map to store the device id of each device (in case of multilidar).
  std::map<std::string, int> DeviceIdMap;

  double NbLasers = 16.; ///< Number of lasers of the LiDAR. Optional as it can be taken from header attribute .height of the PointCloud.
  bool InitEstimParamToDo = true; ///< Flag to initialize the parameters useful for laser_id and time estimations.
  bool ClockwiseRotationBool;  ///< True if the LiDAR rotates clockwise, false otherwise.

  // Useful variable to estimate RPM (itself used to estimate time)
  // NOTE: to be precise, this RPM estimation requires that each input
  // scan is an entire scan covering excatly 360°
  double Rpm = -1.;
  double PreviousTimeStamp = -1.;
  const std::vector<double> PossibleFrequencies = {5., 6., 7., 8., 9., 10., 11., 12., 13., 14., 15., 16., 17., 18., 19., 20.}; ///< Vector of all the possible frequencies for Velodyne LiDAR

  // Useful variable to estimate laser_id
  std::vector<Utils::Cluster> Clusters;
};

}  // end of namespace lidar_conversions
