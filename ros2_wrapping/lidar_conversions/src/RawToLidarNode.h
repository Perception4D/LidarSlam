//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Faure Jeanne (Kitware SAS)
// Creation date: 2023-08-31
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
#include <pcl/point_types.h>
#include <LidarSlam/LidarPoint.h>
#include "Utilities.h"
#include <unordered_map>
#include <random>

namespace lidar_conversions
{

/**
 * @class RawToLidarNode aims at converting pointclouds published by RSLidar
 * ROS driver to the expected SLAM pointcloud format.
 *
 * The ROS RSLidar driver can be found here :
 * https://github.com/RoboSense-LiDAR/ros_rslidar
 */
class RawToLidarNode : public rclcpp::Node
{
public:
  using PointXYZ = pcl::PointXYZ;
  using PointXYZI = pcl::PointXYZI;

  using CloudXYZ = pcl::PointCloud<PointXYZ>;  ///< Pointcloud published by lidar driver
  using CloudXYZI = pcl::PointCloud<PointXYZI>;

  using PointS = LidarSlam::LidarPoint;
  using CloudS = pcl::PointCloud<PointS>;  ///< Pointcloud needed by SLAM
  using Pcl2_msg = sensor_msgs::msg::PointCloud2;

  //----------------------------------------------------------------------------
  /*!
   * @brief Constructor.
   * @param node_name name of the node created
   * @param[in] options Options of the node, default no options
   */
  RawToLidarNode(std::string node_name,
                rclcpp::NodeOptions options = rclcpp::NodeOptions());

  //------------------------------------------------------------------------------
  /*!
   * @brief Publish the message with the PointCloud
   * @param cloudS PointCloud to convert
   */
  inline void PublishMsg(CloudS cloudS)
  {
    // Conversion to msg
    // Publish message only if cloud non empty
    if (!cloudS.empty())
    {
      Pcl2_msg msg_sended;
      pcl::toROSMsg(cloudS, msg_sended);
      this->Talker->publish(msg_sended);
    }
  }

 //----------------------------------------------------------------------------
  /*!
   * @brief New lidar frame callback, converting and publishing PointCloud of Points with only x, y, z as SLAM PointCloud.
   * @param msg_received New Lidar Frame, published by lidar_pointcloud/cloud_node.
   */
  void CallbackXYZ(const Pcl2_msg& msg_received);

  //----------------------------------------------------------------------------
  /*!
   * @brief New lidar frame callback, converting and publishing PointCloud with only x, y, z, intensity as SLAM PointCloud.
   * @param msg_received New Lidar Frame, published by lidar_pointcloud/cloud_node.
   */
  void CallbackXYZI(const Pcl2_msg& msg_received);

private:
  //----------------------------------------------------------------------------

  // ROS node handles, subscriber and publisher
  rclcpp::Subscription<Pcl2_msg>::SharedPtr ListenerXYZ;
  rclcpp::Subscription<Pcl2_msg>::SharedPtr ListenerXYZI;
  rclcpp::Publisher<Pcl2_msg>::SharedPtr Talker;

  // Map to store the device id of each device (in case of multilidar).
  std::map<std::string, int> DeviceIdMap;

  // Number of lasers of the LiDAR.
  double NbLasers = 16.;

  // Useful variable to estimate RPM
  // NOTE: to be precise, this RPM estimation requires that each input
  // scan is an entire scan covering excatly 360°
  double Rpm = -1.;
  double PreviousTimeStamp = -1.;
  const std::vector<double> PossibleFrequencies; ///< Vector of all the possible frequencies of a certain type of LiDAR
};

}  // end of namespace lidar_conversions
