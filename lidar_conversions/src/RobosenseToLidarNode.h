//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
//         Cadart Nicolas (Kitware SAS)
// Creation date: 2020-12-22
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
// #include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <LidarSlam/LidarPoint.h>

namespace lidar_conversions
{

/**
 * @class RobosenseToLidarNode aims at converting pointclouds published by RSLidar
 * ROS driver to the expected SLAM pointcloud format.
 *
 * The ROS RSLidar driver can be found here :
 * https://github.com/RoboSense-LiDAR/ros_rslidar
 */
class RobosenseToLidarNode: public rclcpp::Node
{
public:
  using PointRS = pcl::PointXYZI;
  using CloudRS = pcl::PointCloud<PointRS>;  ///< Pointcloud published by rslidar driver
  using PointS = LidarSlam::LidarPoint;
  using CloudS = pcl::PointCloud<PointS>;  ///< Pointcloud needed by SLAM
  using Pcl2_msg = sensor_msgs::msg::PointCloud2;

  //----------------------------------------------------------------------------
  /*!
   * @brief Constructor.
   * @param node_name Name of the node created
   */
  RobosenseToLidarNode(std::string node_name);

  //----------------------------------------------------------------------------
  /*!
   * @brief New lidar frame callback, converting and publishing RSLidar PointCloud as SLAM LidarPoint.
   * @param cloud New Lidar Frame, published by rslidar_pointcloud/cloud_node.
   */
  void Callback(const Pcl2_msg& msg_received);

private:

  //----------------------------------------------------------------------------

  // ROS node handles, subscriber and publisher
  // ros::NodeHandle &Nh, &PrivNh;
  rclcpp::Subscription<Pcl2_msg>::SharedPtr Listener;
  rclcpp::Publisher<Pcl2_msg>::SharedPtr Talker;

  // Optional mapping used to correct the numeric identifier of the laser ring that shot each point.
  // SLAM expects that the lowest/bottom laser ring is 0, and is increasing upward.
  // If unset, the following mappings will be used :
  // - if input cloud has 16 rings : RS16 mapping [0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8]
  // - otherwise : identity mapping (no laser_id change)
  std::vector<int> LaserIdMapping;

  int DeviceId = 0;  ///< LiDAR device identifier to set for each point.

  // Useful variables for approximate point-wise timestamps computation
  // These parameters should be set to the same values as ROS RSLidar driver's.
  // NOTE: to be precise, this timestamp estimation requires that each input
  // scan is an entire scan covering excatly 360°.
  double Rpm = 600;  ///< Spinning speed of sensor [rpm]. The duration of each input scan will be 60 / Rpm seconds.
};

}  // end of namespace lidar_conversions
