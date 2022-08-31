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

#include <rclcpp/rclcpp.hpp>
// #include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ouster_point.h>
#include <LidarSlam/LidarPoint.h>

namespace lidar_conversions
{

/**
 * @class OusterToLidarNode aims at converting pointclouds published by ROS
 * Velodyne driver to the expected SLAM pointcloud format.
 *
 * The ROS Velodyne driver can be found here :
 * https://github.com/ros-drivers/velodyne
 */
class OusterToLidarNode : public rclcpp::Node
{
public:
  using PointO = ouster_ros::Point;
  using CloudV = pcl::PointCloud<PointO>;  ///< Pointcloud published by ouster driver
  using PointS = LidarSlam::LidarPoint;
  using CloudS = pcl::PointCloud<PointS>;  ///< Pointcloud needed by SLAM
  using Pcl2_msg = sensor_msgs::msg::PointCloud2;

  //----------------------------------------------------------------------------
  /*!
   * @brief Constructor.
   * @param node_name name of the node created
   */
  OusterToLidarNode(std::string node_name);

  //----------------------------------------------------------------------------
  /*!
   * @brief New lidar frame callback, converting and publishing Velodyne PointCloud as SLAM LidarPoint.
   * @param cloud New Lidar Frame, published by velodyne_pointcloud/transform_node.
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
  // If unset, identity mapping (no laser_id change) will be used.
  // NOTE: the Velodyne ROS driver should already correctly modify the laser_id,
  // so this shouldn't be needed.
  std::vector<int> LaserIdMapping;

  int DeviceId = 0;  ///< LiDAR device identifier to set for each point.

  // Useful variables for approximate point-wise timestamps computation
  // These parameters should be set to the same values as ROS Velodyne driver's.
  double Rpm = 600;  ///< Spinning speed of sensor [rpm]
  bool TimestampFirstPacket = false;  ///< Wether timestamping is based on the first or last packet of each scan
};

}  // end of namespace lidar_conversions
