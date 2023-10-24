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

#ifndef AGGREGATION_NODE_H
#define AGGREGATION_NODE_H

// ROS
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>

// Service
#include <lidar_slam/srv/save_pc.hpp>
#include <lidar_slam/srv/reset.hpp>

// LidarSlam
#include <LidarSlam/LidarPoint.h>
#include <LidarSlam/RollingGrid.h>

class AggregationNode : public rclcpp::Node
{
public:

  using PointS = LidarSlam::LidarPoint;
  using CloudS = pcl::PointCloud<PointS>;  ///< Pointcloud needed by SLAM
  using Pcl2_msg = sensor_msgs::msg::PointCloud2; //messages for Pointcloud

  //----------------------------------------------------------------------------
  /*!
   * @brief     Constructor.
   * @param[in] name_node Name of the node
   * @param[in] options Options of the node
   */
  AggregationNode(std::string name_node = "aggregation",
                const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  //----------------------------------------------------------------------------
  /*!
   * @brief     New main frame callback, aggregating frames
   * @param[in] registeredCloudMsg New registered frame message published by the LidarSlamNode
   *
   * Input pointcloud must have following fields :
   *  - x, y, z (float): point coordinates
   *  - time (double): time offset to add to the pointcloud header timestamp to
   *    get approximate point-wise acquisition timestamp
   *  - intensity (float): intensity/reflectivity of the point
   *  - laser_id (uint16): numeric identifier of the laser ring that shot this point.
   *    The lowest/bottom laser ring should be 0, and it should increase upward.
   *  - device_id (uint8): numeric identifier of the LiDAR device/sensor.
   *    This id should be the same for all points of the cloud acquired by the same sensor.
   *  - label (uint8): optional input, not yet used.
   */
  void Callback(const Pcl2_msg& registeredCloudMsg);

  void SavePointcloudService(
    const std::shared_ptr<lidar_slam::srv::SavePc::Request> req,
    const std::shared_ptr<lidar_slam::srv::SavePc::Response> res);

  void ResetService(
    const std::shared_ptr<lidar_slam::srv::Reset::Request> req,
    const std::shared_ptr<lidar_slam::srv::Reset::Response> res);


private:

  // ROS subscribers, publishers and services
  rclcpp::Subscription<Pcl2_msg>::SharedPtr FrameSubscriber;
  rclcpp::Publisher<Pcl2_msg>::SharedPtr PointsPublisher;
  rclcpp::Service<lidar_slam::srv::SavePc>::SharedPtr SaveService;
  rclcpp::Service<lidar_slam::srv::Reset>::SharedPtr RstService;

  // Dense map containing aggregated points from all frames
  std::shared_ptr<LidarSlam::RollingGrid> DenseMap;
  CloudS::Ptr Pointcloud;
};

#endif // AGGREGATION_NODE_H
