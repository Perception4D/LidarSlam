//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Arthur Bourbousson (Kitware SAS)
// Creation date: 2019-10-24
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

#ifndef LIDAR_SLAM_NODE_H
#define LIDAR_SLAM_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lidar_slam_interfaces/msg/confidence.hpp>
#include <lidar_slam_interfaces/msg/slam_command.hpp>
#include <apriltag_ros/msg/april_tag_detection.hpp>
#include <apriltag_ros/msg/april_tag_detection_array.hpp>

// SLAM
#include <LidarSlam/Slam.h>


class LidarSlamNode
{
public:

  using PointS = LidarSlam::Slam::Point;
  using CloudS = pcl::PointCloud<PointS>;  ///< Pointcloud needed by SLAM

  //----------------------------------------------------------------------------
  /*!
   * @brief     Constructor.
   * @param[in] nh      Public ROS node handle, used to init publisher/subscribers.
   * @param[in] priv_nh Private ROS node handle, used to access parameters.
   */
  LidarSlamNode(rclcpp::Node::SharedPtr& nh, rclcpp::Node::SharedPtr& priv_nh);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Destructor.
   *
   * Used to shut down external spinners
   */
  ~LidarSlamNode();

};

#endif // LIDAR_SLAM_NODE_H