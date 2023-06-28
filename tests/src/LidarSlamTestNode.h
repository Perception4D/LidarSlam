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

#define _USE_MATH_DEFINES
#include <cmath> 

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <lidar_slam/msg/confidence.hpp>
#include <Eigen/Geometry>

namespace Eigen
{
  using Vector6d = Matrix<double, 6, 1>;
}

namespace lidar_slam_test
{

struct Evaluator
{
  double Stamp = 0.;
  float Overlap = 0.f;
  float NbMatches = 0;
  float Duration = 0.f;
};

struct Pose
{
  double Stamp = 0.;
  Eigen::Vector6d data = Eigen::Vector6d::Zero();
};

/**
 * @class VelodyneToLidarNode aims at converting pointclouds published by ROS
 * Velodyne driver to the expected SLAM pointcloud format.
 *
 * The ROS Velodyne driver can be found here :
 * https://github.com/ros-drivers/velodyne
 */
class LidarSlamTestNode : public rclcpp::Node
{

public:
  //----------------------------------------------------------------------------
  /*!
   * @brief Constructor.
   * @param[in] name_node Name of the node, used to init publisher/subscribers and log messages
   * @param[in] options Options of the node, default no options
   */
  LidarSlamTestNode(std::string name_node = "lidar_slam_test",
                const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  //----------------------------------------------------------------------------
  /*!
   * @brief Check if comparison with reference data can be performed
   */
  bool CanBeCompared();
  //----------------------------------------------------------------------------
  /*!
   * @brief pose call back, log the data and compare it with reference if required
   * @param pose outputed by lidar slam node
   */
  void PoseCallback(const nav_msgs::msg::Odometry& poseMsg);

  //----------------------------------------------------------------------------
  /*!
   * @brief confidence estimators call back, log the data and compare it with reference if required
   * @param confidence message defined in lidar slam package
   */
  void ConfidenceCallback(const lidar_slam::msg::Confidence& confidence);

private:

  //----------------------------------------------------------------------------

  // ROS subscriber and publisher
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr PoseListener;
  rclcpp::Subscription<lidar_slam::msg::Confidence>::SharedPtr ConfidenceListener;

  bool Verbose = false;

  // Main boolean to define the success or the failure of the test
  bool Failure = false;

  Eigen::Isometry3d PrevTransform = Eigen::Isometry3d::Identity();
  float LastAngleDiff = 0.f;
  float LastPositionDiff = 0.f;

  // Path to the folder where to store the results (folder must exist)
  std::string ResPath;

  // Reference data for comparison :

  // Path to the folder containing the reference results to compare with
  // If empty, no comparison is performed
  std::string RefPath;

  // Storage for reference data (loaded from RefPath)
  std::vector<Evaluator> RefEvaluators;
  std::vector<Pose> RefPoses;

  // Storage for current results :

  // Counter to keep track of the pose index to compare with reference
  unsigned int PoseCounter = 0;
  float DiffAngle = 0.f;
  float DiffPosition = 0.f;

  // Counter to keep track of the confidence index to compare with reference
  unsigned int ConfidenceCounter = 0;
  float DiffOverlap = 0.f;
  float DiffTime = 0.f;
  float DiffNbMatches = 0.f;

  // Thresholds to warn the user :
  float PositionThreshold = 0.2f;  // 20cm
  float AngleThreshold = 5.f;       // 5°
  float TimeThreshold = 0.01f;     // 10ms

private:

  void LoadRef();
  void OutputTestResult();
};

}  // end of namespace lidar_conversions
