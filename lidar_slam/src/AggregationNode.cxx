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

#include "AggregationNode.h"

#include <LidarSlam/Utilities.h>
#include <LidarSlam/PointCloudStorage.h>

#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <boost/filesystem.hpp>

//==============================================================================
//   Basic SLAM use
//==============================================================================

//------------------------------------------------------------------------------
AggregationNode::AggregationNode(std::string name_node, const rclcpp::NodeOptions& options)
  : rclcpp::Node(name_node, options)
{
  // ***************************************************************************
  // Init ROS publisher
  // aggregated points with specified density
  this->PointsPublisher = this->create_publisher<Pcl2_msg>("aggregated_cloud", 10);

  // Init ROS subscriber
  // Lidar frame undistorted
  this->FrameSubscriber = this->create_subscription<Pcl2_msg>(
            "slam_registered_points", 1, std::bind(&AggregationNode::Callback, this, std::placeholders::_1));

  // Init service
  this->SaveService = this->create_service<lidar_slam::srv::SavePc>(
      "lidar_slam/save_pc",
      std::bind(&AggregationNode::SavePointcloudService, this, std::placeholders::_1, std::placeholders::_2));

  // Init rolling grid with parameters
  this->DenseMap = std::make_shared<LidarSlam::RollingGrid>();
  // Voxel size
  float leafSize;
  this->get_parameter_or<float>("leaf_size", leafSize, 0.1);
  this->DenseMap->SetLeafSize(leafSize);
  // Maximum size in voxels -> The second dimension of
  // the rolling grid is not needed in this context
  this->DenseMap->SetGridSize(3);
  float maxSize;
  this->get_parameter_or<float>("max_size", maxSize, 200.);
  maxSize /= 3.;
  this->DenseMap->SetVoxelResolution(maxSize);
  // Min number of frames seeing a voxel to extract it
  int minNbPointsPerVoxel;
  this->get_parameter_or<int>("min_points_per_voxel", minNbPointsPerVoxel, 2);
  this->DenseMap->SetMinFramesPerVoxel(minNbPointsPerVoxel);

  RCLCPP_INFO_STREAM(this->get_logger(), "Aggregation node is ready !");
}

//------------------------------------------------------------------------------
void AggregationNode::Callback(const Pcl2_msg& registeredCloudMsg)
{
  //Convert msg to PointCloud
  CloudS::Ptr registeredCloud = std::make_shared<CloudS>();
  pcl::fromROSMsg(registeredCloudMsg, *registeredCloud);

  // Aggregated points from all frames
  this->DenseMap->Add(registeredCloud, false);
  this->Pointcloud = this->DenseMap->Get(true);
  this->Pointcloud->header = registeredCloud->header;

  //convert aggregatedCloud to message
  Pcl2_msg aggregatedCloudMsg;
  pcl::toROSMsg(*this->Pointcloud, aggregatedCloudMsg);

  // Publish aggregatedCloud
  this->PointsPublisher->publish(aggregatedCloudMsg);
}

//------------------------------------------------------------------------------
void AggregationNode::SavePointcloudService(
    const std::shared_ptr<lidar_slam::srv::SavePc::Request> req,
    const std::shared_ptr<lidar_slam::srv::SavePc::Response> res)
{
  std::string outputPrefix = req->output_prefix_path.empty() ? std::getenv("HOME") : req->output_prefix_path;
  boost::filesystem::path outputPrefixPath(outputPrefix);
  if (!boost::filesystem::exists(outputPrefixPath.parent_path()))
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Output folder does not exist, saving to home folder :" << std::getenv("HOME"));
    outputPrefixPath = boost::filesystem::path(std::getenv("HOME")) / boost::filesystem::path(outputPrefixPath.stem());
  }

  if (req->format > 2 || req->format < 0)
    req->format = 0;
  LidarSlam::PCDFormat f = static_cast<LidarSlam::PCDFormat>(req->format);

  std::string outputFilePath = outputPrefixPath.string() + "_" + std::to_string(int(this->now().seconds())) + ".pcd";
  LidarSlam::savePointCloudToPCD<PointS>(outputFilePath, *this->Pointcloud, f);
  RCLCPP_INFO_STREAM(this->get_logger(), "Pointcloud saved to " << outputFilePath);
  res->success = true;
}

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create options for the node to use undeclared parameters
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.allow_undeclared_parameters(true);

  // Create lidar slam node, which subscribes to pointclouds coming from conversion node
  // and to external sensor messages in parallel.
  std::shared_ptr<AggregationNode> aggregationNodePtr =
            std::make_shared<AggregationNode>("aggregation", options);

  // Handle callbacks until shut down
  rclcpp::spin(aggregationNodePtr);

  return 0;
}
