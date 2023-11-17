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

// LidarSlam
#include <LidarSlam/Utilities.h>
#include <LidarSlam/PointCloudStorage.h>

// ROS
#include <pcl_conversions/pcl_conversions.h>

// Boost
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
  // Optional publisher
  this->get_parameter_or<bool>("slice.enable", this->DoExtractSlice, false);
  if (this->DoExtractSlice)
  {
    this->SlicePublisher = this->create_publisher<Pcl2_msg>("slice_cloud", 10);
    this->SliceAreaPublisher = this->create_publisher<std_msgs::msg::Float64>("slice_area", 10);
  }

  // Init ROS subscribers
  // Lidar frame undistorted
  this->FrameSubscriber = this->create_subscription<Pcl2_msg>(
            "slam_registered_points", 1, std::bind(&AggregationNode::Callback, this, std::placeholders::_1));
  // Optional lidar SLAM pose
  if (this->DoExtractSlice)
    this->PoseSubscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "slam_odom", 1, std::bind(&AggregationNode::PoseCallback, this, std::placeholders::_1));

  // Init service
  this->SaveService = this->create_service<lidar_slam::srv::SavePc>(
      "lidar_slam/save_pc",
      std::bind(&AggregationNode::SavePointcloudService, this, std::placeholders::_1, std::placeholders::_2));

  // Init service
  this->RstService = this->create_service<lidar_slam::srv::Reset>(
      "lidar_slam/reset",
      std::bind(&AggregationNode::ResetService, this, std::placeholders::_1, std::placeholders::_2));


  // Init rolling grid with parameters
  this->DenseMap = std::make_shared<LidarSlam::RollingGrid>();

  // Slice parameters (needed to define the main voxel grid)
  this->get_parameter_or<double>("slice.traj_length", this->TrajectoryMaxLength, 1.);
  // Compute the width of the slice to extract
  this->get_parameter_or<double>("slice.width", this->SliceWidth, 0.2);
  // Set the max distance from the pose to compute a slice
  this->get_parameter_or<double>("slice.max_dist", this->SliceMaxDist, 5.);
  double angleStepDeg;
  this->get_parameter_or<double>("slice.angle_resolution", angleStepDeg, 3.);
  this->AngleStep = (M_PI / 180.) * angleStepDeg;

  // Get max size in meters
  double maxSize;
  this->get_parameter_or<double>("max_size", maxSize, 200.);
  // Set max size in voxels : the second dimension of
  // the rolling grid is used for the slice
  this->DenseMap->SetGridSize(maxSize / this->SliceMaxDist);

  // Set the voxels' sizes
  // Set the inner voxel size
  float leafSize;
  this->get_parameter_or<float>("leaf_size", leafSize, 0.1);
  this->DenseMap->SetLeafSize(leafSize);
  // Set the outer voxel size,
  // it should be greater than the inner voxel
  this->DenseMap->SetVoxelResolution(std::max(3. * leafSize, this->SliceMaxDist));
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
  // Map is not rolled because if some points are wrong or a frame is bad,
  // the whole map will be rolled and parts will be forgotten.
  // The size of the map is relative to the first frame reveived
  this->DenseMap->Add(registeredCloud, false, false);
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
void AggregationNode::ResetService(const std::shared_ptr<lidar_slam::srv::Reset::Request> req,
                                   const std::shared_ptr<lidar_slam::srv::Reset::Response> res)
{
  this->DenseMap->Reset();
  RCLCPP_INFO_STREAM(this->get_logger(), "Resetting aggregation");
  res->success = true;
}

//------------------------------------------------------------------------------
void AggregationNode::PoseCallback(const nav_msgs::msg::Odometry& poseMsg)
{
  if (!this->DoExtractSlice)
    return;

  // Store pose
  this->Positions.push_back(Utils::PoseMsgToIsometry(poseMsg.pose.pose).translation());
  while ((this->Positions.back() - this->Positions.front()).norm() > this->TrajectoryMaxLength)
    this->Positions.pop_front();

  if (this->Positions.size() < 2)
    return;

  // Extract the slice boundary and compute its area
  // using the last positions and the map
  CloudS boundary;
  double area = this->ExtractSlice(this->SliceWidth, this->SliceMaxDist, this->AngleStep, boundary);

  // Publish the area value
  std_msgs::msg::Float64 areaMsg;
  areaMsg.data = area;
  this->SliceAreaPublisher->publish(areaMsg);

  // Publish the slice points
  Pcl2_msg sliceMsg;
  pcl::toROSMsg(boundary, sliceMsg);
  this->SlicePublisher->publish(sliceMsg);
}

//------------------------------------------------------------------------------
double AggregationNode::ExtractSlice(double sliceWidth, double sliceMaxDist, double angleStep, CloudS& boundary)
{
  // Shortcut to last position
  Eigen::Vector3d& currPosition = this->Positions.back();
  // Deduce the motion direction
  Eigen::Vector3d motionDirection = (currPosition - this->Positions.front()).normalized();
  // Extract the outer voxel in which lays the new pose
  // To reduce the projection tests
  CloudS tmpCloud;
  tmpCloud.resize(1);
  tmpCloud[0].getVector3fMap() = currPosition.cast<float>();
  this->DenseMap->BuildSubMap(tmpCloud, this->MinSlicePtsWithoutMovObjects);
  CloudS::Ptr submap = this->DenseMap->GetSubMap();

  // Initialize the slice pointcloud
  CloudS slice;
  slice.header = this->Pointcloud->header;
  slice.reserve(submap->size());

  // Project them onto the slice plane
  for (const auto& pt : *submap)
  {
    // Check if point in sphere around the trajectory pose
    if ((pt.getVector3fMap().cast<double>() - currPosition).norm() > sliceMaxDist)
      continue;

    double ptDistance = (pt.getVector3fMap().cast<double>() - currPosition).dot(motionDirection);
    // Check if the point is not too far from the trajectory point
    if (std::abs(ptDistance) > sliceWidth)
      continue;

    slice.emplace_back(pt);
    slice.back().getVector3fMap() -= (ptDistance * motionDirection).cast<float>();
  }

  // Create the boundary using a moving average
  // 1. Create a circular histogram
  std::vector<std::vector<Eigen::Vector3d>> histogram(std::ceil((2. * M_PI + 1e-6) / angleStep));
  Eigen::Vector3d refDir = (slice.front().getVector3fMap().cast<double>() - currPosition).normalized();
  for (const auto& pt : slice)
  {
    Eigen::Vector3d currVec = (pt.getVector3fMap().cast<double>() - currPosition).normalized();
    double dotProduct = std::min(std::max(currVec.dot(refDir), -1.), 1.);
    double angle = std::acos(dotProduct);
    if (currVec.cross(refDir).dot(motionDirection) < 0.)
      angle = 2. * M_PI - angle;
    histogram[int(angle / angleStep)].push_back(pt.getVector3fMap().cast<double>());
  }

  // 2. Average bins and fill the boundary cloud
  boundary.header = this->Pointcloud->header;
  boundary.reserve(histogram.size());
  for (const auto& bin : histogram)
  {
    if (bin.empty())
      continue;
    Eigen::Vector3d ptMean = Eigen::Vector3d::Zero();
    for (const auto& pt : bin)
      ptMean += pt;
    ptMean /= bin.size();
    PointS ptBoundary;
    ptBoundary.getVector3fMap() = ptMean.cast<float>();
    boundary.emplace_back(ptBoundary);
  }

  // Browse over the bins to build the area
  double area = 0.;
  for (unsigned int i = 1; i < boundary.size(); ++i)
  {
    Eigen::Vector3d vec1 = boundary[i].getVector3fMap().cast<double>() - currPosition;
    Eigen::Vector3d vec2 = boundary[i - 1].getVector3fMap().cast<double>() - currPosition;
    area += 0.5 * (vec1).cross(vec2).norm();
  }

  return area;
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
