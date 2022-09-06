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


#include "LidarSlamNode.h"
#include "ros_transform_utils.h"

#include <LidarSlam/Utilities.h>
#include "pcl_conversions/pcl_conversions.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

// Macro to publish with a cast toward the type of publisher
// because Ros2, the publisher has a template
#define publishWithCast(publisher, type_msg, message) \
    std::static_pointer_cast<rclcpp::Publisher<type_msg>>(publisher)->publish(message);

enum Output
{
  POSE_ODOM,                 // Publish SLAM pose as an Odometry msg on 'slam_odom' topic (default : true).
  POSE_TF,                   // Publish SLAM pose as a TF from 'odometry_frame' to 'tracking_frame' (default : true).
  POSE_PREDICTION_ODOM,      // Publish latency-corrected SLAM pose as an Odometry msg on 'slam_predicted_odom' topic.
  POSE_PREDICTION_TF,        // Publish latency-corrected SLAM pose as a TF from 'odometry_frame' to '<tracking_frame>_prediction'.

  EDGES_MAP,                 // Publish edge keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/edges'.
  INTENSITY_EDGES_MAP,       // Publish intensity edge keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/intensity_edges'.
  PLANES_MAP,                // Publish plane keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/planes'.
  BLOBS_MAP,                 // Publish blob keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/blobs'.

  EDGES_SUBMAP,              // Publish edge keypoints submap as a LidarPoint PointCloud2 msg to topic 'submaps/edges'.
  INTENSITY_EDGES_SUBMAP,    // Publish intensity edge keypoints submap as a LidarPoint PointCloud2 msg to topic 'submaps/intensity_edges'.
  PLANES_SUBMAP,             // Publish plane keypoints submap as a LidarPoint PointCloud2 msg to topic 'submaps/planes'.
  BLOBS_SUBMAP,              // Publish blob keypoints submap as a LidarPoint PointCloud2 msg to topic 'submaps/blobs'.

  EDGE_KEYPOINTS,            // Publish extracted edge keypoints from current frame as a PointCloud2 msg to topic 'keypoints/edges'.
  INTENSITY_EDGE_KEYPOINTS,  // Publish extracted intensity edge keypoints from current frame as a PointCloud2 msg to topic 'keypoints/intensity_edges'.
  PLANE_KEYPOINTS,           // Publish extracted plane keypoints from current frame as a PointCloud2 msg to topic 'keypoints/planes'.
  BLOB_KEYPOINTS,            // Publish extracted blob keypoints from current frame as a PointCloud2 msg to topic 'keypoints/blobs'.

  SLAM_REGISTERED_POINTS,    // Publish SLAM pointcloud as LidarPoint PointCloud2 msg to topic 'slam_registered_points'.

  CONFIDENCE,                // Publish confidence estimators on output pose to topic 'slam_confidence'.

  PGO_PATH,                  // Publish optimized SLAM trajectory as Path msg to 'pgo_slam_path' latched topic.
};

//==============================================================================
//   Basic SLAM use
//==============================================================================

//------------------------------------------------------------------------------
LidarSlamNode::LidarSlamNode(std::string name_node, const rclcpp::NodeOptions& options):
  Node(name_node, options)
{
  // Init Tf2 variables
  // doc: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html
  this->TfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->TfListener = std::make_shared<tf2_ros::TransformListener>(*this->TfBuffer);
  this->TfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  this->StaticTfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // ***************************************************************************
  // Init SLAM state
  this->SetSlamInitialState();
  // Get SLAM params
  this->SetSlamParameters();

  // Use GPS data for GPS/SLAM calibration or Pose Graph Optimization.
  this->get_parameter<bool>("external_sensors.gps.use_gps", this->UseGps);

  // Use tags data for local optimization.
  this->get_parameter_or<bool>("external_sensors.landmark_detector.use_tags", this->UseTags, false);

  // ***************************************************************************
  // Init ROS publishers

  #define initPublisher(publisher, topic, type, rosParam, publishDefault, queue, latch)   \
    this->get_parameter_or<bool>(rosParam, this->Publish[publisher], publishDefault);   \
    if (this->Publish[publisher])                                                         \
      this->Publishers[publisher] = this->create_publisher<type>(topic, queue);

  this->get_parameter_or<bool>("output.pose.tf",          this->Publish[POSE_TF],             true);
  this->get_parameter_or<bool>("output.pose.predicted_tf", this->Publish[POSE_PREDICTION_TF], false);

  initPublisher(POSE_ODOM,            "slam_odom",           nav_msgs::msg::Odometry, "output.pose.odom",           true,  1, false);
  initPublisher(POSE_PREDICTION_ODOM, "slam_predicted_odom", nav_msgs::msg::Odometry, "output.pose.predicted_odom", false, 1, false);
  

  if(this->LidarSlam.KeypointTypeEnabled(LidarSlam::EDGE))
  {
    initPublisher(EDGES_MAP,  "maps/edges",  Pcl2_msg, "output.maps.edges",  true, 1, false);
    initPublisher(EDGES_SUBMAP,  "submaps/edges",  Pcl2_msg, "output.submaps.edges",  true, 1, false);
    initPublisher(EDGE_KEYPOINTS,  "keypoints/edges",  Pcl2_msg, "output.keypoints.edges",  true, 1, false);
  }

  if(this->LidarSlam.KeypointTypeEnabled(LidarSlam::INTENSITY_EDGE))
  {
    initPublisher(INTENSITY_EDGES_MAP,  "maps/intensity_edges",  Pcl2_msg, "output.maps.intensity_edges",  true, 1, false);
    initPublisher(INTENSITY_EDGES_SUBMAP,  "submaps/intensity_edges",  Pcl2_msg, "output.submaps.intensity_edges",  true, 1, false);
    initPublisher(INTENSITY_EDGE_KEYPOINTS,  "keypoints/intensity_edges",  Pcl2_msg, "output.keypoints.intensity_edges",  true, 1, false);
  }

  if(this->LidarSlam.KeypointTypeEnabled(LidarSlam::PLANE))
  {
    initPublisher(PLANES_MAP, "maps/planes", Pcl2_msg, "output.maps.planes", true, 1, false);
    initPublisher(PLANES_SUBMAP, "submaps/planes", Pcl2_msg, "output.submaps.planes", true, 1, false);
    initPublisher(PLANE_KEYPOINTS, "keypoints/planes", Pcl2_msg, "output.keypoints.planes", true, 1, false);
  }

  if(this->LidarSlam.KeypointTypeEnabled(LidarSlam::BLOB))
  {
    initPublisher(BLOBS_MAP,  "maps/blobs",  Pcl2_msg, "output.maps.blobs",  true, 1, false);
    initPublisher(BLOBS_SUBMAP,  "submaps/blobs",  Pcl2_msg, "output.submaps.blobs",  true, 1, false);
    initPublisher(BLOB_KEYPOINTS,  "keypoints/blobs",  Pcl2_msg, "output.keypoints.blobs",  true, 1, false);
  }

  initPublisher(SLAM_REGISTERED_POINTS, "slam_registered_points", Pcl2_msg, "output.registered_points", true, 1, false);

  initPublisher(CONFIDENCE, "slam_confidence", lidar_slam_interfaces::msg::Confidence, "output.confidence", true, 1, false);

  if (this->UseGps || this->UseTags)
    initPublisher(PGO_PATH, "pgo_slam_path", nav_msgs::msg::Path, "graph.publish_path", false, 1, true);

  // Init ROS subscribers

  // LiDAR inputs
  //check if the parameter type is a string_array
  //if the parameter doesn't exist or is a string, push back a value
  std::vector<std::string> lidarTopics;
  if (this->has_parameter("intput") && this->get_parameter_types({"input"})[0] == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    this->get_parameter<std::vector<std::string>>("input", lidarTopics);
  else
  {
    std::string input;
    this->get_parameter_or<std::string>("input", input, "lidar_points");
    lidarTopics.push_back(input);
  }

  this->CloudSubs.push_back(this->create_subscription<Pcl2_msg>(lidarTopics[0], 1,
                                                                            std::bind(&LidarSlamNode::ScanCallback, this, std::placeholders::_1)));
  PRINT_VERBOSE(3, "Using LiDAR frames on topic '" << lidarTopics[0] << "'");
  for (unsigned int lidarTopicId = 1; lidarTopicId < lidarTopics.size(); lidarTopicId++)
  {
    this->CloudSubs.push_back(this->create_subscription<Pcl2_msg>(lidarTopics[lidarTopicId], 1,
                                                                                std::bind(&LidarSlamNode::SecondaryScanCallback, this, std::placeholders::_1)));
    PRINT_VERBOSE(3, "Using secondary LiDAR frames on topic '" << lidarTopics[lidarTopicId] << "'");
  }

  // Set SLAM pose from external guess
  this->SetPoseSub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("set_slam_pose", 1,
                                                                                              std::bind(&LidarSlamNode::SetPoseCallback, this, std::placeholders::_1));

  // Init logging of GPS data for GPS/SLAM calibration or Pose Graph Optimization.
  if (this->UseGps)
    this->GpsOdomSub = this->create_subscription<nav_msgs::msg::Odometry>("gps_odom", 1,
                                                std::bind(&LidarSlamNode::GpsCallback, this, std::placeholders::_1));

  // Init logging of landmark data
  if (this->UseTags)
  {
    this->LandmarkSub = this->create_subscription<apriltag_ros::msg::AprilTagDetectionArray>("tag_detections", 200,
                                                                                      std::bind(&LidarSlamNode::TagCallback, this, std::placeholders::_1));
  }

  PRINT_VERBOSE(0, BOLD_GREEN("LiDAR SLAM is ready !"));
}

//------------------------------------------------------------------------------
LidarSlamNode::~LidarSlamNode()
{
}

//------------------------------------------------------------------------------
void LidarSlamNode::ScanCallback(const Pcl2_msg& pcl_msg)
{
  CloudS::Ptr cloudS_ptr = std::make_shared<CloudS>();

  pcl::fromROSMsg(pcl_msg, *cloudS_ptr);

  if(cloudS_ptr->empty())
  {
    PRINT_WARNING("Input point cloud sent by Lidar sensor driver is empty -> ignoring message");
    return;
  }

  // Compute time offset
  // Get ROS frame reception time
  
  double TimeFrameReceptionPOSIX = this->now().seconds();
  // Get last acquired point timestamp
  double TimeLastPoint = LidarSlam::Utils::PclStampToSec(cloudS_ptr->header.stamp) + cloudS_ptr->back().time;
  // Compute offset
  double potentialOffset = TimeLastPoint - TimeFrameReceptionPOSIX;
  // Get current offset
  double absCurrentOffset = std::abs(this->LidarSlam.GetSensorTimeOffset());
  // If the current computed offset is more accurate, replace it
  if (absCurrentOffset < 1e-6 || std::abs(potentialOffset) < absCurrentOffset)
    this->LidarSlam.SetSensorTimeOffset(potentialOffset);

  // Update TF from BASE to LiDAR
  if (!this->UpdateBaseToLidarOffset(cloudS_ptr->header.frame_id, cloudS_ptr->front().device_id))
    return;

  // Set the SLAM main input frame at first position
  this->Frames.insert(this->Frames.begin(), cloudS_ptr);

  // Run SLAM : register new frame and update localization and map.
  this->LidarSlam.AddFrames(this->Frames);
  this->Frames.clear();

  // Publish SLAM output as requested by user
  this->PublishOutput();
}

//------------------------------------------------------------------------------
void LidarSlamNode::SecondaryScanCallback(const Pcl2_msg& pcl_msg)
{
  CloudS::Ptr cloudS_ptr = std::make_shared<CloudS>();

  pcl::fromROSMsg(pcl_msg, *cloudS_ptr);
  
  if(cloudS_ptr->empty())
  {
    PRINT_WARNING("Secondary input point cloud sent by Lidar sensor driver is empty -> ignoring message");
    return;
  }

  // Update TF from BASE to LiDAR for this device
  if (!this->UpdateBaseToLidarOffset(cloudS_ptr->header.frame_id, cloudS_ptr->front().device_id))
    return;

  // Add new frame to SLAM input frames
  this->Frames.push_back(cloudS_ptr);
}

//------------------------------------------------------------------------------
void LidarSlamNode::GpsCallback(const nav_msgs::msg::Odometry& gpsMsg)
{
    if (!this->UseGps)
      return;

    // Transform to apply to points represented in GPS frame to express them in base frame
    Eigen::Isometry3d baseToGps;
    if (Utils::Tf2LookupTransform(baseToGps, *this->TfBuffer, this->TrackingFrameId, gpsMsg.header.frame_id, gpsMsg.header.stamp))
    {
      PRINT_VERBOSE(3, "Adding GPS info");
      // Get gps pose
      this->LastGpsMeas.Position = Utils::PoseMsgToIsometry(gpsMsg.pose.pose).translation();
      // Get gps timestamp
      this->LastGpsMeas.Time = gpsMsg.header.stamp.sec + gpsMsg.header.stamp.nanosec * 1e-9;

      // Get tag covariance
      bool ValidCovariance = false;
      for (int i = 0; i < 6; ++i)
      {
        for (int j = 0; j < 6; ++j)
        {
          this->LastGpsMeas.Covariance(i, j) = gpsMsg.pose.covariance[i * 6 + j];
          if (abs(this->LastGpsMeas.Covariance(i, j)) > 1e-10)
            ValidCovariance = true;
        }
      }
      if (!ValidCovariance)
        this->LastGpsMeas.Covariance = Eigen::Matrix3d::Identity() * 1e-4;

      if (!this->LidarSlam.GpsHasData())
        this->LidarSlam.SetGpsCalibration(baseToGps);

      // Add gps measurement to measurements list
      this->LidarSlam.AddGpsMeasurement(this->LastGpsMeas);
      this->GpsLastTime = rclcpp::Time(this->LastGpsMeas.Time);
      this->GpsFrameId = gpsMsg.header.frame_id;
    }
    else
      PRINT_WARNING("The transform between the GPS and the tracking frame was not found -> GPS info ignored");
}

// //------------------------------------------------------------------------------
int LidarSlamNode::BuildId(const std::vector<int>& ids)
{
  int id = ids[0];
  if (ids.size() > 1)
  {
    id = 0;
    int power = 0;
    for (unsigned int i = 0; i < ids.size(); ++i)
    {
      power += int(std::log10(ids[i]));
      id += std::pow(10, power) * ids[i];
    }
  }
  return id;
}

//------------------------------------------------------------------------------
void LidarSlamNode::TagCallback(const apriltag_ros::msg::AprilTagDetectionArray& tagsInfo)
{
  if (!this->UseTags)
    return;
  for (auto& tagInfo : tagsInfo.detections)
  {
    // Transform to apply to points represented in detector frame to express them in base frame
    Eigen::Isometry3d baseToLmDetector;
    if (Utils::Tf2LookupTransform(baseToLmDetector, *this->TfBuffer, this->TrackingFrameId, tagInfo.pose.header.frame_id, tagInfo.pose.header.stamp))
    {
      PRINT_VERBOSE(3, "Adding tag info");
      LidarSlam::ExternalSensors::LandmarkMeasurement lm;
      // Get tag pose
      lm.TransfoRelative = Utils::PoseMsgToIsometry(tagInfo.pose.pose.pose);
      // Get tag timestamp
      lm.Time = tagInfo.pose.header.stamp.sec + tagInfo.pose.header.stamp.nanosec * 1e-9;

      // Get tag covariance
      bool ValidCovariance = false;
      for (int i = 0; i < 6; ++i)
      {
        for (int j = 0; j < 6; ++j)
        {
          lm.Covariance(i, j) = tagInfo.pose.pose.covariance[i * 6 + j];
          if (abs(lm.Covariance(i, j)) > 1e-10)
            ValidCovariance = true;
        }
      }
      if (!ValidCovariance)
        lm.Covariance = Eigen::Matrix6d::Identity() * 1e-4;

      // Compute tag ID
      int id = this->BuildId(tagInfo.id);

      // Add detector calibration for the first tag detection
      if (!this->LidarSlam.LmHasData())
        this->LidarSlam.SetLmDetectorCalibration(baseToLmDetector);

      // Add tag detection to measurements
      this->LidarSlam.AddLandmarkMeasurement(lm, id);

      if (this->PublishTags)
      {
        // Publish tf
        geometry_msgs::msg::TransformStamped tfStamped;
        tfStamped.header.stamp = tagInfo.pose.header.stamp;
        tfStamped.header.frame_id = this->TrackingFrameId;
        tfStamped.child_frame_id = "tag_" + std::to_string(id);
        tfStamped.transform = Utils::IsometryToTfMsg(lm.TransfoRelative);
        this->TfBroadcaster->sendTransform(tfStamped);
      }
    }
    else
      PRINT_WARNING("The transform between the landmark detector and the tracking frame was not found -> landmarks info ignored");
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::LoadLandmarks(const std::string& path)
{
  // Check the file
  if (path.substr(path.find_last_of(".") + 1) != "csv")
  {
    PRINT_ERROR("The landmarks file is not csv : landmarks absolute constraint can not be used");
    return;
  }

  std::ifstream lmFile(path);
  if (lmFile.fail())
  {
    PRINT_ERROR("The landmarks csv file " << path << " was not found : landmarks absolute constraint can not be used");
    return;
  }

  const unsigned int fieldsNumber = 43;
  // Check which delimiter is used
  std::string lmStr;
  std::vector<std::string> delimiters = {";", ",", " ", "\t"};
  std::string delimiter;
  getline (lmFile, lmStr);
  std::vector<std::string> fields;

  for (const auto& del : delimiters)
  {
    fields.clear();
    size_t pos = 0;
    std::string headerLine = lmStr;
    pos = headerLine.find(del);
    while (pos != std::string::npos)
    {
      fields.push_back(headerLine.substr(0, pos));
      headerLine.erase(0, pos + del.length());
      pos = headerLine.find(del);
    }
    // If there is some element after the last delimiter, add it
    if (!headerLine.substr(0, pos).empty())
      fields.push_back(headerLine.substr(0, pos));
    // Check that the number of fields is correct
    if (fields.size() == fieldsNumber)
    {
      delimiter = del;
      break;
    }
  }
  if (fields.size() != fieldsNumber)
  {
    PRINT_WARNING("The landmarks csv file is ill formed : " << fields.size() << " fields were found (" << fieldsNumber
                    << " expected), landmarks absolute poses will not be used");
    return;
  }

  int lineIdx = 1;
  int ntags = 0;
  do
  {
    size_t pos = 0;
    std::vector<std::string> lm;
    while ((pos = lmStr.find(delimiter)) != std::string::npos)
    {
      // Remove potential extra spaces after the delimiter
      unsigned int charIdx = 0;
      while (charIdx < lmStr.size() && lmStr[charIdx] == ' ')
        ++charIdx;
      lm.push_back(lmStr.substr(charIdx, pos));
      lmStr.erase(0, pos + delimiter.length());
    }
    lm.push_back(lmStr.substr(0, pos));
    if (lm.size() != fieldsNumber)
    {
      PRINT_WARNING("landmark on line " + std::to_string(lineIdx) + " is not correct -> Skip");
      ++lineIdx;
      continue;
    }

    // Check numerical values in the studied line
    bool numericalIssue = false;
    for (std::string field : lm)
    {
      try
      {
        std::stof(field);
      }
      catch(std::invalid_argument& e)
      {
        numericalIssue = true;
        break;
      }
    }
    if (numericalIssue)
    {
      // if this is the first line, it might be a header line,
      // else, print a warning
      if (lineIdx > 1)
        PRINT_WARNING("landmark on line " + std::to_string(lineIdx) + " contains a not numerical value -> Skip");
      ++lineIdx;
      continue;
    }

    // Build measurement
    // Set landmark id
    int id = std::stoi(lm[0]);
    // Fill pose
    Eigen::Vector6d absolutePose;
    absolutePose << std::stof(lm[1]), std::stof(lm[2]), std::stof(lm[3]), std::stof(lm[4]), std::stof(lm[5]), std::stof(lm[6]);
    // Fill covariance
    Eigen::Matrix6d absolutePoseCovariance;
    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < 6; ++j)
        absolutePoseCovariance(i, j) = std::stof(lm[7 + 6 * i + j]);
    }
    // Add a new landmark manager for absolute constraint computing
    this->LidarSlam.AddLandmarkManager(id, absolutePose, absolutePoseCovariance);
    PRINT_VERBOSE(3, "Tag #" << id << " initialized to \n" << absolutePose.transpose());
    ++ntags;
    ++lineIdx;
  }
  while (getline (lmFile, lmStr));

  lmFile.close();
}

//------------------------------------------------------------------------------
void LidarSlamNode::SetPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
{
  // Get transform between msg frame and odometry frame
  Eigen::Isometry3d msgFrameToOdom;
  if (Utils::Tf2LookupTransform(msgFrameToOdom, *this->TfBuffer, msg.header.frame_id, this->OdometryFrameId, msg.header.stamp))
  {
    // Compute pose in odometry frame and set SLAM pose
    Eigen::Isometry3d odomToBase = msgFrameToOdom.inverse() * Utils::PoseMsgToIsometry(msg.pose.pose);
    this->LidarSlam.SetWorldTransformFromGuess(odomToBase);
    PRINT_WARNING("SLAM pose set to :\n" << odomToBase.matrix());
    // TODO: properly deal with covariance: rotate it, pass it to SLAM, notify trajectory jump?
  }
}

//==============================================================================
//   Utilities
//==============================================================================

//------------------------------------------------------------------------------
bool LidarSlamNode::UpdateBaseToLidarOffset(const std::string& lidarFrameId, uint8_t lidarDeviceId)
{
  // If tracking frame is different from input frame, get TF from LiDAR to BASE
  if (lidarFrameId != this->TrackingFrameId)
  {
    // We expect a static transform between BASE and LIDAR, so we don't care
    // about timestamp and get only the latest transform
    Eigen::Isometry3d baseToLidar;
    if (Utils::Tf2LookupTransform(baseToLidar, *this->TfBuffer, this->TrackingFrameId, lidarFrameId))
      this->LidarSlam.SetBaseToLidarOffset(baseToLidar, lidarDeviceId);
    else
      return false;
  }
  return true;
}

// //------------------------------------------------------------------------------
void LidarSlamNode::PublishOutput()
{

  LidarSlam::LidarState& currentState = this->LidarSlam.GetLastState();
  double currentTime = currentState.Time;
    geometry_msgs::msg::TransformStamped tfMsg;
    tfMsg.header.stamp = rclcpp::Time(currentTime);
    tfMsg.header.frame_id = this->OdometryFrameId;
    tfMsg.child_frame_id = this->TrackingFrameId;
    tfMsg.transform = Utils::IsometryToTfMsg(currentState.Isometry);
    this->TfBroadcaster->sendTransform(tfMsg);


  // Publish a pointcloud only if required and if someone is listening to it to spare bandwidth.
  // Change to publish pcl2 msgs
  // url : https://github.com/mikeferguson/ros2_cookbook/blob/main/rclcpp/pcl.md
  #define publishPointCloud(publisher, pc)                                                  \
  if (this->Publishers[publisher]->get_subscription_count())    \
    {                                                                                       \
      Pcl2_msg pcl_msg;                                                \
      pcl::toROSMsg(*pc, pcl_msg);                                                          \
      publishWithCast(this->Publishers[publisher], Pcl2_msg, pcl_msg)  \
    }

  publishPointCloud(PLANES_MAP, this->LidarSlam.GetMap(LidarSlam::PLANE));
}

//------------------------------------------------------------------------------
void LidarSlamNode::SetSlamParameters()
{
  #define SetSlamParam(type, rosParam, slamParam) { type val; if (this->get_parameter<type>(rosParam, val)) this->LidarSlam.Set##slamParam(val); }
  // General
  SetSlamParam(bool,   "slam.2d_mode", TwoDMode)
  SetSlamParam(int,    "slam.verbosity", Verbosity)
  SetSlamParam(int,    "slam.n_threads", NbThreads)
  SetSlamParam(double,    "slam.logging.timeout", LoggingTimeout)
  SetSlamParam(bool,   "slam.logging.only_keyframes", LogOnlyKeyframes)

  int egoMotionMode;
  if (this->get_parameter<int>("slam.ego_motion", egoMotionMode))
  {
    LidarSlam::EgoMotionMode egoMotion = static_cast<LidarSlam::EgoMotionMode>(egoMotionMode);
    if (egoMotion != LidarSlam::EgoMotionMode::NONE &&
        egoMotion != LidarSlam::EgoMotionMode::MOTION_EXTRAPOLATION &&
        egoMotion != LidarSlam::EgoMotionMode::REGISTRATION &&
        egoMotion != LidarSlam::EgoMotionMode::MOTION_EXTRAPOLATION_AND_REGISTRATION)
    {
      PRINT_ERROR("Invalid ego-motion mode (" << egoMotionMode << "). Setting it to 'MOTION_EXTRAPOLATION'.");
      egoMotion = LidarSlam::EgoMotionMode::MOTION_EXTRAPOLATION;
    }
    this->LidarSlam.SetEgoMotion(egoMotion);
  }

  int undistortionMode;
  if (this->get_parameter<int>("slam.undistortion", undistortionMode))
  {
    LidarSlam::UndistortionMode undistortion = static_cast<LidarSlam::UndistortionMode>(undistortionMode);
    if (undistortion != LidarSlam::UndistortionMode::NONE &&
        undistortion != LidarSlam::UndistortionMode::ONCE &&
        undistortion != LidarSlam::UndistortionMode::REFINED)
    {
      PRINT_ERROR("Invalid undistortion mode (" << undistortion << "). Setting it to 'REFINED'.");
      undistortion = LidarSlam::UndistortionMode::REFINED;
    }
    LidarSlam.SetUndistortion(undistortion);
  }

  int pointCloudStorage;
  if (this->get_parameter<int>("slam.logging.storage_type", pointCloudStorage))
  {
    LidarSlam::PointCloudStorageType storage = static_cast<LidarSlam::PointCloudStorageType>(pointCloudStorage);
    if (storage != LidarSlam::PointCloudStorageType::PCL_CLOUD &&
        storage != LidarSlam::PointCloudStorageType::OCTREE_COMPRESSED &&
        storage != LidarSlam::PointCloudStorageType::PCD_ASCII &&
        storage != LidarSlam::PointCloudStorageType::PCD_BINARY &&
        storage != LidarSlam::PointCloudStorageType::PCD_BINARY_COMPRESSED)
    {
      PRINT_ERROR("Incorrect pointcloud logging type value (" << storage << "). Setting it to 'PCL'.");
      storage = LidarSlam::PointCloudStorageType::PCL_CLOUD;
    }
    LidarSlam.SetLoggingStorage(storage);
  }

  // Frame Ids
  this->get_parameter<std::string>("odometry_frame", this->OdometryFrameId);
  this->LidarSlam.SetWorldFrameId(this->OdometryFrameId);
  this->get_parameter<std::string>("tracking_frame", this->TrackingFrameId);
  this->LidarSlam.SetBaseFrameId(this->TrackingFrameId);

  // Keypoint extractors
  auto InitKeypointsExtractor = [this](auto& ke, const std::string& prefix)
  {
    #define SetKeypointsExtractorParam(type, rosParam, keParam) {type val; if (this->get_parameter<type>(rosParam, val)) ke->Set##keParam(val);}
    SetKeypointsExtractorParam(int,   "slam.n_threads", NbThreads)
    SetKeypointsExtractorParam(float, prefix + "min_distance_to_sensor", MinDistanceToSensor)
    SetKeypointsExtractorParam(float, prefix + "min_beam_surface_angle", MinBeamSurfaceAngle)
    SetKeypointsExtractorParam(float, prefix + "min_azimuth", AzimuthMin)
    SetKeypointsExtractorParam(float, prefix + "max_azimuth", AzimuthMax)
    SetKeypointsExtractorParam(float, prefix + "plane_sin_angle_threshold", PlaneSinAngleThreshold)
    SetKeypointsExtractorParam(float, prefix + "edge_sin_angle_threshold", EdgeSinAngleThreshold)
    SetKeypointsExtractorParam(float, prefix + "edge_depth_gap_threshold", EdgeDepthGapThreshold)
    SetKeypointsExtractorParam(int, prefix + "edge_nb_gap_points", EdgeNbGapPoints)
    SetKeypointsExtractorParam(float, prefix + "edge_intensity_gap_threshold", EdgeIntensityGapThreshold)
    SetKeypointsExtractorParam(int,   prefix + "max_points", MaxPoints)
    SetKeypointsExtractorParam(float, prefix + "voxel_grid_resolution", VoxelResolution)
    SetKeypointsExtractorParam(float, prefix + "input_sampling_ratio", InputSamplingRatio)

    #define EnableKeypoint(kType) \
    { \
      bool enabled = false; \
      std::string name = LidarSlam::KeypointTypeNames.at(kType); \
      if (this->get_parameter<bool>(prefix + "enable."+ name, enabled)) \
        this->LidarSlam.EnableKeypointType(kType, enabled); \
      if (enabled) \
        {PRINT_VERBOSE(3, "Keypoint of type " + name + " enabled");} \
      else \
        PRINT_VERBOSE(3, "Keypoint of type " + name + " disabled"); \
    }
    EnableKeypoint(LidarSlam::Keypoint::EDGE);
    EnableKeypoint(LidarSlam::Keypoint::INTENSITY_EDGE);
    EnableKeypoint(LidarSlam::Keypoint::PLANE);
    EnableKeypoint(LidarSlam::Keypoint::BLOB);
  };

  // Multi-LiDAR devices
  std::vector<int64_t> deviceIds;
  if (this->get_parameter("slam.ke.device_ids", deviceIds))
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Multi-LiDAR devices setup");
    for (auto deviceId: deviceIds)
    {
      // Init Keypoint extractor with default params
      auto ke = std::make_shared<LidarSlam::SpinningSensorKeypointExtractor>();

      // Change default parameters using ROS parameter server
      std::string prefix = "slam.ke.device_" + std::to_string(deviceId) + ".";
      InitKeypointsExtractor(ke, prefix);

      // Add extractor to SLAM
      this->LidarSlam.SetKeyPointsExtractor(ke, deviceId);
      RCLCPP_INFO_STREAM(this->get_logger(), "Adding keypoint extractor for LiDAR device " << deviceId);
    }
  }
  // Single LiDAR device
  else
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Single LiDAR device setup");
    auto ke = std::make_shared<LidarSlam::SpinningSensorKeypointExtractor>();
    InitKeypointsExtractor(ke, "slam.ke.");
    this->LidarSlam.SetKeyPointsExtractor(ke);
  }


  // Ego motion
  SetSlamParam(int,    "slam.ego_motion_registration.ICP_max_iter", EgoMotionICPMaxIter)
  SetSlamParam(int,    "slam.ego_motion_registration.LM_max_iter", EgoMotionLMMaxIter)
  SetSlamParam(double, "slam.ego_motion_registration.max_neighbors_distance", EgoMotionMaxNeighborsDistance)
  SetSlamParam(int,    "slam.ego_motion_registration.edge_nb_neighbors", EgoMotionEdgeNbNeighbors)
  SetSlamParam(int,    "slam.ego_motion_registration.edge_min_nb_neighbors", EgoMotionEdgeMinNbNeighbors)
  SetSlamParam(double, "slam.ego_motion_registration.edge_max_model_error", EgoMotionEdgeMaxModelError)
  SetSlamParam(int,    "slam.ego_motion_registration.plane_nb_neighbors", EgoMotionPlaneNbNeighbors)
  SetSlamParam(double, "slam.ego_motion_registration.planarity_threshold", EgoMotionPlanarityThreshold)
  SetSlamParam(double, "slam.ego_motion_registration.plane_max_model_error", EgoMotionPlaneMaxModelError)
  SetSlamParam(double, "slam.ego_motion_registration.init_saturation_distance", EgoMotionInitSaturationDistance)
  SetSlamParam(double, "slam.ego_motion_registration.final_saturation_distance", EgoMotionFinalSaturationDistance)

  // Localization
  SetSlamParam(int,    "slam.localization.ICP_max_iter", LocalizationICPMaxIter)
  SetSlamParam(int,    "slam.localization.LM_max_iter", LocalizationLMMaxIter)
  SetSlamParam(double, "slam.localization.max_neighbors_distance", LocalizationMaxNeighborsDistance)
  SetSlamParam(int,    "slam.localization.edge_nb_neighbors", LocalizationEdgeNbNeighbors)
  SetSlamParam(int,    "slam.localization.edge_min_nb_neighbors", LocalizationEdgeMinNbNeighbors)
  SetSlamParam(double, "slam.localization.edge_max_model_error", LocalizationEdgeMaxModelError)
  SetSlamParam(int,    "slam.localization.plane_nb_neighbors", LocalizationPlaneNbNeighbors)
  SetSlamParam(double, "slam.localization.planarity_threshold", LocalizationPlanarityThreshold)
  SetSlamParam(double, "slam.localization.plane_max_model_error", LocalizationPlaneMaxModelError)
  SetSlamParam(int,    "slam.localization.blob_nb_neighbors", LocalizationBlobNbNeighbors)
  SetSlamParam(double, "slam.localization.init_saturation_distance", LocalizationInitSaturationDistance)
  SetSlamParam(double, "slam.localization.final_saturation_distance", LocalizationFinalSaturationDistance)

  // External sensors
  SetSlamParam(int,  "external_sensors.max_measures", SensorMaxMeasures)
  SetSlamParam(float,  "external_sensors.time_threshold", SensorTimeThreshold)
  SetSlamParam(float,  "external_sensors.landmark_detector.weight", LandmarkWeight)
  SetSlamParam(float,  "external_sensors.landmark_detector.saturation_distance", LandmarkSaturationDistance)
  SetSlamParam(bool,   "external_sensors.landmark_detector.position_only", LandmarkPositionOnly)
  this->get_parameter_or<bool>("external_sensors.landmark_detector.publish_tags", this->PublishTags, false);

  // Graph parameters
  SetSlamParam(std::string, "graph.g2o_file_name", G2oFileName)
  SetSlamParam(bool,   "graph.fix_first", FixFirstVertex)
  SetSlamParam(bool,   "graph.fix_last", FixLastVertex)

  // Confidence estimators
  // Overlap
  SetSlamParam(float,  "slam.confidence.overlap.sampling_ratio", OverlapSamplingRatio)
  {
    //convert to float for Eigen
    std::vector<float> accf{(float)acc[0], (float)acc[1]};
    this->LidarSlam.SetAccelerationLimits(Eigen::Map<const Eigen::Array2f>(accf.data()));
  }
  std::vector<double> vel;
  if (this->get_parameter("slam.confidence.motion_limits.velocity", vel) && vel.size() == 2)
  {
    //convert to float for Eigen
    std::vector<float> velf{(float)vel[0], (float)vel[1]};
     this->LidarSlam.SetVelocityLimits(Eigen::Map<const Eigen::Array2f>(velf.data()));
  } 
  SetSlamParam(float, "slam.confidence.motion_limits.time_window_duration", TimeWindowDuration)

  // Keyframes
  SetSlamParam(double, "slam.keyframes.distance_threshold", KfDistanceThreshold)
  SetSlamParam(double, "slam.keyframes.angle_threshold", KfAngleThreshold)

  // Maps
  int mapUpdateMode;
  if (this->get_parameter<int>("slam.voxel_grid.update_maps", mapUpdateMode))
  {
    LidarSlam::MappingMode mapUpdate = static_cast<LidarSlam::MappingMode>(mapUpdateMode);
    if (mapUpdate != LidarSlam::MappingMode::NONE &&
        mapUpdate != LidarSlam::MappingMode::ADD_KPTS_TO_FIXED_MAP &&
        mapUpdate != LidarSlam::MappingMode::UPDATE)
    {
      PRINT_ERROR("Invalid map update mode (" << mapUpdateMode << "). Setting it to 'UPDATE'.");
      mapUpdate = LidarSlam::MappingMode::UPDATE;
    }
    this->LidarSlam.SetMapUpdate(mapUpdate);
  }

  double size;
  if (this->get_parameter("slam.voxel_grid.leaf_size.edges", size) && this->LidarSlam.KeypointTypeEnabled(LidarSlam::EDGE))
    this->LidarSlam.SetVoxelGridLeafSize(LidarSlam::EDGE, size);
  if (this->get_parameter("slam.voxel_grid.leaf_size.intensity_edges", size) && this->LidarSlam.KeypointTypeEnabled(LidarSlam::INTENSITY_EDGE))
    this->LidarSlam.SetVoxelGridLeafSize(LidarSlam::INTENSITY_EDGE, size);
  if (this->get_parameter("slam.voxel_grid.leaf_size.planes", size) && this->LidarSlam.KeypointTypeEnabled(LidarSlam::PLANE))
    this->LidarSlam.SetVoxelGridLeafSize(LidarSlam::PLANE, size);
  if (this->get_parameter("slam.voxel_grid.leaf_size.blobs", size) && this->LidarSlam.KeypointTypeEnabled(LidarSlam::BLOB))
    this->LidarSlam.SetVoxelGridLeafSize(LidarSlam::BLOB, size);
  SetSlamParam(double, "slam.voxel_grid.resolution", VoxelGridResolution)
  SetSlamParam(int,    "slam.voxel_grid.size", VoxelGridSize)
  SetSlamParam(int, "slam.voxel_grid.decaying_threshold", VoxelGridDecayingThreshold)
  SetSlamParam(int,    "slam.voxel_grid.min_frames_per_voxel", VoxelGridMinFramesPerVoxel)
  for (auto k : LidarSlam::KeypointTypes)
  {
    if (!this->LidarSlam.KeypointTypeEnabled(k))
      continue;
    int samplingMode;
    if (this->get_parameter("slam.voxel_grid.sampling_mode." + LidarSlam::KeypointTypeNames.at(k), samplingMode))
    {
      LidarSlam::SamplingMode sampling = static_cast<LidarSlam::SamplingMode>(samplingMode);
      if (sampling != LidarSlam::SamplingMode::FIRST &&
          sampling != LidarSlam::SamplingMode::LAST &&
          sampling != LidarSlam::SamplingMode::MAX_INTENSITY &&
          sampling != LidarSlam::SamplingMode::CENTER_POINT &&
          sampling != LidarSlam::SamplingMode::CENTROID)
      {
        PRINT_ERROR("Invalid sampling mode (" << samplingMode << ") for "
                                                   << LidarSlam::Utils::Plural(LidarSlam::KeypointTypeNames.at(k))
                                                   << ". Setting it to 'MAX_INTENSITY'.");
        sampling = LidarSlam::SamplingMode::MAX_INTENSITY;
      }
      this->LidarSlam.SetVoxelGridSamplingMode(k, sampling);
    }
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::SetSlamInitialState()
{
  // Load initial SLAM maps if requested
  std::string mapsPathPrefix = "";
  this->get_parameter<std::string>("maps.initial_maps", mapsPathPrefix);
  if (!mapsPathPrefix.empty())
  {
    PRINT_VERBOSE(3, "Loading initial keypoints maps from PCD.");
    this->LidarSlam.LoadMapsFromPCD(mapsPathPrefix);
  }

  // Load initial Landmarks poses if requested
  std::string lmpath = "";
  this->get_parameter("external_sensors.landmark_detector.landmarks_file_path", lmpath);
  if (!lmpath.empty())
  {
    PRINT_VERBOSE(3, "Loading initial landmarks info from CSV.");
    this->LoadLandmarks(lmpath);
    this->LidarSlam.SetLandmarkConstraintLocal(false);
  }
  else
    this->LidarSlam.SetLandmarkConstraintLocal(true);

  // Set initial SLAM pose if requested
  std::vector<double> initialPose;
  if (this->get_parameter<std::vector<double>>("maps.initial_pose", initialPose) && initialPose.size() == 6)
  {
    Eigen::Isometry3d poseTransform = LidarSlam::Utils::XYZRPYtoIsometry(initialPose.data());
    this->LidarSlam.SetWorldTransformFromGuess(poseTransform);
    PRINT_VERBOSE(3, "Setting initial SLAM pose to:\n" << poseTransform.matrix());
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::BroadcastGpsOffset()
{
  Eigen::Isometry3d offset = this->LidarSlam.GetGpsOffset().inverse();
  geometry_msgs::msg::TransformStamped tfStamped;
  tfStamped.header.stamp = this->GpsLastTime;
  tfStamped.header.frame_id = this->GpsFrameId;
  tfStamped.child_frame_id = this->OdometryFrameId;
  tfStamped.transform = Utils::IsometryToTfMsg(offset);
  this->StaticTfBroadcaster->sendTransform(tfStamped);
}
