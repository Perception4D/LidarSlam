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

#ifdef USE_CV_BRIDGE
#include <cv_bridge/cv_bridge.h>
#endif
//? #include <sensor_msgs/msg/image_encodings.hpp>

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

static const std::vector<std::string> DELIMITERS = {";", ",", " ", "\t"};

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
  // Get SLAM params
  // Init SLAM state
  this->SetSlamParameters();
  this->SetSlamInitialState();

  // Use GPS data for GPS/SLAM calibration or Pose Graph Optimization.
  this->get_parameter_or<bool>("external_sensors.gps.use_gps", this->UseExtSensor[LidarSlam::GPS], false);

  // Use tags data for local optimization.
  this->get_parameter_or<bool>("external_sensors.landmark_detector.use_tags", this->UseExtSensor[LidarSlam::LANDMARK_DETECTOR], false);
  this->LidarSlam.EnablePGOConstraint(LidarSlam::LANDMARK, this->UseExtSensor[LidarSlam::LANDMARK_DETECTOR]);

  // Use camera rgb images in local optimization.
  this->get_parameter_or<bool>("external_sensors.camera.use_camera", this->UseExtSensor[LidarSlam::CAMERA], false);

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

  initPublisher(CONFIDENCE, "slam_confidence", lidar_slam::msg::Confidence, "output.confidence", true, 1, false);

  if (this->UseExtSensor[LidarSlam::GPS] || this->UseExtSensor[LidarSlam::LANDMARK_DETECTOR])
    initPublisher(PGO_PATH, "pgo_slam_path", nav_msgs::msg::Path, "graph.publish_path", false, 1, true);

  // Set frequency of output pose (all poses are published at the end of the frames process)
  this->get_parameter_or<double>("output.pose.frequency", this->TrajFrequency, -1.);

  // ***************************************************************************
  // Init ROS subscribers

  // LiDAR inputs
  this->get_parameter_or<int>("lidars_nb", this->MultiLidarsNb, 1);
  int frameMode;
  this->get_parameter_or<int>("frames_mode", frameMode, 1);
  this->WaitFramesDef = static_cast<LidarSlamNode::FramesCollectionMode>(frameMode);
  this->get_parameter_or<double>("frames_waiting_time", this->WaitFramesTime, 0.2);

  // Check if the parameter type is a string_array
  // If the parameter doesn't exist or is a string, push back a value
  std::vector<std::string> lidarTopics;
  if (this->has_parameter("input") && this->get_parameter_types({"input"})[0] == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    this->get_parameter<std::vector<std::string>>("input", lidarTopics);
  else
  {
    std::string input;
    this->get_parameter_or<std::string>("input", input, "lidar_points");
    lidarTopics.push_back(input);
  }

  for (unsigned int lidarTopicId = 0; lidarTopicId < lidarTopics.size(); lidarTopicId++)
  {
    this->CloudSubs.push_back(this->create_subscription<Pcl2_msg>(lidarTopics[lidarTopicId], 1,
                                                                                std::bind(&LidarSlamNode::ScanCallback, this, std::placeholders::_1)));
    RCLCPP_INFO_STREAM(this->get_logger(), "Using LiDAR frames on topic '" << lidarTopics[lidarTopicId] << "'");
  }

  // Set SLAM pose from external guess
  this->SetPoseSub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("set_slam_pose", 1,
                                                                                              std::bind(&LidarSlamNode::SetPoseCallback, this, std::placeholders::_1));

  // SLAM commands
  this->SlamCommandSub = this->create_subscription<lidar_slam::msg::SlamCommand>("slam_command", 1,
                                                                                            std::bind(&LidarSlamNode::SlamCommandCallback, this, std::placeholders::_1));

  // Init logging of GPS data for GPS/SLAM calibration or Pose Graph Optimization.
  // Perfect synchronization is not required as GPS data are not used in SLAM local process
  if (this->UseExtSensor[LidarSlam::GPS])
    this->GpsOdomSub = this->create_subscription<nav_msgs::msg::Odometry>("gps_odom", 1,
                                                std::bind(&LidarSlamNode::GpsCallback, this, std::placeholders::_1));

  // Init logging of landmark data and/or Camera data
  if (this->UseExtSensor[LidarSlam::LANDMARK_DETECTOR] || this->UseExtSensor[LidarSlam::CAMERA])
  {
    // Create an external independent spinner to get the landmarks and/or camera info in a parallel way
    this->ExternalSensorGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions ops;
    ops.callback_group = this->ExternalSensorGroup;

    if (this->UseExtSensor[LidarSlam::LANDMARK_DETECTOR])
    {
        this->LandmarkSub = this->create_subscription<apriltag_ros::msg::AprilTagDetectionArray>("tag_detections",
                                          200, std::bind(&LidarSlamNode::TagCallback, this, std::placeholders::_1), ops);
    }

    if (this->UseExtSensor[LidarSlam::CAMERA])
    {
      this->CameraSub = this->create_subscription<sensor_msgs::msg::Image>("camera",
                                        10, std::bind(&LidarSlamNode::ImageCallback, this, std::placeholders::_1), ops);
      this->CameraInfoSub = this->create_subscription<sensor_msgs::msg::CameraInfo>("camera_info",
                                        10, std::bind(&LidarSlamNode::CameraInfoCallback, this, std::placeholders::_1), ops);
    }
  }

  RCLCPP_INFO_STREAM(this->get_logger(), BOLD_GREEN("LiDAR SLAM is ready !"));
}

//------------------------------------------------------------------------------
LidarSlamNode::~LidarSlamNode()
{
}

//------------------------------------------------------------------------------
void LidarSlamNode::ScanCallback(const Pcl2_msg& pcl_msg)
{
  if (!this->SlamEnabled)
    return;

  CloudS::Ptr cloudS_ptr = std::make_shared<CloudS>();

  // Convert message to cloudS pointer
  pcl::fromROSMsg(pcl_msg, *cloudS_ptr);

  if(cloudS_ptr->empty())
  {
    RCLCPP_WARN(this->get_logger(), "Input point cloud sent by Lidar sensor driver is empty -> ignoring message");
    return;
  }

  // Update TF from BASE to LiDAR
  if (!this->UpdateBaseToLidarOffset(cloudS_ptr->header.frame_id, cloudS_ptr->front().device_id))
    return;

  double timeInterval = this->Frames.empty() ? 0 : (LidarSlam::Utils::PclStampToSec(cloudS_ptr->header.stamp) - LidarSlam::Utils::PclStampToSec(this->Frames[0]->header.stamp));

  // Add other frames when there is more than one lidar
  if (!this->Frames.empty())
  {
    this->Frames.push_back(cloudS_ptr);
    auto check_device = this->MultiLidarsCounter.find(cloudS_ptr->front().device_id);
    if (check_device == this->MultiLidarsCounter.end())
      this->MultiLidarsCounter.insert(std::make_pair(cloudS_ptr->front().device_id, 1));
    else
      check_device->second++;

    // Multilidar mode: drop old frame when waiting for long time
    if (this->WaitFramesDef == LidarSlamNode::FramesCollectionMode::BY_NBLIDARS && timeInterval > this->WaitFramesTime)
    {
      auto check = this->MultiLidarsCounter.find(this->Frames[0]->front().device_id);
      check->second--;
      if (check->second == 0)
        this->MultiLidarsCounter.erase(check);
      this->Frames.erase(this->Frames.begin());
    }
  }
  // Add first frame
  else
  {
    // Fill Frames with pointcloud
    this->Frames = {cloudS_ptr};
    this->MultiLidarsCounter.insert(std::make_pair(cloudS_ptr->front().device_id, 1));

    this->MainLidarId = cloudS_ptr->header.frame_id;

    this->StartTime = this->now().seconds();

    if (!this->LidarTimePosix)
    {
      // Compute time offset
      // Get ROS frame reception time
      double timeFrameReceptionPOSIX = this->now().seconds();
      // Get last acquired point timestamp
      double timeLastPoint = LidarSlam::Utils::PclStampToSec(cloudS_ptr->header.stamp) + cloudS_ptr->back().time;
      // Compute offset
      double potentialOffset = timeLastPoint - timeFrameReceptionPOSIX;
      // Get current offset
      double absCurrentOffset = std::abs(this->LidarSlam.GetSensorTimeOffset());
      // If the current computed offset is more accurate, replace it
      if (absCurrentOffset < 1e-6 || std::abs(potentialOffset) < absCurrentOffset)
        this->LidarSlam.SetSensorTimeOffset(potentialOffset + this->SensorTimeOffset);
    }
    else
      this->LidarSlam.SetSensorTimeOffset(this->SensorTimeOffset);
  }

  // Run SLAM when:
  // 1. frames are arrived from all lidar devices when frames collection mode is defined by waiting all lidar,
  // 2. time interval is greater than waiting time (0.2s) when frames collection mode is defined by waiting time
  // 3. the frame is collected when there is only one lidar
  if ((this->WaitFramesDef == LidarSlamNode::FramesCollectionMode::BY_NBLIDARS && this->MultiLidarsCounter.size() == this->MultiLidarsNb) ||
      (this->WaitFramesDef == LidarSlamNode::FramesCollectionMode::BY_TIME && this->MultiLidarsNb > 1 && timeInterval > 0.2) ||
      this->MultiLidarsNb == 1)
  {
    // Run SLAM : register new frames and update localization and map.
    this->LidarSlam.AddFrames(this->Frames);

    // Check if SLAM has failed
    if (this->LidarSlam.IsRecovery())
    {
      // TMP : in the future, the user should have a look
      // at the result to validate recovery
      // Check if the SLAM can go on and pose has to be displayed
      if (this->LidarSlam.GetOverlapEstimation() > 0.2f &&
          this->LidarSlam.GetPositionErrorStd()  < 0.1f)
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "Getting out of recovery mode");
        // Frame is relocalized, reset params
        this->LidarSlam.EndRecovery();
      }
      else
        RCLCPP_WARN_STREAM(this->get_logger(), "Still waiting for recovery");
    }
    else if (this->LidarSlam.HasFailed())
    {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "SLAM has failed : entering recovery mode :\n"
                         << "\t -Maps will not be updated\n"
                         << "\t -Egomotion and undistortion are disabled\n"
                         << "\t -The number of ICP iterations is increased\n"
                         << "\t -The maximum distance between a frame point and a map target point is increased");
      // Enable recovery mode :
      // Last frames are removed
      // Maps are not updated
      // Param are tuned to handle bigger motions
      // Warning : real time is not ensured
      this->LidarSlam.StartRecovery(this->RecoveryTime);
    }

    // Publish SLAM output as requested by user
    this->PublishOutput();

    // Reset frames vector and multi lidar counter
    this->Frames.clear();
    this->MultiLidarsCounter.clear();
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::ImageCallback(const sensor_msgs::msg::Image& imageMsg)
{
  if (!this->SlamEnabled)
    return;

  #ifdef USE_CV_BRIDGE
  if (!this->UseExtSensor[LidarSlam::CAMERA])
    return;

  // Transform to apply to points represented in GPS frame to express them in base frame
  Eigen::Isometry3d baseToCamera;
  if (Utils::Tf2LookupTransform(baseToCamera, *this->TfBuffer, this->TrackingFrameId, imageMsg.header.frame_id, imageMsg.header.stamp))
  {
    if (!this->LidarSlam.CameraCanBeUsedLocally())
      this->LidarSlam.SetCameraCalibration(baseToCamera);

    cv_bridge::CvImagePtr cvPtr;
    try
    {
      cvPtr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Camera info cannot be used -> cv_bridge exception: %s", e.what());
      return;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Adding Camera info : "<< std::fixed << std::setprecision(9) << imageMsg.header.stamp.sec + imageMsg.header.stamp.nanosec * 1e-9);
    // Add camera measurement to measurements list
    LidarSlam::ExternalSensors::Image image;
    image.Time = imageMsg.header.stamp.sec + imageMsg.header.stamp.nanosec * 1e-9;
    image.Data = cvPtr->image;
    this->LidarSlam.AddCameraImage(image);
  }
  #else
  static_cast<void>(imageMsg);
  RCLCPP_WARN_STREAM(this->get_logger(), "cv_bridge was not found so images cannot be processed, camera will not be used");
  #endif
}

//------------------------------------------------------------------------------
void LidarSlamNode::CameraInfoCallback(const sensor_msgs::msg::CameraInfo& calibMsg)
{
  if (!this->SlamEnabled)
    return;

  #ifdef USE_CV_BRIDGE
  // The intrinsic calibration must not changed so we can only use
  // the camera info until the camera is ready to be used
  if (this->LidarSlam.CameraCanBeUsedLocally())
    return;

  Eigen::Matrix3f k;
  for (int i = 0; i < 9; ++i)
    k(int(i / 3), i % 3) = calibMsg.k[i];

  this->LidarSlam.SetCameraIntrinsicCalibration(k);
  #else
  static_cast<void>(calibMsg);
  #endif
}

//------------------------------------------------------------------------------
void LidarSlamNode::GpsCallback(const nav_msgs::msg::Odometry& gpsMsg)
{
  if (!this->SlamEnabled)
    return;

    if (!this->UseExtSensor[LidarSlam::GPS])
      return;

    // Transform to apply to points represented in GPS frame to express them in base frame
    Eigen::Isometry3d baseToGps;
    if (Utils::Tf2LookupTransform(baseToGps, *this->TfBuffer, this->TrackingFrameId, gpsMsg.header.frame_id, gpsMsg.header.stamp))
    {
      RCLCPP_INFO(this->get_logger(), "Adding GPS info");
      // Get gps pose
      this->LastGpsMeas.Position = Utils::PoseMsgToIsometry(gpsMsg.pose.pose).translation();
      // Get gps timestamp
      this->LastGpsMeas.Time = gpsMsg.header.stamp.sec + gpsMsg.header.stamp.nanosec * 1e-9;

      // Get GPS covariance
      // ROS covariance message is row major
      // Eigen matrix is col major by default
      for (int i = 0; i < 3; ++i)
      {
        for (int j = 0; j < 3; ++j)
          this->LastGpsMeas.Covariance(i, j) = gpsMsg.pose.covariance[i * 6 + j];
      }
      // Correct GPS covariance if needed
      if (!LidarSlam::Utils::isCovarianceValid(this->LastGpsMeas.Covariance))
        this->LastGpsMeas.Covariance = Eigen::Matrix3d::Identity() * 4e-4; // 2cm

      if (!this->LidarSlam.GpsHasData())
        this->LidarSlam.SetGpsCalibration(baseToGps);

      // Add gps measurement to measurements list
      this->LidarSlam.AddGpsMeasurement(this->LastGpsMeas);
      this->GpsLastTime = rclcpp::Time(this->LastGpsMeas.Time * 1e9);
      this->GpsFrameId = gpsMsg.header.frame_id;
    }
    else
      RCLCPP_WARN(this->get_logger(), "The transform between the GPS and the tracking frame was not found -> GPS info ignored");
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
  if (!this->SlamEnabled)
    return;

  if (!this->UseExtSensor[LidarSlam::LANDMARK_DETECTOR])
    return;

  for (auto& tagInfo : tagsInfo.detections)
  {
    // Transform to apply to points represented in detector frame to express them in base frame
    Eigen::Isometry3d baseToLmDetector;
    if (Utils::Tf2LookupTransform(baseToLmDetector, *this->TfBuffer, this->TrackingFrameId, tagInfo.pose.header.frame_id, tagInfo.pose.header.stamp))
    {
      RCLCPP_INFO(this->get_logger(), "Adding tag info");
      LidarSlam::ExternalSensors::LandmarkMeasurement lm;
      // Get tag pose
      lm.TransfoRelative = Utils::PoseMsgToIsometry(tagInfo.pose.pose.pose);
      // Get tag timestamp
      lm.Time = tagInfo.pose.header.stamp.sec + tagInfo.pose.header.stamp.nanosec * 1e-9;

      // Get tag covariance
      // ROS covariance message is row major
      // Eigen matrix is col major by default
      for (int i = 0; i < 6; ++i)
      {
        for (int j = 0; j < 6; ++j)
          lm.Covariance(i, j) = tagInfo.pose.pose.covariance[i * 6 + j];
      }
      // Correct tag covariance if needed
      if (!LidarSlam::Utils::isCovarianceValid(lm.Covariance))
        lm.Covariance = LidarSlam::Utils::CreateDefaultCovariance(1e-2, LidarSlam::Utils::Deg2Rad(1)); // 1cm, 1°

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
        double TagTimeSec = tagInfo.pose.header.stamp.sec + tagInfo.pose.header.stamp.nanosec * 1e-9;
        this->PublishTransformTF(TagTimeSec, this->TrackingFrameId, "tag_" + std::to_string(id), lm.TransfoRelative);
      }
    }
    else
      RCLCPP_WARN(this->get_logger(), "The transform between the landmark detector and the tracking frame was not found -> landmarks info ignored");
  }
}

//------------------------------------------------------------------------------
std::vector<std::vector<std::string>> LidarSlamNode::ReadCSV(const std::string& path,
                                                             unsigned int nbFields,
                                                             unsigned int nbHeaderLines)
{
  // Check the file
  if (path.substr(path.find_last_of(".") + 1) != "csv")
  {
    RCLCPP_ERROR(this->get_logger(), "The file is not CSV! Cancel loading");
    return {};
  }

  std::ifstream lmFile(path);
  if (lmFile.fail())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "The CSV file " << path << " was not found!");
    return {};
  }

  // Check which delimiter is used
  std::string lmStr;
  std::string delimiter;
  getline (lmFile, lmStr);
  std::vector<std::string> fields;

  // Get the first data line (after header)
  for (int i = 0; i <= nbHeaderLines; ++i)
    std::getline(lmFile, lmStr);

  for (const auto& del : DELIMITERS)
  {
    fields.clear();
    size_t pos = 0;
    std::string firstLine = lmStr;
    pos = firstLine.find(del);
    while (pos != std::string::npos)
    {
      fields.push_back(firstLine.substr(0, pos));
      firstLine.erase(0, pos + del.length());
      pos = firstLine.find(del);
    }
    // If there is some element after the last delimiter, add it
    if (!firstLine.substr(0, pos).empty())
      fields.push_back(firstLine.substr(0, pos));
    // Check that the number of fields is correct
    if (fields.size() == nbFields)
    {
      delimiter = del;
      break;
    }
  }
  if (fields.size() != nbFields)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "The CSV file is ill formed : " << fields.size() << " fields were found ("
                                            << nbFields << " expected), the loading is cancelled");
    return {};
  }

  std::vector<std::vector<std::string>> lines;

  int lineIdx = nbHeaderLines;
  do
  {
    size_t pos = 0;
    std::vector<std::string> sentence;
    while ((pos = lmStr.find(delimiter)) != std::string::npos)
    {
      // Remove potential extra spaces after the delimiter
      unsigned int charIdx = 0;
      while (charIdx < lmStr.size() && lmStr[charIdx] == ' ')
        ++charIdx;
      sentence.push_back(lmStr.substr(charIdx, pos));
      lmStr.erase(0, pos + delimiter.length());
    }
    sentence.push_back(lmStr.substr(0, pos));
    if (sentence.size() != nbFields)
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "data on line " + std::to_string(lineIdx) + " of the CSV file is not correct -> Skip");
      ++lineIdx;
      continue;
    }

    // Check numerical values in the studied line
    bool numericalIssue = false;
    for (std::string field : sentence)
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
      RCLCPP_WARN_STREAM(this->get_logger(), "Data on line " + std::to_string(lineIdx) + " contains a not numerical value -> Skip");
      ++lineIdx;
      continue;
    }

    lines.push_back(sentence);
    ++lineIdx;
  }
  while (std::getline(lmFile, lmStr));

  lmFile.close();
  return lines;
}

//------------------------------------------------------------------------------
std::string LidarSlamNode::ReadPoses(const std::string& path)
{
  std::vector<std::vector<std::string>> lines = this->ReadCSV(path, 13, 2);
  if (lines.empty())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot read file :" << path << ", poses are not loaded");
    return "";
  }

  // Get frame ID
  std::ifstream lmFile(path);
  std::string frameID;
  std::getline(lmFile, frameID);
  for (const auto& del : DELIMITERS)
  {
    if (frameID.find(del) != std::string::npos)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Frame ID is not specified in the first line of the CSV file, stop loading");
      return "";
    }
  }

  for (auto& l : lines)
  {
    // Build pose measurement
    LidarSlam::ExternalSensors::PoseMeasurement poseMeas;
    // Time
    poseMeas.Time = std::stod(l[0]);
    // Translation
    for (int i = 0; i < 3; ++i)
      poseMeas.Pose(i, 3) = std::stof(l[i + 1]);
    // Rotation (format is col major to represent the orientation axis)
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
        poseMeas.Pose.linear()(i, j) = std::stof(l[i + j*3 + 4]);
    }
    this->LidarSlam.AddPoseMeasurement(poseMeas);
  }
  return frameID;
}

//------------------------------------------------------------------------------
void LidarSlamNode::ReadTags(const std::string& path)
{
  std::vector<std::vector<std::string>> lines = this->ReadCSV(path, 43, 1);
  for (auto& l : lines)
  {
    // Build measurement
    // Set landmark id
    int id = std::stoi(l[0]);
    // Fill pose
    Eigen::Vector6d absolutePose;
    absolutePose << std::stof(l[1]), std::stof(l[2]), std::stof(l[3]), std::stof(l[4]), std::stof(l[5]), std::stof(l[6]);
    // Fill covariance
    Eigen::Matrix6d absolutePoseCovariance;
    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < 6; ++j)
        absolutePoseCovariance(i, j) = std::stof(l[7 + 6 * i + j]);
    }
    // Add a new landmark manager for absolute constraint computing
    this->LidarSlam.AddLandmarkManager(id, absolutePose, absolutePoseCovariance);
    RCLCPP_INFO_STREAM(this->get_logger(), "Tag #" << id << " initialized to \n" << absolutePose.transpose());
  }
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
    RCLCPP_WARN_STREAM(this->get_logger(), "SLAM pose set to :\n" << odomToBase.matrix());
    // TODO: properly deal with covariance: rotate it, pass it to SLAM, notify trajectory jump?
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::SlamCommandCallback(const lidar_slam::msg::SlamCommand& msg)
{
  // Parse command
  switch(msg.command)
  {
    // Set SLAM pose from last received GPS pose
    // NOTE : This function should only be called after PGO or SLAM/GPS calib have been triggered.
    case lidar_slam::msg::SlamCommand::GPS_SLAM_CALIBRATION:
    {
      if (!this->UseExtSensor[LidarSlam::GPS] || !this->LidarSlam.GpsHasData())
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot set SLAM pose from GPS"
                         "Please check that 'external_sensors.gps.use_gps' private parameter is set to 'true'."
                         "and that GPS data have been received.");
        return;
      }
      this->LidarSlam.CalibrateWithGps();
      RCLCPP_WARN_STREAM(this->get_logger(), "SLAM pose set using GPS pose to :\n" << this->LidarSlam.GetLastState().Isometry.matrix());
      // Broadcast new calibration offset (GPS reference frame (i.e. generally UTM) to odom)
      this->BroadcastGpsOffset();
      break;
    }

    // Set SLAM pose from last received GPS pose
    // NOTE : This function should only be called after PGO or SLAM/GPS calib have been triggered.
    case lidar_slam::msg::SlamCommand::SET_SLAM_POSE_FROM_GPS:
    {
      if (!this->UseExtSensor[LidarSlam::GPS] || !this->LidarSlam.GpsHasData())
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot set SLAM pose from GPS"
                          "Please check that 'external_sensors.gps/use_gps' private parameter is set to 'true'."
                          "and that GPS data have been received.");
        return;
      }

      LidarSlam::ExternalSensors::GpsMeasurement& meas = this->LastGpsMeas;
      // Get position of Lidar in UTM
      Eigen::Vector3d position = this->LidarSlam.GetGpsCalibration().inverse() * meas.Position;
      // Get position of Lidar in odometry frame
      position = this->LidarSlam.GetGpsOffset() * position;
      // Orientation is supposed to be close to odometry frame
      // Warning : this hypothesis can be totally wrong and lead to bad registrations
      Eigen::Isometry3d pose = this->LidarSlam.GetLogStates().front().Isometry;
      pose.translation() = position;
      this->LidarSlam.SetWorldTransformFromGuess(pose);
      RCLCPP_WARN_STREAM(this->get_logger(), "SLAM pose set from GPS pose to :\n" << pose.matrix());
      break;
    }

    // Disable SLAM maps update
    case lidar_slam::msg::SlamCommand::DISABLE_SLAM_MAP_UPDATE:
    {
      this->LidarSlam.SetMapUpdate(LidarSlam::MappingMode::NONE);
      RCLCPP_WARN(this->get_logger(), "Disabling SLAM maps update.");
      break;
    }

    // Enable the agregation of keypoints to a fixed initial map
    case lidar_slam::msg::SlamCommand::ENABLE_SLAM_MAP_EXPANSION:
    {
      if (this->LidarSlam.IsRecovery())
        RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot unable map expansion in recovery mode!");
      this->LidarSlam.SetMapUpdate(LidarSlam::MappingMode::ADD_KPTS_TO_FIXED_MAP);
      RCLCPP_WARN(this->get_logger(), "Enabling SLAM maps expansion with new keypoints.");
      break;
    }

    // Enable the update of the map with new keypoints
    case lidar_slam::msg::SlamCommand::ENABLE_SLAM_MAP_UPDATE:
    {
      if (this->LidarSlam.IsRecovery())
        RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot unable map update in recovery mode!");
      this->LidarSlam.SetMapUpdate(LidarSlam::MappingMode::UPDATE);
      RCLCPP_WARN(this->get_logger(), "Enabling SLAM maps update with new keypoints.");
      break;
    }

    // Reset the SLAM internal state.
    case lidar_slam::msg::SlamCommand::RESET_SLAM:
    {
      RCLCPP_WARN(this->get_logger(), "Resetting the SLAM internal state.");
      this->LidarSlam.Reset(true);
      this->SetSlamInitialState();
      break;
    }

    // Enable/Disable the SLAM process
    case lidar_slam::msg::SlamCommand::SWITCH_ON_OFF:
    {
      if (this->SlamEnabled)
        RCLCPP_WARN_STREAM(this->get_logger(), "Disabling the SLAM process");
      else
        RCLCPP_WARN_STREAM(this->get_logger(), "Enabling again the SLAM process");

      this->SlamEnabled = !this->SlamEnabled;
      break;
    }
   // Save current trajectory tracking base frame
    case lidar_slam::msg::SlamCommand::SAVE_TRAJECTORY:
    {
      if (msg.string_arg.empty())
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "No path is specified, the trajectory cannot be saved");
        return;
      }
      std::list<LidarSlam::LidarState> states = this->LidarSlam.GetLogStates();
      if (states.empty())
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "No current trajectory logged, nothing will be saved");
        return;
      }
      RCLCPP_INFO_STREAM(this->get_logger(), "Saving current trajectory of base frame as " << msg.string_arg);
      std::ofstream fout(msg.string_arg);
      fout << this->TrackingFrameId << "\n";
      fout << "t,x,y,z,x0,y0,z0,x1,y1,z1,x2,y2,z2\n";
      for (auto& s : states)
        fout << s;
      fout.close();
      break;
    }

    // Save current trajectory tracking Lidar
    case lidar_slam::msg::SlamCommand::SAVE_LIDAR_TRAJECTORY:
    {
      if (msg.string_arg.empty())
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "No path is specified, the trajectory cannot be saved");
        return;
      }
      Eigen::Isometry3d baseToLidar;
      if (!Utils::Tf2LookupTransform(baseToLidar, *this->TfBuffer, this->TrackingFrameId, this->MainLidarId))
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "No transform from base to Lidar : cannot save Lidar trajectory.");
        return;
      }
      std::list<LidarSlam::LidarState> states = this->LidarSlam.GetLogStates();
      if (states.empty())
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "No current trajectory logged, nothing will be saved");
        return;
      }
      RCLCPP_INFO_STREAM(this->get_logger(), "Saving current trajectory of the Lidar sensor as " << msg.string_arg);
      std::ofstream fout(msg.string_arg);
      fout << this->MainLidarId << "\n";
      fout << "t,x,y,z,x0,y0,z0,x1,y1,z1,x2,y2,z2\n";
      for (auto& s : states)
      {
        s.Isometry = s.Isometry * baseToLidar;
        fout << s;
      }
      fout.close();
      break;
    }

    // Save SLAM keypoints maps to PCD files
    case lidar_slam::msg::SlamCommand::SAVE_KEYPOINTS_MAPS:
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Saving keypoint maps as PCD files in " << msg.string_arg);
      if (this->LidarSlam.GetMapUpdate() == LidarSlam::MappingMode::NONE)
        RCLCPP_WARN(this->get_logger(), "The initially loaded maps were not modified but are saved anyway.");
      int pcdFormatInt;
      this->get_parameter_or<int>("maps/export_pcd_format", pcdFormatInt, static_cast<int>(LidarSlam::PCDFormat::BINARY_COMPRESSED));
      LidarSlam::PCDFormat pcdFormat = static_cast<LidarSlam::PCDFormat>(pcdFormatInt);
      if (pcdFormat != LidarSlam::PCDFormat::ASCII &&
          pcdFormat != LidarSlam::PCDFormat::BINARY &&
          pcdFormat != LidarSlam::PCDFormat::BINARY_COMPRESSED)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Incorrect PCD format value (" << pcdFormat << "). Setting it to 'BINARY_COMPRESSED'.");
        pcdFormat = LidarSlam::PCDFormat::BINARY_COMPRESSED;
      }
      this->LidarSlam.SaveMapsToPCD(msg.string_arg, pcdFormat, false);
      break;
    }

    // Save SLAM keypoints submaps to PCD files
    case lidar_slam::msg::SlamCommand::SAVE_FILTERED_KEYPOINTS_MAPS:
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Saving keypoints submaps to PCD.");
      int pcdFormatInt;
      this->get_parameter_or<int>("maps/export_pcd_format", pcdFormatInt, static_cast<int>(LidarSlam::PCDFormat::BINARY_COMPRESSED));
      LidarSlam::PCDFormat pcdFormat = static_cast<LidarSlam::PCDFormat>(pcdFormatInt);
      if (pcdFormat != LidarSlam::PCDFormat::ASCII &&
          pcdFormat != LidarSlam::PCDFormat::BINARY &&
          pcdFormat != LidarSlam::PCDFormat::BINARY_COMPRESSED)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Incorrect PCD format value (" << pcdFormat << "). Setting it to 'BINARY_COMPRESSED'.");
        pcdFormat = LidarSlam::PCDFormat::BINARY_COMPRESSED;
      }
      this->LidarSlam.SaveMapsToPCD(msg.string_arg, pcdFormat, true);
      break;
    }

    // Load SLAM keypoints maps from PCD files
    case lidar_slam::msg::SlamCommand::LOAD_KEYPOINTS_MAPS:
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Loading keypoints maps from PCD.");
      this->LidarSlam.LoadMapsFromPCD(msg.string_arg);
      break;
    }

    case lidar_slam::msg::SlamCommand::OPTIMIZE_GRAPH:
    {
      if ((!this->UseExtSensor[LidarSlam::GPS] && !this->UseExtSensor[LidarSlam::LANDMARK_DETECTOR])
            || this->LidarSlam.GetSensorMaxMeasures() < 2 || this->LidarSlam.GetLoggingTimeout() < 0.2)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot optimize pose graph as sensor info logging has not been enabled. "
                         "Please make sure that 'external_sensors.landmark_detector.use_tags' OR 'external_sensors.gps/use_gps' private parameter is set to 'true', "
                         "and that 'external_sensors.landmark_detector.weight' and 'slam/logging/timeout' private parameters are set to convenient values.");
        break;
      }

      if (this->LidarSlam.LmHasData())
      {
        if (!msg.string_arg.empty())
        {
          RCLCPP_INFO_STREAM(this->get_logger(), "Loading the absolute landmark poses");
          this->ReadTags(msg.string_arg);
        }
        else if (this->LidarSlam.GetLandmarkConstraintLocal())
          RCLCPP_WARN(this->get_logger(), "No absolute landmark poses are supplied : the last estimated poses will be used");
      }

      RCLCPP_INFO_STREAM(this->get_logger(), "Optimizing the pose graph");
      this->LidarSlam.OptimizeGraph();
      // Broadcast new calibration offset (GPS to base)
      // if GPS used
      if (this->LidarSlam.GpsHasData())
        this->BroadcastGpsOffset();
      // Publish new trajectory
      if (this->Publish[PGO_PATH])
      {
        nav_msgs::msg::Path optimSlamTraj;
        optimSlamTraj.header.frame_id = this->OdometryFrameId;
        std::list<LidarSlam::LidarState> optimizedSlamStates = this->LidarSlam.GetLogStates();
        optimSlamTraj.header.stamp = rclcpp::Time(optimizedSlamStates.back().Time * 1e9);
        for (const LidarSlam::LidarState& s: optimizedSlamStates)
          optimSlamTraj.poses.emplace_back(Utils::IsometryToPoseStampedMsg(s.Isometry, s.Time, this->OdometryFrameId));
        publishWithCast(this->Publishers[PGO_PATH], nav_msgs::msg::Path, optimSlamTraj);
      }
      break;
    }

    case lidar_slam::msg::SlamCommand::SWITCH_SENSOR:
    {
      // Get sensor to enable/disable
      LidarSlam::ExternalSensor sensor;
      try
      {
        sensor = static_cast<LidarSlam::ExternalSensor>(std::stoi(msg.string_arg));
      }
      catch(...)
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "External sensor #" << msg.string_arg << " does not exist.");
        break;
      }

      std::string onOff;
      onOff = this->UseExtSensor[sensor]? "Disabling " : "Enabling ";
      RCLCPP_INFO_STREAM(this->get_logger(), onOff << LidarSlam::ExternalSensorNames.at(sensor));
      this->UseExtSensor[sensor] = !this->UseExtSensor[sensor];
      break;
    }

    case lidar_slam::msg::SlamCommand::CALIBRATE_WITH_POSES:
    {
      if (msg.string_arg.empty())
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "Cannot calibrate with poses, no file provided");
        return;
      }
      // Clear current pose manager
      this->LidarSlam.ResetSensor(true, LidarSlam::ExternalSensor::POSE);
      // Fill external pose manager with poses from a CSV file
      std::string frameId = this->ReadPoses(msg.string_arg);
      if (frameId.empty())
        return;
      // Calibrate the external poses with current SLAM trajectory
      this->LidarSlam.CalibrateWithExtPoses();
      // Get the calibration
      Eigen::Isometry3d calibration = this->LidarSlam.GetPoseCalibration();

      //Publish new static TF
      geometry_msgs::msg::TransformStamped tfStamped;
      tfStamped.header.stamp = this->now();
      tfStamped.header.frame_id = this->OdometryFrameId;
      tfStamped.child_frame_id = frameId;
      tfStamped.transform = Utils::IsometryToTfMsg(calibration);
      this->StaticTfBroadcaster->sendTransform(tfStamped);

      RCLCPP_INFO_STREAM(this->get_logger(), "Calibration estimated to :\n" << calibration.matrix());
      // Clean the pose manager in the SLAM
      this->LidarSlam.ResetSensor(true, LidarSlam::ExternalSensor::POSE);
      break;
    }

    // Unknown command
    default:
      RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown SLAM command : " << (unsigned int) msg.command);
      break;
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

//------------------------------------------------------------------------------
void LidarSlamNode::PublishOutput()
{
  // Get current SLAM poses in WORLD coordinates at the specified frequency
  std::vector<LidarSlam::LidarState> lastStates = this->LidarSlam.GetLastStates(this->TrajFrequency);
  double computationTime = (this->now().seconds() - this->StartTime);

  // Publish SLAM pose
  if (this->Publish[POSE_ODOM] || this->Publish[POSE_TF])
  {
    for (const auto& state : lastStates)
    {
      // Publish as odometry msg
      if (this->Publish[POSE_ODOM])
      {
        nav_msgs::msg::Odometry odomMsg;
        odomMsg.header.stamp = rclcpp::Time(state.Time * 1e9);
        odomMsg.header.frame_id = this->OdometryFrameId;
        odomMsg.child_frame_id = this->TrackingFrameId;
        odomMsg.pose.pose = Utils::IsometryToPoseMsg(state.Isometry);
        // Note : in eigen 3.4 iterators are available on matrices directly
        //        >> std::copy(state.Covariance.begin(), state.Covariance.end(), confidenceMsg.covariance.begin());
        // For now the only way is to copy or iterate on indices :
        for (unsigned int i = 0; i < state.Covariance.size(); ++i)
          odomMsg.pose.covariance[i] = state.Covariance(i);
        publishWithCast(this->Publishers[POSE_ODOM], nav_msgs::msg::Odometry, odomMsg);
      }

      // Publish as TF from OdometryFrameId to TrackingFrameId
      if (this->Publish[POSE_TF])
        this->PublishTransformTF(state.Time, this->OdometryFrameId, this->TrackingFrameId, state.Isometry);

      // Enable subscribers to receive those messages
      // Warning : this may alter cout working in this code area
      rclcpp::sleep_for(std::chrono::milliseconds(1));
    }
  }

  // Publish latency compensated SLAM pose
  if (this->Publish[POSE_PREDICTION_ODOM] || this->Publish[POSE_PREDICTION_TF])
  {
    double predTime = lastStates.back().Time + computationTime;
    Eigen::Isometry3d predIsometry = this->LidarSlam.GetTworld(predTime);

    // Publish as odometry msg
    if (this->Publish[POSE_PREDICTION_ODOM])
    {
      nav_msgs::msg::Odometry odomMsg;
      odomMsg.header.stamp = rclcpp::Time(predTime * 1e9);
      odomMsg.header.frame_id = this->OdometryFrameId;
      odomMsg.child_frame_id = this->TrackingFrameId + "_prediction";
      odomMsg.pose.pose = Utils::IsometryToPoseMsg(predIsometry);
      for (unsigned int i = 0; i < lastStates.back().Covariance.size(); ++i)
        odomMsg.pose.covariance[i] = lastStates.back().Covariance(i);
      publishWithCast(this->Publishers[POSE_PREDICTION_ODOM], nav_msgs::msg::Odometry, odomMsg);
    }

    // Publish as TF from OdometryFrameId to <TrackingFrameId>_prediction
    if (this->Publish[POSE_PREDICTION_TF])
      this->PublishTransformTF(predTime, this->OdometryFrameId, this->TrackingFrameId + "_prediction", predIsometry);
  }


  // Publish a pointcloud only if required and if someone is listening to it to spare bandwidth.
  // Change to publish pcl2 msgs
  // Url : https://github.com/mikeferguson/ros2_cookbook/blob/main/rclcpp/pcl.md
  #define publishPointCloud(publisher, pc)                                                  \
  if (this->Publish[publisher] && this->Publishers[publisher]->get_subscription_count())    \
    {                                                                                       \
      Pcl2_msg pcl_msg;                                                \
      pcl::toROSMsg(*pc, pcl_msg);                                                          \
      publishWithCast(this->Publishers[publisher], Pcl2_msg, pcl_msg)  \
    }

  // Keypoints maps
  publishPointCloud(EDGES_MAP,           this->LidarSlam.GetMap(LidarSlam::EDGE));
  publishPointCloud(INTENSITY_EDGES_MAP, this->LidarSlam.GetMap(LidarSlam::INTENSITY_EDGE));
  publishPointCloud(PLANES_MAP,          this->LidarSlam.GetMap(LidarSlam::PLANE));
  publishPointCloud(BLOBS_MAP,           this->LidarSlam.GetMap(LidarSlam::BLOB));

  // Keypoints submaps
  publishPointCloud(EDGES_SUBMAP,           this->LidarSlam.GetTargetSubMap(LidarSlam::EDGE));
  publishPointCloud(INTENSITY_EDGES_SUBMAP, this->LidarSlam.GetTargetSubMap(LidarSlam::INTENSITY_EDGE));
  publishPointCloud(PLANES_SUBMAP,          this->LidarSlam.GetTargetSubMap(LidarSlam::PLANE));
  publishPointCloud(BLOBS_SUBMAP,           this->LidarSlam.GetTargetSubMap(LidarSlam::BLOB));

  // Current keypoints
  publishPointCloud(EDGE_KEYPOINTS,           this->LidarSlam.GetKeypoints(LidarSlam::EDGE));
  publishPointCloud(INTENSITY_EDGE_KEYPOINTS, this->LidarSlam.GetKeypoints(LidarSlam::INTENSITY_EDGE));
  publishPointCloud(PLANE_KEYPOINTS,          this->LidarSlam.GetKeypoints(LidarSlam::PLANE));
  publishPointCloud(BLOB_KEYPOINTS,           this->LidarSlam.GetKeypoints(LidarSlam::BLOB));

  // Registered aggregated (and optionally undistorted) input scans points
  publishPointCloud(SLAM_REGISTERED_POINTS, this->LidarSlam.GetRegisteredFrame());

  // Overlap estimation
  if (this->Publish[CONFIDENCE])
  {
    // Get SLAM pose
    lidar_slam::msg::Confidence confidenceMsg;
    confidenceMsg.header.stamp = rclcpp::Time(lastStates.back().Time * 1e9);
    confidenceMsg.header.frame_id = this->OdometryFrameId;
    confidenceMsg.overlap = this->LidarSlam.GetOverlapEstimation();
    confidenceMsg.computation_time = computationTime;
    // Note : in eigen 3.4, iterators are available on matrices directly
    //        >> std::copy(lastStates.back().Covariance.begin(), lastStates.back().Covariance.end(), confidenceMsg.covariance.begin());
    for (unsigned int i = 0; i < lastStates.back().Covariance.size(); ++i)
      confidenceMsg.covariance[i] = lastStates.back().Covariance(i);

    confidenceMsg.nb_matches = this->LidarSlam.GetTotalMatchedKeypoints();
    confidenceMsg.comply_motion_limits = this->LidarSlam.GetComplyMotionLimits();
    confidenceMsg.std_position_error = this->LidarSlam.GetPositionErrorStd();
    confidenceMsg.failure = this->LidarSlam.HasFailed() || this->LidarSlam.IsRecovery();
    publishWithCast(this->Publishers[CONFIDENCE], lidar_slam::msg::Confidence, confidenceMsg);
  }
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
      RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid ego-motion mode (" << egoMotionMode << "). Setting it to 'MOTION_EXTRAPOLATION'.");
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
      RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid undistortion mode (" << undistortion << "). Setting it to 'REFINED'.");
      undistortion = LidarSlam::UndistortionMode::REFINED;
    }
    LidarSlam.SetUndistortion(undistortion);
  }
  int i_interpolationModel;
  if (this->get_parameter<int>("slam.interpolation_model", i_interpolationModel))
  {
    auto interpoModel = static_cast<LidarSlam::Interpolation::Model>(i_interpolationModel);
    if (interpoModel != LidarSlam::Interpolation::Model::LINEAR &&
        interpoModel != LidarSlam::Interpolation::Model::QUADRATIC &&
        interpoModel != LidarSlam::Interpolation::Model::CUBIC)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid interpolation model (" << interpoModel << "). Setting it to 'LINEAR'.");
      interpoModel = LidarSlam::Interpolation::Model::LINEAR;
    }
    LidarSlam.SetInterpolation(interpoModel);
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
      RCLCPP_ERROR_STREAM(this->get_logger(), "Incorrect pointcloud logging type value (" << storage << "). Setting it to 'PCL'.");
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
    SetKeypointsExtractorParam(int,   prefix + "neighbors_side_nb", MinNeighNb)
    SetKeypointsExtractorParam(float, prefix + "neighbors_radius", MinNeighRadius)
    SetKeypointsExtractorParam(float, prefix + "min_distance_to_sensor", MinDistanceToSensor)
    SetKeypointsExtractorParam(float, prefix + "max_distance_to_sensor", MaxDistanceToSensor)
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
        {RCLCPP_INFO_STREAM(this->get_logger(), "Keypoint of type " + name + " enabled");} \
      else \
        RCLCPP_INFO_STREAM(this->get_logger(), "Keypoint of type " + name + " disabled"); \
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
  this->get_parameter_or<bool>("external_sensors.lidar_is_posix", this->LidarTimePosix, true);
  SetSlamParam(float,   "external_sensors.time_offset", SensorTimeOffset)
  this->SensorTimeOffset = this->LidarSlam.GetSensorTimeOffset();
  SetSlamParam(float,  "external_sensors.landmark_detector.weight", LandmarkWeight)
  SetSlamParam(float,  "external_sensors.landmark_detector.saturation_distance", LandmarkSaturationDistance)
  SetSlamParam(bool,   "external_sensors.landmark_detector.position_only", LandmarkPositionOnly)
  this->get_parameter_or<bool>("external_sensors.landmark_detector.publish_tags", this->PublishTags, false);
  SetSlamParam(float,  "external_sensors.camera.weight", CameraWeight)
  SetSlamParam(float,  "external_sensors.camera.saturation_distance", CameraSaturationDistance)

  // Graph parameters
  SetSlamParam(std::string, "graph.g2o_file_name", G2oFileName)
  SetSlamParam(bool,        "graph.fix_first", FixFirstVertex)
  SetSlamParam(bool,        "graph.fix_last", FixLastVertex)
  SetSlamParam(float,       "graph.covariance_scale", CovarianceScale)
  SetSlamParam(int,         "graph.iterations_nb", NbGraphIterations)

  // Confidence estimators
  // Overlap
  SetSlamParam(float,  "slam.confidence.overlap.sampling_ratio", OverlapSamplingRatio)

  // Motion limitations (hard constraints to detect failure)
  std::vector<double> acc;
  if (this->get_parameter("slam.confidence.motion_limits.acceleration", acc) && acc.size() == 2)
  {
    // Convertion needed because infinity is a double in ROS
    Eigen::Array2f acc_array((float)acc[0], (float)acc[1]);
    this->LidarSlam.SetAccelerationLimits(acc_array);
  }
  std::vector<double> vel;
  if (this->get_parameter("slam.confidence.motion_limits.velocity", vel) && vel.size() == 2)
  {
    // Convertion needed because infinity is a double in ROS
    Eigen::Array2f vel_array((float)vel[0], (float)vel[1]);
    this->LidarSlam.SetVelocityLimits(vel_array);
  }
  std::vector<double> pos;
  if (this->get_parameter("slam.confidence.motion_limits.pose", pos) && pos.size() == 2)
  {
    // Convertion needed because infinity is a double in ROS
    Eigen::Array2f pos_array((float)pos[0], (float)pos[1]);
    this->LidarSlam.SetPoseLimits(pos_array);
  }

  SetSlamParam(int,   "slam.confidence.window", ConfidenceWindow)
  SetSlamParam(float, "slam.confidence.overlap.gap_threshold", OverlapDerivativeThreshold)
  SetSlamParam(float, "slam.confidence.position_error.threshold", PositionErrorThreshold)
  this->get_parameter("slam.confidence.failure_detector.recovery_time", this->RecoveryTime);
  SetSlamParam(bool,  "slam.confidence.failure_detector.enable", FailureDetectionEnabled)

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
      RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid map update mode (" << mapUpdateMode << "). Setting it to 'UPDATE'.");
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
        RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid sampling mode (" << samplingMode << ") for "
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
    RCLCPP_INFO_STREAM(this->get_logger(), "Loading initial keypoints maps from PCD.");
    this->LidarSlam.LoadMapsFromPCD(mapsPathPrefix);
  }

  // Load initial Landmarks poses if requested
  std::string lmpath = "";
  this->get_parameter("external_sensors.landmark_detector.landmarks_file_path", lmpath);
  if (!lmpath.empty())
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Loading initial landmarks info from CSV.");
    this->ReadTags(lmpath);
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

    RCLCPP_INFO_STREAM(this->get_logger(), "Setting initial SLAM pose to:\n" << poseTransform.matrix());
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::PublishTransformTF(double timeSec, std::string frameId, std::string childFrameId, const Eigen::Isometry3d& transfo)
{
  geometry_msgs::msg::TransformStamped tfMsg;
  tfMsg.header.stamp = rclcpp::Time(timeSec * 1e9);
  tfMsg.header.frame_id = frameId;
  tfMsg.child_frame_id = childFrameId;
  tfMsg.transform = Utils::IsometryToTfMsg(transfo);
  this->TfBroadcaster->sendTransform(tfMsg);
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
