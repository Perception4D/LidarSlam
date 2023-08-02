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
#include <rclcpp/publisher_base.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lidar_slam/msg/confidence.hpp>
#include <lidar_slam/msg/slam_command.hpp>
#include <apriltag_ros/msg/april_tag_detection.hpp>
#include <apriltag_ros/msg/april_tag_detection_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

// SLAM
#include <LidarSlam/Slam.h>

#define PRINT_VERBOSE(minVerbosityLevel, stream) if (this->LidarSlam.GetVerbosity() >= (minVerbosityLevel)) {std::cout << stream << std::endl;}

class LidarSlamNode : public rclcpp::Node
{
public:

  using PointS = LidarSlam::Slam::Point;
  using CloudS = pcl::PointCloud<PointS>;
  using Pcl2_msg = sensor_msgs::msg::PointCloud2;

  //----------------------------------------------------------------------------
  /*!
   * @brief     Constructor. LidarSlamNode is directly the node
   * @param[in] name_node Name of the node, used to init publisher/subscribers and log messages
   * @param[in] options Options of the node, default no options
   */
  LidarSlamNode(std::string name_node = "lidar_slam",
                const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  //----------------------------------------------------------------------------
  /*!
   * @brief     Destructor.
   * Used to shut down external spinners
   */
  ~LidarSlamNode();

  //----------------------------------------------------------------------------
  /*!
   * @brief     New main LiDAR frame callback, running SLAM and publishing TF.
   * @param[in] cloud New frame message, published by conversion node.
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
  virtual void ScanCallback(const Pcl2_msg& pcl_msg);

  //----------------------------------------------------------------------------
  /*!
   * @brief     New secondary lidar frame callback, buffered to be latered processed by SLAM.
   * @param[in] cloud New frame message, published by conversion node.
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
  virtual void SecondaryScanCallback(const Pcl2_msg& pcl_msg);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Optional GPS odom callback, accumulating poses.
   * @param[in] msg Converted GPS pose with its associated covariance.
   */
  void GpsCallback(const nav_msgs::msg::Odometry& msg);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Optional tag detection callback, adding a landmark relative pose to the SLAM
   * @param[in] msg april tag node output message
   */
  void TagCallback(const apriltag_ros::msg::AprilTagDetectionArray& tagInfo);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Optional RGB image callback, adding 2D color features to SLAM
   * @param[in] msg compressed RGB image
   */
  void ImageCallback(const sensor_msgs::msg::Image& imageMsg);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Optional image info callback, when using RGB camera into SLAM
   * @param[in] msg camera calibration
   */
  void CameraInfoCallback(const sensor_msgs::msg::CameraInfo& calibMsg);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Set SLAM pose from external guess.
   * @param[in] msg The pose to use.
   *
   * NOTE: A valid TF tree must link msg.header.frame_id to OdometryFrameId.
   * NOTE: The covariance is not used yet.
   */
  void SetPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& msg);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Receive an external command to process, such as pose graph
   *            optimization, GPS/SLAM calibration, set SLAM pose, save maps etc.
   * @param[in] msg The command message.
   */
  void SlamCommandCallback(const lidar_slam::msg::SlamCommand& msg);

protected:

  //----------------------------------------------------------------------------
  /*!
   * @brief     Update transform offset between BASE and LIDAR using TF2
   * @param[in] lidarFrameId The input LiDAR pointcloud frame_id.
   * @param[in] lidarDeviceId The numerical identifier of the LiDAR sensor.
   */
  bool UpdateBaseToLidarOffset(const std::string& lidarFrameId, uint8_t lidarDeviceId);

  //----------------------------------------------------------------------------
  /*!
   * @brief Publish SLAM outputs as requested by user.
   *
   * It is possible to send :
   *  - pose and covariance as Odometry msg or TF
   *  - extracted keypoints from current frame
   *  - keypoints maps
   *  - undistorted input points registered in odometry frame
   */
  void PublishOutput();

  //----------------------------------------------------------------------------
  /*!
   * @brief Get and fill Slam parameters from ROS parameters server.
   */
  void SetSlamParameters();

  //----------------------------------------------------------------------------
  /*!
   * @brief Fill the SLAM initial state with the given initial maps, pose and
   *        landmarks.
   */
  void SetSlamInitialState();

  //----------------------------------------------------------------------------
  /*!
   * @brief Get and fill landmarks managers with absolute pose information
   *        provided by the user in a csv file.
   *        The fields must be : idx, x, y, z, roll, pitch, yaw, cov0, ..., cov35
   *        The delimiters can be "," ";" " " "/t"
   *        /!\ order matters
   *        A header line can be added
   */
  void LoadLandmarks(const std::string& path);

  //----------------------------------------------------------------------------
  /*!
   * @brief Build an id for the april tag output message
   *        if it gives the info of one landmark, the id is the one of this landMark
   *        if it gives the info of a tag bundle, the id is built as [idN [...] id1 id0]
   */
  int BuildId(const std::vector<int>& ids);

  //----------------------------------------------------------------------------
  // Publish static tf to link world (UTM) frame to SLAM origin
  // PGO must have been run, so we can average
  // the correspondant poses (GPS/LidarSLAM) distances to get the offset
  void BroadcastGpsOffset();

  //----------------------------------------------------------------------------
  /*!
   * @brief Publish a Tf2 static transform
   *        Use the StaticTfBroadcaster of the node
   *
   * @param[in] timeSec The time in second
   * @param[in] frameId The frame Id
   * @param[in] childFrameId The child frame Id
   * @param[in] transfo The transformation matrix
   */
  void PublishTransformTF(double timeSec, std::string frameId, std::string childFrameId, const Eigen::Isometry3d& transfo);


  //----------------------------------------------------------------------------

  // SLAM stuff
  LidarSlam::Slam LidarSlam;
  std::vector<CloudS::Ptr> Frames;
  bool SlamEnabled = true;

  // ROS subscribers and publishers
  std::vector<rclcpp::Subscription<Pcl2_msg>::SharedPtr> CloudSubs;
  rclcpp::Subscription<lidar_slam::msg::SlamCommand>::SharedPtr SlamCommandSub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr SetPoseSub;
  std::unordered_map<int, rclcpp::PublisherBase::SharedPtr> Publishers;
  std::unordered_map<int, bool> Publish;

  // Output pose required frequency (Hz)
  double TrajFrequency = -1;

  // Start time (which corresponds to master Lidar scan reception)
  // It is stored to get the process time and be able to compensate the motion if required
  double StartTime = 0.;

  // TF stuff
  std::string OdometryFrameId = "odom";       ///< Frame in which SLAM odometry and maps are expressed.
  std::string TrackingFrameId = "base_link";  ///< Frame to track (ensure a valid TF tree is published).
  std::string GpsFrameId = "GPS"; ///< Frame to represent GPS positions.
  rclcpp::Time GpsLastTime;
  std::unique_ptr<tf2_ros::Buffer> TfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> TfListener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> TfBroadcaster;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> StaticTfBroadcaster;

  // External sensors
  // Callback group separate from default one to receive  messages from external sensors in parallel
  rclcpp::CallbackGroup::SharedPtr ExternalSensorGroup;

  // Booleans to select which sensor to activate
  // If sensor enabled, data are received and stored
  // External sensor data can be used in local optimization or in postprocess pose graph optimization
  std::unordered_map<LidarSlam::ExternalSensor, bool> UseExtSensor = {{LidarSlam::GPS, false},
                                                                      {LidarSlam::LANDMARK_DETECTOR, false},
                                                                      {LidarSlam::POSE, false},
                                                                      {LidarSlam::CAMERA, false}};

  // If lidar time contained in the header is not POSIX
  // The offset between network reception time
  // and Lidar time is computed
  bool LidarTimePosix = true;
  // Offset to apply to external sensors to get lidar time
  float SensorTimeOffset = 0.;

  // Failure detector
  // In case of failure, duration (in seconds) to come back in time to previous state
  float RecoveryTime = 1.f;

  // Landmarks
  rclcpp::Subscription<apriltag_ros::msg::AprilTagDetectionArray>::SharedPtr LandmarkSub;
  bool PublishTags = false;

  // GPS
  Eigen::Isometry3d BaseToGpsOffset = Eigen::Isometry3d::Identity();  ///< Pose of the GPS antenna in BASE coordinates.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr GpsOdomSub;
  LidarSlam::ExternalSensors::GpsMeasurement LastGpsMeas;

  // Camera
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr CameraSub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr CameraInfoSub;
};

#endif // LIDAR_SLAM_NODE_H