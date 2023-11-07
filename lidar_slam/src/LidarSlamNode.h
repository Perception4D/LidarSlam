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
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>

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
   *  - label (uint8): optional input, not yet used.
   */
  virtual void ScanCallback(const Pcl2_msg& pcl_msg);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Optional GPS odom callback, accumulating poses.
   * @param[in] msg Converted GPS pose with its associated covariance.
   */
  void GpsCallback(const nav_msgs::msg::Odometry& msg);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Optional wheel odometer callback, accumulating poses.
   * @param[in] msg Wheel odometer distance measurement.
   */
  void WheelOdomCallback(const std_msgs::msg::Float64& msg);

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
   * @brief     Optional external pose callback, adding an external pose to the SLAM
   * @param[in] msg external pose with its associated covariance
   */
  void ExtPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& poseMsg);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Optional IMU callback, adding a gravity reference to the SLAM
   * @param[in] msg imu acceleration
   */
  void ImuCallback(const sensor_msgs::msg::Imu& imuMsg);

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
  bool UpdateBaseToLidarOffset(const std::string& lidarFrameId);

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
   * @brief Get and fill a vector of sentences (sentence = vector of strings)
   *        provided in a csv file.
   *        The delimiters can be "," ";" " " "/t"
   *        /!\ the order of fields matters
   *        A header line can be added but won't be used to fill each sentence
   */
  std::vector<std::vector<std::string>> ReadCSV(const std::string& path,
                                                unsigned int nbFields,
                                                unsigned int nbHeaderLines);

  //----------------------------------------------------------------------------
  /*!
   * @brief Helper to get and fill landmarks managers with absolute pose information
   *        provided in a csv file.
   *        The fields of the file must be : idx, x, y, z, roll, pitch, yaw, cov0, ..., cov35
   *        /!\ order matters
   */
  void ReadTags(const std::string& path);
  //----------------------------------------------------------------------------
  /*!
   * @brief Helper to get and store pose measurements
   *        provided in a csv file.
   *        The fields of the file must be : time, x, y, z, rot(0,0), rot(1,0), rot(2,0), ..., rot(3, 3)
   *        /!\ order matters
   */
  std::string ReadPoses(const std::string& path, bool resetTraj = false);
  //----------------------------------------------------------------------------
  /*!
   * @brief Helper to get and store loop closure indices
   *        provided in a csv file.
   *        The fields of the file must be : queryIdx, revisitedIdx
   *        /!\ order matters
   */
  void ReadLoopIndices(const std::string& path);

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

  // Number of Lidars connected to the SLAM process
  int MultiLidarsNb = 1;
  // To save the lidar devices id from which frames are received and count the number of frames of each lidar
  std::unordered_map<std::string, int> MultiLidarsCounter;

  // Enum to choose the frames collection mode when there is more than one lidar
  enum FramesCollectionMode
  {
    BY_TIME     = 0, // wait for a time to receive frames(0.2s)
    BY_NBLIDARS = 1  // wait until received frames from all lidar devices
  };
  FramesCollectionMode WaitFramesDef = FramesCollectionMode::BY_NBLIDARS;
  // Time to wait to receive frames
  double WaitFramesTime = 0.2;

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
  std::string MainLidarId;
  std::string GpsFrameId = "GPS";             ///< Frame to represent GPS positions.
  std::string WheelFrameId = "wheel";         ///< Frame of the wheel

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
  std::unordered_map<LidarSlam::ExternalSensor, bool> UseExtSensor = {{LidarSlam::ExternalSensor::GPS,               false},
                                                                      {LidarSlam::ExternalSensor::LANDMARK_DETECTOR, false},
                                                                      {LidarSlam::ExternalSensor::POSE,              false},
                                                                      {LidarSlam::ExternalSensor::CAMERA,            false},
                                                                      {LidarSlam::ExternalSensor::WHEEL_ODOM,        false},
                                                                      {LidarSlam::ExternalSensor::IMU,               false}};

  // Choose whether to use the time coming from the sensors' messages header
  // or the time of the message reception to synchronize the sensors
  // It is always better to use the times provided by the sensors themselves when possible.
  bool UseHeaderTime = true;

  // Offset to apply to external sensors to get lidar time
  float SensorTimeOffset = 0.;

  // Failure detector
  // In case of failure, duration (in seconds) to come back in time to previous state
  float RecoveryTime = 1.f;

  // Boolean to signal the trajectory was planar and a
  // degree of liberty is missing when looking for a calibration
  bool PlanarTrajectory = false;

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

  // External poses
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ExtPoseSub;
  std::string ExtPoseFrameId;

  // Wheel encoder
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr WheelOdomSub;

  // IMU
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ImuSub;
};

#endif // LIDAR_SLAM_NODE_H