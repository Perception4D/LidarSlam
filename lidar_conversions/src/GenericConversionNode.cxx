//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Faure Jeanne (Kitware SAS)
// Creation date: 2023-08-31
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

#include "GenericConversionNode.h"
#include "GenericPoint.h"
#include <chrono>

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_conversions
{

GenericConversionNode::GenericConversionNode(std::string node_name, const rclcpp::NodeOptions options)
    : Node(node_name, options)
{
  // Get number of lasers
  this->get_parameter("nb_lasers", this->NbLasers);

  // Get possible frequencies
  this->get_parameter("possible_frequencies", this->PossibleFrequencies);

  // Get number of threads
  this->get_parameter("nb_threads", this->NbThreads);

  // Init ROS publisher
  this->Talker = this->create_publisher<Pcl2_msg>("lidar_points", 1);

  // Init ROS subscriber
  this->Listener = this->create_subscription<Pcl2_msg>("/generic_points", 1,
                                        std::bind(&GenericConversionNode::Callback, this, std::placeholders::_1));

  // Init ROS service
  this->EstimService = this->create_service<lidar_conversions::srv::EstimParams>(
      "lidar_conversions/estim_params",
      std::bind(&GenericConversionNode::EstimParamsService, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO_STREAM(this->get_logger(), BOLD_GREEN("Generic LiDAR data converter is ready !"));
}

//------------------------------------------------------------------------------
void GenericConversionNode::Callback(const Pcl2_msg& msg_received)
{
  // DEBUG: init time for all callback : to be used only if other timers are off
  // auto start = std::chrono::high_resolution_clock::now();

  CloudXYZ cloudRaw = Utils::InitCloudRaw<CloudXYZ>(msg_received);

  // If input cloud is empty, ignore it
  if (cloudRaw.empty())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Input pointcloud is empty : frame ignored.");
    return;
  }

  // Fill the map of device_id if the device hasn't already been attributed one
  if (this->DeviceIdMap.count(cloudRaw.header.frame_id) == 0)
    this->DeviceIdMap[cloudRaw.header.frame_id] = (uint8_t)(this->DeviceIdMap.size());

  // Rotation duration is estimated to be used in time estimation if needed
  double currFrameTime = Utils::PclStampToSec(cloudRaw.header.stamp);
  double diffTimePrevFrame = currFrameTime - this->PrevFrameTime;
  this->PrevFrameTime = currFrameTime;

  // If the rotation duration has not been estimated
  if (this->RotationDuration < 0.)
  {
    // Check if this duration is possible
    if (Utils::CheckRotationDuration(diffTimePrevFrame, this->PossibleFrequencies))
    {
      // Check a confirmation of the frame duration to avoid outliers (frames dropped)
      // For the first frame, RotationDurationPrior is -1, this condition won't be fulfilled
      // For the second frame, RotationDurationPrior is absurd, this condition won't be fulfilled
      // First real intempt occurs at the 3rd frame
      if (std::abs(diffTimePrevFrame - this->RotationDurationPrior) < 5e-3) // 5ms threshold
        this->RotationDuration = (diffTimePrevFrame + this->RotationDurationPrior) / 2.;
      this->RotationDurationPrior = diffTimePrevFrame;
      RCLCPP_INFO_STREAM(this->get_logger(), std::setprecision(12) << "Difference between successive frames is :" << diffTimePrevFrame);
    }
  }

  if (this->RotationDuration < 0.)
    return;

  CloudS cloudS = Utils::InitCloudS<CloudXYZ>(cloudRaw);

  const int nbLasers = ((cloudRaw.height >= 8 && cloudRaw.height <= 128)
                   ? static_cast<double>(cloudRaw.height)
                   : (cloudRaw.width >= 8 && cloudRaw.width <= 128)
                     ? static_cast<double>(cloudRaw.width)
                     : this->NbLasers);

  // Init of parameters useful for laser_id and time estimations
  if (!this->RotSenseAndClustersEstimated)
  {
    Utils::InitEstimationParameters<PointXYZ>(cloudRaw, nbLasers, this->Clusters, this->RotationIsClockwise, this->NbThreads);
    this->RotSenseAndClustersEstimated = true;
  }
  Eigen::Vector2d firstPoint = {cloudRaw[0].x, cloudRaw[0].y};

  // Initialize vectors of optional fields
  std::vector<float> intensities;
  std::vector<std::uint16_t> laser_ids;
  std::vector<double> times;

  // Macro to fill arrays of optional fields
  #define FILL_ARRAY(pointType, field, array) \
  {  \
    pcl::PointCloud<pointType> cloud = Utils::InitCloudRaw<pcl::PointCloud<pointType>>(msg_received); \
    array.reserve(cloud.size()); \
    for (const pointType& point : cloud) \
    { \
        array.emplace_back(point.field); \
    } \
  }

  // Macro to handle the 8 different types for each name of field (see use below)
  #define FIND_TYPE_AND_FILL_ARRAY(pointSuffix, fieldName, array) \
  do { \
    switch (field.datatype) { \
        case pcl::PCLPointField::INT8: { \
            FILL_ARRAY(point_conversions::Point##pointSuffix##_Int8, fieldName, array); \
            break; \
        } \
        case pcl::PCLPointField::UINT8: { \
            FILL_ARRAY(point_conversions::Point##pointSuffix##_Uint8, fieldName, array); \
            break; \
        } \
        case pcl::PCLPointField::INT16: { \
            FILL_ARRAY(point_conversions::Point##pointSuffix##_Int16, fieldName, array); \
            break; \
        } \
        case pcl::PCLPointField::UINT16: { \
            FILL_ARRAY(point_conversions::Point##pointSuffix##_Uint16, fieldName, array); \
            break; \
        } \
        case pcl::PCLPointField::INT32: { \
            FILL_ARRAY(point_conversions::Point##pointSuffix##_Int32, fieldName, array); \
            break; \
        } \
        case pcl::PCLPointField::UINT32: { \
            FILL_ARRAY(point_conversions::Point##pointSuffix##_Uint32, fieldName, array); \
            break; \
        } \
        case pcl::PCLPointField::FLOAT32: { \
            FILL_ARRAY(point_conversions::Point##pointSuffix##_Float, fieldName, array); \
            break; \
        } \
        case pcl::PCLPointField::FLOAT64: { \
            FILL_ARRAY(point_conversions::Point##pointSuffix##_Double, fieldName, array); \
            break; \
        } \
        default: { \
            RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown datatype for field " << field.name); \
            break; \
        } \
    } \
  } while(0)

  // DEBUG: init time for filling vectors of fields values
  // WARNING: if you want to use this timer to know how long it will take to fill
  // intensities AND times AND laser_ids, you need to change the field names in the tests
  auto start_vectors = std::chrono::high_resolution_clock::now();

  // Explore optional fields (if field is x, y or z : ignore it)
  // #pragma omp parallel for num_threads(this->NbThreads)
  for (const sensor_msgs::msg::PointField& field : msg_received.fields)
  {
    if (field.name == "x" || field.name == "y" || field.name == "z")
      continue;
    // Intensity SlamPoint field
    else if (field.name == "intensity")
    // DEBUG: to print time of a SWITCH_AND_FILL_FIELD only
    // {
    //   auto start_macro = std::chrono::high_resolution_clock::now();
      FIND_TYPE_AND_FILL_ARRAY(I, intensity, intensities);
    //   auto end_macro = std::chrono::high_resolution_clock::now();
    //   std::chrono::duration<double> elapsed_macro = end_macro - start_macro;
    //   std::cout << elapsed_macro.count() << std::endl;
    // }
    else if (field.name == "reflectivity")
      FIND_TYPE_AND_FILL_ARRAY(Ref, reflectivity, intensities);
    // Laser_id SlamPoint field
    //DEBUG: change to "laser_id" if you want to test the time of filling the 3 vectors
    else if (field.name == "TEST_LASER_ID_NOT_FOUND")
      FIND_TYPE_AND_FILL_ARRAY(Id, laser_id, laser_ids);
    //DEBUG: change to "ring" if you want to test the time of filling the 3 vectors
    else if (field.name == "TEST_RING_NOT_FOUND")
      FIND_TYPE_AND_FILL_ARRAY(Ring, ring, laser_ids);
    // Time SlamPoint field
    //DEBUG: change to "time" if you want to test the time of filling the 3 vectors
    else if (field.name == "TEST_TIME_NOT_FOUND")
      FIND_TYPE_AND_FILL_ARRAY(Time, time, times);
    //DEBUG: change to "t" if you want to test the time of filling the 3 vectors
    else if (field.name == "TEST_T_NOT_FOUND")
      FIND_TYPE_AND_FILL_ARRAY(T, t, times);
    // else
    //   RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown field name : " << field.name);
  }

  //DEBUG: print time of filling vectors of fields values
  // auto end_vectors = std::chrono::high_resolution_clock::now();
  // std::chrono::duration<double> elapsed_vectors = end_vectors - start_vectors;

  // Check if time field looks properly set
  bool timeIsValid = false;
  if (!times.empty())
  {
    double duration = times.back() - times.front();
    double factor = Utils::GetTimeFactor(duration, this->RotationDuration);
    timeIsValid = duration > 1e-8 && duration < 2. * this->RotationDuration;
    if (!timeIsValid)
      RCLCPP_WARN_STREAM(this->get_logger(), "Invalid 'time' field, it will be built from azimuth advancement.");
  }

  uint8_t deviceId = this->DeviceIdMap[cloudRaw.header.frame_id];

  // DEBUG: init time measure for the whole slamcloud building
  // auto start_slamcloud = std::chrono::high_resolution_clock::now();

  // Build SLAM pointcloud
  //#pragma omp parallel for num_threads(this->NbThreads)
  for (unsigned int i = 0; i < cloudRaw.size(); ++i)
  {
    PointXYZ& rawPoint = cloudRaw[i];

    // Remove no return points by checking unvalid values (NaNs or zeros)
    if (!Utils::IsPointValid(rawPoint))
      continue;

    if(!cloudS.empty() && std::equal(rawPoint.data, rawPoint.data + 3, cloudS.back().data))
      continue;

    // Copy space coordinates and add other slamPoint fields from previously filled vectors or computations
    PointS slamPoint;

    slamPoint.x = rawPoint.x;
    slamPoint.y = rawPoint.y;
    slamPoint.z = rawPoint.z;
    slamPoint.device_id = deviceId;
    slamPoint.intensity = intensities.empty() ? 0. : intensities[i];

    slamPoint.laser_id = laser_ids.empty() ? Utils::ComputeLaserId({slamPoint.x, slamPoint.y, slamPoint.z}, nbLasers, this->Clusters) : laser_ids[i];

    slamPoint.time = (times.empty() || !timeIsValid)? Utils::EstimateTime({slamPoint.x, slamPoint.y}, this->RotationDuration, firstPoint, this->RotationIsClockwise) : times[i];

    cloudS.push_back(slamPoint);
  }
  // WARNING: each timer to be used seperately, by commenting all other timers to avoid side effects
  //DEBUG: print time of the whole slamcloud building
  // auto end_slamcloud = std::chrono::high_resolution_clock::now();
  // std::chrono::duration<double> elapsed_slamcloud = end_slamcloud - start_slamcloud;
  // std::cout << elapsed_slamcloud.count() << std::endl;  // std::cout << elapsed_slamcloud.count() << std::endl;

  //DEBUG: mesure total time of the callback
  // auto end = std::chrono::high_resolution_clock::now();;
  // std::chrono::duration<double> elapsed_all = end - start;
  // std::cout << elapsed_all.count() << std::endl;

  PublishMsg(cloudS);
}

//------------------------------------------------------------------------------
void GenericConversionNode::EstimParamsService(
  const std::shared_ptr<lidar_conversions::srv::EstimParams::Request> req,
  const std::shared_ptr<lidar_conversions::srv::EstimParams::Response> res)
{
  this->RotSenseAndClustersEstimated = false;
  RCLCPP_INFO_STREAM(this->get_logger(), "Estimation parameters will be re-estimated with next frames.");
  res->success = true;
}

} // end of namespace lidar_conversions

//------------------------------------------------------------------------------
/*!
 * @brief Main node entry point.
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Create options for the node to use undeclared parameters
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.allow_undeclared_parameters(true);

  std::shared_ptr<lidar_conversions::GenericConversionNode> raw2s
    = std::make_shared<lidar_conversions::GenericConversionNode>("generic_conversion", options);

  rclcpp::spin(raw2s);

  return 0;
}