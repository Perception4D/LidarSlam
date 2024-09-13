//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
//         Fu Tong (Kitware SAS)
// Creation date: 2023-11-03
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

#include <pcl_conversions/pcl_conversions.h>

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_conversions
{

GenericConversionNode::GenericConversionNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
  : Nh(nh)
  , PrivNh(priv_nh)
{
  // Get number of lasers
  int nbLasers = 16;
  if (this->PrivNh.getParam("nb_laser", nbLasers))
    this->NbLasers = static_cast<unsigned int>(nbLasers);

  // Get possible frequencies
  this->PrivNh.param("possible_frequencies", this->PossibleFrequencies, this->PossibleFrequencies);

  // Get number of threads
  this->PrivNh.param("nb_threads", this->NbThreads, this->NbThreads);

  // Init ROS publisher
  this->Talker = nh.advertise<CloudS>("lidar_points", 1);

  // Init ROS subscriber
  this->Listener = nh.subscribe("generic_points", 1, &GenericConversionNode::Callback, this);

  // Init ROS service
  this->EstimService = nh.advertiseService("lidar_conversions/estim_params", &GenericConversionNode::EstimParamsService, this);

  ROS_INFO_STREAM(BOLD_GREEN("Generic LiDAR data converter is ready !"));
}

//------------------------------------------------------------------------------
void GenericConversionNode::Callback(const sensor_msgs::PointCloud2& msg_received)
{
  CloudXYZ cloudRaw = Utils::InitCloudRaw<CloudXYZ>(msg_received);
  // If input cloud is empty, ignore it
  if (cloudRaw.empty())
  {
    ROS_ERROR_STREAM("Input pointcloud is empty : frame ignored.");
    return;
  }

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
      ROS_INFO_STREAM(std::setprecision(12) << "Difference between successive frames is :" << diffTimePrevFrame);
    }
  }

  if (this->RotationDuration < 0.)
    return;

  CloudS cloudS = Utils::InitCloudS<CloudXYZ>(cloudRaw);

  const unsigned int nbLasers = (cloudRaw.height >= 8 && cloudRaw.height <= 128) ?
                                 cloudRaw.height :
                                   (cloudRaw.width >= 8 && cloudRaw.width <= 128) ?
                                    cloudRaw.width :
                                    this->NbLasers;

  // Init of parameters useful for laser_id and time estimations
  if (!this->RotSenseAndClustersEstimated)
  {
    Utils::InitEstimationParameters<PointXYZ>(cloudRaw, nbLasers, this->Clusters, this->RotationIsClockwise, this->NbThreads);
    this->RotSenseAndClustersEstimated = true;
  }
  Eigen::Vector2d firstPoint = Utils::GetFirstValidPoint(cloudRaw);

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
            ROS_ERROR_STREAM("Unknown datatype for field " << field.name); \
            break; \
        } \
    } \
  } while(0)

  // Explore optional fields
  #pragma omp parallel for num_threads(this->NbThreads)
  for (const sensor_msgs::PointField& field : msg_received.fields)
  {
    if (field.name == "x" || field.name == "y" || field.name == "z")
      continue;
    // Intensity SlamPoint field
    else if (field.name == "intensity")
      FIND_TYPE_AND_FILL_ARRAY(I, intensity, intensities);
    else if (field.name == "reflectivity")
      FIND_TYPE_AND_FILL_ARRAY(Ref, reflectivity, intensities);
    // Laser_id SlamPoint field
    else if (field.name == "laser_id")
      FIND_TYPE_AND_FILL_ARRAY(Id, laser_id, laser_ids);
    else if (field.name == "ring")
      FIND_TYPE_AND_FILL_ARRAY(Ring, ring, laser_ids);
    // Time SlamPoint field
    else if (field.name == "time")
      FIND_TYPE_AND_FILL_ARRAY(Time, time, times);
    else if (field.name == "t")
      FIND_TYPE_AND_FILL_ARRAY(T, t, times);
    else
      ROS_WARN_STREAM("Unknown field name : " << field.name);
  }

  // Check if time field looks properly set
  bool timeIsValid = false;
  if (!times.empty())
  {
    auto minmaxTime = std::minmax_element(times.begin(), times.end());
    double duration = *minmaxTime.second - *minmaxTime.first;
    double factor = Utils::GetTimeFactor(duration, this->RotationDuration);
    duration *= factor;
    timeIsValid = duration > 1e-8 && duration < 2. * this->RotationDuration;
    if (!timeIsValid)
      ROS_WARN_STREAM("Invalid 'time' field, it will be built from azimuth advancement.");
  }

  // Build SLAM pointcloud
  #pragma omp parallel for num_threads(this->NbThreads)
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
    slamPoint.intensity = intensities.empty() ? 0. : intensities[i];
    slamPoint.laser_id = laser_ids.empty() ? Utils::ComputeLaserId({slamPoint.x, slamPoint.y, slamPoint.z}, nbLasers, this->Clusters) : laser_ids[i];
    slamPoint.time = (times.empty() || !timeIsValid) ? Utils::EstimateTime({slamPoint.x, slamPoint.y}, this->RotationDuration, firstPoint, this->RotationIsClockwise) : times[i];

    if (!Utils::HasNanField(slamPoint))
      cloudS.push_back(slamPoint);
  }

  // Publish pointcloud only if non empty
  if (cloudS.empty())
  {
    ROS_ERROR_STREAM("Slam pointcloud is empty : frame ignored.");
    return;
  }
  this->Talker.publish(cloudS);
}

//------------------------------------------------------------------------------
bool GenericConversionNode::EstimParamsService(lidar_conversions::EstimParamsRequest& req, lidar_conversions::EstimParamsResponse& res)
{
  this->RotSenseAndClustersEstimated = false;
  ROS_INFO_STREAM("Estimation parameters will be re-estimated with next frames.");
  res.success = true;
  return true;
}

}  // end of namespace lidar_conversions

//------------------------------------------------------------------------------
/*!
 * @brief Main node entry point.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "generic_conversion");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  lidar_conversions::GenericConversionNode raw2s(n, priv_nh);

  ros::spin();

  return 0;
}
