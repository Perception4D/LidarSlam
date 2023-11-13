//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
// Creation date: 2020-12-10
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

#include <pcl/point_cloud.h>
#include <cmath>
#include <map>

#include <LidarSlam/LidarPoint.h>

namespace lidar_conversions
{
namespace Utils
{

struct Cluster
{
  double Mean = 0.;
  double Std = 0.;
  bool Empty = true;
  std::vector<double> Inliers;
};

//------------------------------------------------------------------------------
/*!
 * @brief Copy pointcloud metadata to an other cloud
 * @param[in] from The pointcloud to copy info from
 * @param[out] to The pointcloud to copy info to
 */
template<typename PointI, typename PointO>
inline void CopyPointCloudMetadata(const pcl::PointCloud<PointI>& from, pcl::PointCloud<PointO>& to)
{
  to.header = from.header;
  to.is_dense = from.is_dense;
  to.sensor_orientation_ = from.sensor_orientation_;
  to.sensor_origin_ = from.sensor_origin_;
}

//----------------------------------------------------------------------------
/*!
  * @brief Initialize the PointCloud published by lidar driver
  * @param msg_received New Lidar Frame, published by lidar_pointcloud/cloud_node.
  */
template<typename CloudRawType>
CloudRawType InitCloudRaw(const sensor_msgs::PointCloud2& msg_received)
{
  CloudRawType cloudRaw;
  pcl::fromROSMsg(msg_received, cloudRaw);
  return cloudRaw;
}

//------------------------------------------------------------------------------
/*!
  * @brief Initialize the PointCloud needed by SLAM
  * @param cloudRaw PointCloud published by lidar driver
  */
template<typename CloudRawType>
pcl::PointCloud<LidarSlam::LidarPoint> InitCloudS(CloudRawType cloudRaw)
{
  // Init SLAM pointcloud
  pcl::PointCloud<LidarSlam::LidarPoint> cloudS;
  cloudS.reserve(cloudRaw.size());

  // Copy pointcloud metadata
  Utils::CopyPointCloudMetadata(cloudRaw, cloudS);
  cloudS.is_dense = true;

  return cloudS;
}

//------------------------------------------------------------------------------
/*!
 * @brief Check if a PCL point is valid
 * @param point A PCL point, with possible unvalid coordinates.
 * @return true if point coordinates are finite and
 *         the point distance is in range [1e-6, 1000], false otherwise.
 */
template<typename PointT>
inline bool IsPointValid(const PointT& point)
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
    return false;

  float distance = point.getVector3fMap().norm();
  return distance > 1e-6 && distance < 1000.;
}

//------------------------------------------------------------------------------
/*!
 * @brief Check if a lidarPoint is valid
 * @param point A lidarpoint, with possible NaN as field values.
 * @return true if a field value is NaN.
 */
inline bool HasNanField(const LidarSlam::LidarPoint& point)
{
  return !std::isfinite(point.x)         ||
         !std::isfinite(point.y)         ||
         !std::isfinite(point.z)         ||
         !std::isfinite(point.time)      ||
         !std::isfinite(point.intensity) ||
         !std::isfinite(point.laser_id)  ||
         !std::isfinite(point.label);
}

//------------------------------------------------------------------------------
/*!
 * @brief Check if the rotation duration has a likely value
 * @return boolean to know if the rotation duration is correct
 * @param rotationDuration RotationDuration of the lidar
 */
inline bool CheckRotationDuration(double rotationDuration, std::vector<double> possibleFrequencies)
{
  // We assume that if the user hasn't specified a list of possible frequencies,
  // it's because he doesn't want to check the rotation duration or doesn't care about outliers
  if (possibleFrequencies.empty())
    return true;

  for (double frequency : possibleFrequencies)
  {
    double currentFreq = 1. / rotationDuration;
    double epsilon = 0.05 * frequency; // We accept 5% error
    if (frequency - epsilon < currentFreq && currentFreq < frequency + epsilon)
      return true;
  }
  return false;
}

//------------------------------------------------------------------------------
/*!
 * @brief Convert PCL timestamp (in microseconds) to seconds
 * @param pclStampUs PCL timestamp, in microseconds
 * @return Timestamp in seconds
 */
inline constexpr double PclStampToSec(uint64_t pclStampUs)
{
  return pclStampUs * 1e-6;
}

//------------------------------------------------------------------------------
/*!
 * @brief Get the factor between points time and frame time
 * @return factor to scale point time to frame time : frame.Time = point.Time * factor
 * @param duration difference between first point and last point of frame
 * @param rotationDuration difference between one frame and the following one
 */
inline double GetTimeFactor(double duration, double rotationDuration)
{
  int power = std::round(std::log10(rotationDuration)) - std::round(std::log10(duration));
  return std::pow(10, power);
}


//----------------------------------------------------------------------------
/*!
 * @brief Deduce the rotation sense of the lidar
 * @return true if the LiDAR rotates clockwise, false otherwise.
 * @param cloudRaw PointCloud published by lidar driver
 * @param nbLasers Number of lasers of the lidar
 */
template <typename PointType>
inline bool IsRotationClockwise(const pcl::PointCloud<PointType> cloudRaw, int nbLasers)
{
  Eigen::Vector2d firstPointFirstLine ({cloudRaw.front().x, cloudRaw.front().y});
  Eigen::Vector2d firstPointSecondLine ({cloudRaw[nbLasers].x, cloudRaw[nbLasers].y});
  double crossZ = firstPointFirstLine.x() * firstPointSecondLine.y() - firstPointFirstLine.y() * firstPointSecondLine.x();
  return crossZ > 0;
}

//----------------------------------------------------------------------------
/*!
 * @brief Estimate time of a point missing this field
 * @return time of the current point
 * @param currentPoint Point(x,y) extracted from current slamPoint
 * @param rotationDuration Time for a full rotation of the lidar
 * @param firstPoint First point of the frame
 * @param rotationIsClockwise True if the LiDAR rotates clockwise, false otherwise.
 */
inline double EstimateTime(const Eigen::Vector2d& currentPoint, double rotationDuration, const Eigen::Vector2d& firstPoint, bool rotationIsClockwise = true)
{
  double angle_h = std::acos(firstPoint.normalized().dot(currentPoint.normalized()));
  double crossZ = firstPoint.x() * currentPoint.y() - firstPoint.y() * currentPoint.x();
  if ((crossZ <= 0 && rotationIsClockwise) || (crossZ > 0 && !rotationIsClockwise))
    angle_h = 2. * M_PI - angle_h;
  return ((angle_h / (2.*M_PI)) - 1) * rotationDuration;
}

//------------------------------------------------------------------------------
/*!
 * @struct Helper to estimate point-wise within frame advancement for a spinning
 * lidar sensor using azimuth angle.
 */
struct SpinningFrameAdvancementEstimator
{
  /*!
   * @brief Reset the estimator. This should be called when a new frame is received.
   */
  void Reset()
  {
    this->PreviousAdvancementPerRing.clear();
  }

  /*!
   * @brief Estimate point advancement within current frame using azimuth angle.
   * @param point the point to estimate its relative advancement. It MUST have a 'laser_id' field.
   * @return relative advancement in range [0;~1].
   *
   * This computation is based on azimuth angle of each measured point.
   * The first point will return a 0 advancement.
   * Advancement will increase clock-wise, reaching 1 when azimuth angle reaches
   * initial azimuth value. It may be greater than 1 if the frame spins more
   * than 360 degrees.
   *
   * NOTE: this estimation uses 2 priors:
   * - real azimuth is always strictly inceasing within each laser ring,
   * - real azimuth angle of any first point of a laser ring should be greater than initial azimuth.
   */
  template<typename PointT>
  double operator()(const PointT& point)
  {
    // Compute normalized azimuth angle (in range [0-1])
    double pointAdvancement = (M_PI - std::atan2(point.y, point.x)) / (2 * M_PI);

    // If this is the first point of the frame
    if (this->PreviousAdvancementPerRing.empty())
      this->InitAdvancement = pointAdvancement;

    // Get normalized angle (in [0-1]), with angle 0 being first point direction
    auto wrapMax = [](double x, double max) { return std::fmod(max + std::fmod(x, max), max); };
    double frameAdvancement = wrapMax(pointAdvancement - this->InitAdvancement, 1.);

    // If we detect overflow, correct it
    // If current laser_id is not in map, the following line will insert it,
    // associating it to value 0.0.
    if (frameAdvancement < this->PreviousAdvancementPerRing[point.laser_id])
      frameAdvancement += 1.;
    this->PreviousAdvancementPerRing[point.laser_id] = frameAdvancement;

    return frameAdvancement;
  }

private:
  double InitAdvancement;
  std::map<int, double> PreviousAdvancementPerRing;
};

}  // end of namespace Utils
}  // end of namespace lidar_conversions
