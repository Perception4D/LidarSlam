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
  double mean = 0.;
  double std = 0.;
  bool empty = true;
  std::vector<double> inliers;
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
CloudRawType InitCloudRaw(const sensor_msgs::msg::PointCloud2& msg_received)
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
 * @param point A PCL point, with possible NaN as coordinates or null.
 * @return true if all point coordinates are valid, false otherwise.
 */
template<typename PointT>
inline bool IsPointValid(const PointT& point)
{
  bool isZeroPoint = point.getVector3fMap().norm() < 1e-6;
  bool isFinite = std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
  bool isValid = isFinite && !isZeroPoint;
  return isValid;
}

//------------------------------------------------------------------------------
/*!
 * @brief Check if Rpm has a likely value
 * @return boolean to know if the RPM is correct
 * @param rpm RPM of the lidar
 */
inline bool CheckRpm(double rpm, std::vector<double> possibleFrequencies)
{
  // We assume that if the user hasn't specified a list of possible frequencies,
  // it's because he doesn't want to check the RPM or doesn't care about outliers
  if (possibleFrequencies.empty())
    return true;

  for (double frequency : possibleFrequencies)
  {
    double rpmFreq = frequency * 60.;
    double epsilon = 0.05 * rpmFreq; // We accept 5% error
    if (frequency - epsilon < rpm && rpm < frequency + epsilon)
      return true;
  }
  return false;
}

//------------------------------------------------------------------------------
/*!
 * @brief Estimate the number of rotations per minute of the lidar
 * @return RPM of the lidar
 * @param currentTimestamp Timestamp of current frame
 * @param previousTimestamp Timestamp of previous frame
 * @param previousRpm RPM of the lidar computed from all previous frames
 */
inline double EstimateRpm(double currentTimeStamp, double& previousTimeStamp, double& previousRpm, std::vector<double> possibleFrequencies)
{
  if (previousTimeStamp < 0.)
  {
    previousTimeStamp = currentTimeStamp;
    return -1.; // Indicates that it's the first frame
  }
  else if (previousRpm < 0.)
  {
    double rpmFirst = 1. / ((currentTimeStamp - previousTimeStamp) / (1e6 * 60.)); // /1e6 to convert micros to s, /60 to convert s to min
    if (!CheckRpm(rpmFirst, possibleFrequencies))
      return -1.; // This RPM is unusable so we'll wait next frame
    previousRpm = rpmFirst;
    previousTimeStamp = currentTimeStamp;
    return rpmFirst; // Return the initial RPM (computed frame 2)
  }
  else
  {
    double rpmCurr = 1. / ((currentTimeStamp - previousTimeStamp) / (1e6 * 60.));
    if (!CheckRpm(rpmCurr, possibleFrequencies))
      return previousRpm; // Return the previous RPM
    double averagedRpm = (rpmCurr + previousRpm) / 2.;
    previousRpm = averagedRpm;
    return averagedRpm; // Return the estimated RPM
  }
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
