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
 * @brief Organize angles in clusters (in order to deduce laser_id)
 * @return Clusters of vertical angles
 * @param cloudRaw PointCloud published by lidar driver
 * @param NbLasers Number of lasers of the lidar
 * @param Clusters Clusters of vertical angles to initialize
 */
template <typename PointType>
inline std::vector<Cluster> ClusteringAngles(pcl::PointCloud<PointType> cloudRaw, double NbLasers)
{
  std::vector<double> verticalAngles;
  std::vector<Cluster> Clusters(NbLasers);

  // Initialisation of the means of each cluster
  for (PointType Point : cloudRaw)
    verticalAngles.push_back((180./M_PI) * std::acos(double(Point.z) / std::sqrt(std::pow(Point.x, 2.) + std::pow(Point.y, 2.) + std::pow(Point.z, 2.))));
  double maxAngle = *std::max_element(verticalAngles.begin(), verticalAngles.end());
  double minAngle = *std::min_element(verticalAngles.begin(), verticalAngles.end());
  double clustersRange = (maxAngle - minAngle) / NbLasers;
  for (unsigned int idCluster = 0; idCluster < NbLasers; idCluster++)
    Clusters[idCluster].mean = minAngle + idCluster * clustersRange;

  std::vector<double> prevNbInliers (Clusters.size(), verticalAngles.size() / Clusters.size());
  bool notConverged = true;
  // Adjust these cluster means
  while (notConverged)
  {
    notConverged = false;
    // Fill the inliers vector of a Cluster (~ the angles are sorted in clusters)
    // Necessary hypothesis : Clusters vector is sorted by increasing mean (handled)
    for (double angle : verticalAngles)
    {
      auto insertionIt = std::upper_bound(Clusters.begin(), Clusters.end(), angle, [](double angle, const Cluster &cluster) {return angle < cluster.mean;});
      if (insertionIt == Clusters.begin())
      {
        Clusters.front().inliers.push_back(angle);
        Clusters.front().empty = false;
      }
      else if (insertionIt == Clusters.end())
      {
        Clusters.back().inliers.push_back(angle);
        Clusters.back().empty = false;
      }
      else
      {
        auto prev = std::prev(insertionIt);
        auto& minElemIt = angle - prev->mean < insertionIt->mean - angle ? prev : insertionIt;
        minElemIt->inliers.push_back(angle);
        minElemIt->empty = false;
      }
    }

    // Compute standard dev (std) of each vector of angles (inliers) from a Cluster
    for (Cluster &Cluster : Clusters)
    {
      if (!Cluster.empty)
      {
        double new_mean = std::accumulate(Cluster.inliers.begin(), Cluster.inliers.end(), 0.) / (double)Cluster.inliers.size();
        double sumSquaredSteps = 0.;
        for (double angle : Cluster.inliers)
        {
          sumSquaredSteps += std::pow(angle - new_mean, 2.0);
        }
        Cluster.std = std::sqrt(sumSquaredSteps / Cluster.inliers.size());
        Cluster.mean = new_mean;
      }
    }

    // Compare old sizes of each cluster to their new one, to see if the model tends to stabilize
    double sum = 0.0;
    for (unsigned int i = 0; i < prevNbInliers.size(); ++i)
    {
      double diff = (prevNbInliers[i] - Clusters[i].inliers.size());
      double avg = (prevNbInliers[i] + Clusters[i].inliers.size()) / 2.0;
      if (std::abs(avg) > 1e-8)
        sum += std::abs(diff / avg);
      prevNbInliers[i] = Clusters[i].inliers.size(); // Prepare the next iteration of the while loop
    }
    if (100.0 * (sum / prevNbInliers.size()) > 5.0) // We chose a 5% threshold
      notConverged = true;

    // Identify the "nan" values and change them to the following value of the cluster having the higher standard dev
    for (Cluster &ClusterCurr : Clusters)
    {
      if (ClusterCurr.empty)
      {
        Cluster &ClusterMaxStd = *std::max_element(Clusters.begin(), Clusters.end(), [](const Cluster &cluster1, const Cluster &cluster2) {return cluster1.std < cluster2.std;});
        ClusterMaxStd.std /= 2.;
        ClusterCurr.mean = ClusterMaxStd.mean + ClusterMaxStd.std;
        notConverged = true;
      }
      std::vector<double>().swap(ClusterCurr.inliers); // Empty the inliers vector to prepare the next iteration
      ClusterCurr.empty = true;
    }

    // Before iterating again, we sort the Clusters by mean since the mean of the empty ones has been changed
    std::sort(Clusters.begin(), Clusters.end(), [](const Cluster &cluster1, const Cluster &cluster2){return cluster1.mean < cluster2.mean;});
  }
  return Clusters;
}

//----------------------------------------------------------------------------
/*!
* @brief Compute laser_id of a point missing this field
* @return laser_id of the current point
* @param currentPoint Point(x,y,z) extracted from current slamPoint
* @param NbLasers Number of lasers of the lidar
* @param Clusters Clusters of vertical angles
*/
inline double ComputeLaserId(Eigen::Vector3d currentPoint, double NbLasers, std::vector<Cluster> Clusters)
{
  // Estimate laser ID thanks to a clustering of vertical angles
  double laser_id;
  double angle_v = (180./M_PI) * std::acos(double(currentPoint.z()) / currentPoint.norm());
  auto insertionIt = std::upper_bound(Clusters.begin(), Clusters.end(), angle_v, [](double angle_v, const Cluster &cluster) {return angle_v < cluster.mean;});
  if (insertionIt == Clusters.begin())
    laser_id = 0;
  else if (insertionIt == Clusters.end())
    laser_id = NbLasers - 1;
  else
  {
    auto prev = std::prev(insertionIt);
    laser_id = std::distance(Clusters.begin(), (angle_v - (*prev).mean < (*insertionIt).mean - angle_v) ? prev : insertionIt);
  }
  return laser_id;
}

//------------------------------------------------------------------------------
/*!
 * @brief Initialize estimation parameter : clusters
 * @param cloudRaw PointCloud published by lidar driver
 * @param NbLasers Number of lasers of the lidar
 * @param Clusters Clusters of vertical angles to initialize
 */
template<typename PointT>
inline void InitEstimationParameters(pcl::PointCloud<PointT>& cloudRaw, double NbLasers, std::vector<Cluster>& Clusters)
{
  Clusters = ClusteringAngles<PointT>(cloudRaw, NbLasers);
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
