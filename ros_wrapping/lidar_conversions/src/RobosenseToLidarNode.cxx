//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
//         Cadart Nicolas (Kitware SAS)
// Creation date: 2020-12-22
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

#include "RobosenseToLidarNode.h"
#include <pcl_conversions/pcl_conversions.h>

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_conversions
{
namespace
{
  // Mapping between RSLidar laser id and vertical laser id
  // TODO add laser ID mappings for RS32, RSBPEARL and RSBPEARL_MINI ?
  const std::array<uint16_t, 16> LASER_ID_MAPPING_RS16 = {0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8};
}

RobosenseToLidarNode::RobosenseToLidarNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
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
  this->Listener = nh.subscribe("rslidar_points", 1, &RobosenseToLidarNode::Callback, this);

  // Init ROS service
  this->EstimService = nh.advertiseService("lidar_conversions/estim_params", &RobosenseToLidarNode::EstimParamsService, this);

  ROS_INFO_STREAM(BOLD_GREEN("RSLidar data converter is ready !"));
}

//------------------------------------------------------------------------------
void RobosenseToLidarNode::Callback(const CloudRS& cloudRS)
{
  // If input cloud is empty, ignore it
  if (cloudRS.empty())
  {
    ROS_ERROR_STREAM("Input RSLidar pointcloud is empty : frame ignored.");
    return;
  }

  // Rotation duration is estimated to be used in time estimation if needed
  double currFrameTime = Utils::PclStampToSec(cloudRS.header.stamp);
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

  // Init SLAM pointcloud
  CloudS cloudS = Utils::InitCloudS<CloudRS>(cloudRS);

  const unsigned int nbLasers = (cloudRS.height >= 8 && cloudRS.height <= 128) ?
                                 cloudRS.height :
                                  (cloudRS.width >= 8 && cloudRS.width <= 128) ?
                                   cloudRS.width :
                                   this->NbLasers;

  // Init of parameters useful for laser_id and time estimations
  if (!this->RotSenseAndClustersEstimated)
  {
    Utils::InitEstimationParameters<PointRS>(cloudRS, nbLasers, this->Clusters, this->RotationIsClockwise, this->NbThreads);
    this->RotSenseAndClustersEstimated = true;
  }

  Eigen::Vector2d firstPoint = {cloudRS[0].x, cloudRS[0].y};

  // Build SLAM pointcloud
  #pragma omp parallel for num_threads(this->NbThreads)
  for (unsigned int i = 0; i < cloudRS.size(); ++i)
  {
    const PointRS& rsPoint = cloudRS[i];

    // Remove no return points by checking unvalid values (NaNs or zeros)
    if (!Utils::IsPointValid(rsPoint))
      continue;

    // In case of dual returns mode, check that the second return is not identical to the first
    // CHECK this operation for other sensors than RS16
    if (!cloudS.empty() && std::equal(rsPoint.data, rsPoint.data + 3, cloudS.back().data))
      continue;

    // Copy coordinates and intensity
    PointS slamPoint;
    slamPoint.x = rsPoint.x;
    slamPoint.y = rsPoint.y;
    slamPoint.z = rsPoint.z;
    slamPoint.intensity = rsPoint.intensity;
    slamPoint.laser_id = Utils::ComputeLaserId({slamPoint.x, slamPoint.y, slamPoint.z}, nbLasers, this->Clusters);
    slamPoint.time = Utils::EstimateTime({slamPoint.x, slamPoint.y}, this->RotationDuration, firstPoint, this->RotationIsClockwise);

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
bool RobosenseToLidarNode::EstimParamsService(lidar_conversions::EstimParamsRequest& req, lidar_conversions::EstimParamsResponse& res)
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
  ros::init(argc, argv, "rslidar_conversion");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  lidar_conversions::RobosenseToLidarNode rs2s(n, priv_nh);

  ros::spin();

  return 0;
}
