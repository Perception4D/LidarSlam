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

#include "VelodyneToLidarNode.h"
#include <pcl_conversions/pcl_conversions.h>

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_conversions
{

VelodyneToLidarNode::VelodyneToLidarNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
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
  this->Listener = nh.subscribe("velodyne_points", 1, &VelodyneToLidarNode::Callback, this);

  // Init ROS service
  this->EstimService = nh.advertiseService("lidar_conversions/estim_sense", &VelodyneToLidarNode::EstimSenseService, this);

  ROS_INFO_STREAM(BOLD_GREEN("Velodyne data converter is ready !"));
}

//------------------------------------------------------------------------------
void VelodyneToLidarNode::Callback(const CloudV& cloudV)
{
  // If input cloud is empty, ignore it
  if (cloudV.empty())
  {
    ROS_ERROR_STREAM("Input Velodyne pointcloud is empty : frame ignored.");
    return;
  }

  // Rotation duration is estimated to be used in time estimation if needed
  double currFrameTime = Utils::PclStampToSec(cloudV.header.stamp);
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
    this->RotationDuration = 0.1;

  // Init SLAM pointcloud
  CloudS cloudS = Utils::InitCloudS<CloudV>(cloudV);

  const unsigned int nbLasers = (cloudV.height >= 8 && cloudV.height <= 128) ?
                                 cloudV.height :
                                  (cloudV.width >= 8 && cloudV.width <= 128) ?
                                   cloudV.width :
                                   this->NbLasers;

  // Estimate the rotation sense
  if (!this->RotationSenseEstimated)
  {
    this->RotationIsClockwise = Utils::IsRotationClockwise<PointV>(cloudV, nbLasers);
    this->RotationSenseEstimated = true;
  }

  // Check if time field looks properly set
  double duration = cloudV.back().time - cloudV.front().time;
  double factor = Utils::GetTimeFactor(duration, this->RotationDuration);

  bool isTimeValid = duration > 1e-8 && duration < 2. * this->RotationDuration;

  if (!isTimeValid)
    ROS_WARN_STREAM("Invalid 'time' field, it will be built from azimuth advancement.");

  Eigen::Vector2d firstPoint = {cloudV[0].x, cloudV[0].y};

  // Build SLAM pointcloud
  #pragma omp parallel for num_threads(this->NbThreads)
  for (const PointV& velodynePoint : cloudV)
  {
    // Remove no return points by checking unvalid values (NaNs or zeros)
    if (!Utils::IsPointValid(velodynePoint))
      continue;

    PointS slamPoint;
    slamPoint.x = velodynePoint.x;
    slamPoint.y = velodynePoint.y;
    slamPoint.z = velodynePoint.z;
    slamPoint.intensity = velodynePoint.intensity;
    slamPoint.laser_id = velodynePoint.ring;

    // Use time field if available, else estimate it from azimuth advancement
    if (isTimeValid)
      slamPoint.time = factor * velodynePoint.time;
    else
      slamPoint.time = Utils::EstimateTime({slamPoint.x, slamPoint.y}, this->RotationDuration, firstPoint, this->RotationIsClockwise);

    if (!Utils::HasNanField(slamPoint))
      cloudS.push_back(slamPoint);
  }

  this->Talker.publish(cloudS);
}

//------------------------------------------------------------------------------
bool VelodyneToLidarNode::EstimSenseService(lidar_conversions::EstimSenseRequest& req, lidar_conversions::EstimSenseResponse& res)
{
  this->RotationSenseEstimated = false;
  ROS_INFO_STREAM("Rotation sense will be re-estimated with next frames.");
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
  ros::init(argc, argv, "velodyne_conversion");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  lidar_conversions::VelodyneToLidarNode v2s(n, priv_nh);

  ros::spin();

  return 0;
}
