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

#include <pcl_conversions/pcl_conversions.h>

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_conversions
{

GenericConversionNode::GenericConversionNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
  : Nh(nh)
  , PrivNh(priv_nh)
{
  // Get number of lasers
  this->PrivNh.param("nb_laser", this->NbLasers, this->NbLasers);

  // Get possible frequencies
  this->PrivNh.param("possible_frequencies", this->PossibleFrequencies, this->PossibleFrequencies);

  // Get number of threads
  this->PrivNh.param("nb_threads", this->NbThreads, NbThreads);

  // Init ROS publisher
  this->Talker = nh.advertise<CloudS>("lidar_points", 1);

  // Init ROS subscriber
  this->Listener = nh.subscribe("generic_points", 1, &GenericConversionNode::Callback, this);

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

  const int nbLasers = ((cloudRaw.height >= 8 && cloudRaw.height <= 128)
                   ? static_cast<int>(cloudRaw.height)
                   : (cloudRaw.width >= 8 && cloudRaw.width <= 128)
                     ? static_cast<int>(cloudRaw.width)
                     : this->NbLasers);

  // Init of parameters useful for laser_id and time estimations
  if (!this->RotSenseAndClustersEstimated)
  {
    Utils::InitEstimationParameters<PointXYZ>(cloudRaw, nbLasers, this->Clusters, this->RotationIsClockwise, this->NbThreads);
    this->RotSenseAndClustersEstimated = true;
  }
  Eigen::Vector2d firstPoint = {cloudRaw[0].x, cloudRaw[0].y};

  // Publish pointcloud only if non empty
  if (!cloudS.empty())
    this->Talker.publish(cloudS);
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
