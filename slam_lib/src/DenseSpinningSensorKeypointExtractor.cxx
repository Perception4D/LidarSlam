//==============================================================================
// Copyright 2022 Kitware, Inc., Kitware SAS
// Author: Jeanne Faure (Kitware SAS)
// Creation date: 2023-10-12
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

#include "LidarSlam/DenseSpinningSensorKeypointExtractor.h"
#include "LidarSlam/Utilities.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/common/common.h>

#include <random>

namespace LidarSlam
{

//-----------------------------------------------------------------------------
DenseSpinningSensorKeypointExtractor::PointCloud::Ptr DenseSpinningSensorKeypointExtractor::GetKeypoints(Keypoint k)
{
  //TODO
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::AddKeypoint(const Keypoint& k, const Point& pt)
{
  //TODO
}

//-----------------------------------------------------------------------------
std::shared_ptr<PtFeat> DenseSpinningSensorKeypointExtractor::GetPtFeat(int idxInScan) const
{
  auto& indices = this->Pc2VmIndices[idxInScan];
  return this->VertexMap[indices.Row][indices.Col];
}

//-----------------------------------------------------------------------------
int DenseSpinningSensorKeypointExtractor::GetScanLineSize(const std::vector<std::shared_ptr<PtFeat>>& scanLine)
{
  int nbPoints;
  for (auto& point : scanLine)
  {
    if (point != nullptr)
      nbPoints++;
  }
  return nbPoints;
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::InitInternalParameters()
{
  // Map laser_ids
  for (const auto& point : *this->Scan)
  {
    if (this->LaserIdMap.count(point.laser_id) == 0)
      this->LaserIdMap[point.laser_id] = this->LaserIdMap.size();
  }

  // Save the number of lasers
  this->NbLaserRings = this->LaserIdMap.size();

  bool rotationIsClockwise = Utils::IsRotationClockwise<Point>(*this->Scan, this->NbLaserRings);

  // Estimate azimuthal resolution if not already done
  // or if the previous value found is not plausible
  // (because last scan was badly formed, e.g. lack of points)
  if (this->AzimuthalResolution < 1e-6 || M_PI/4. < this->AzimuthalResolution)
    this->EstimateAzimuthalResolution();

  // Compute the indices of scan points in the future vertex map
  this->Pc2VmIndices.clear(); // Clear previous indices
  this->Pc2VmIndices.reserve(this->Scan->size());
  Eigen::Vector3f firstPt(this->Scan->at(0).data);
  for (const auto& point : *this->Scan)
  {
    double cross = firstPt.x() * point.y - firstPt.y() * point.x;
    double angle = std::atan2(cross, firstPt.head(2).dot(point.getVector3fMap().head(2)));
    float azimuth = rotationIsClockwise ? M_PI + angle : M_PI - angle;
    this->Pc2VmIndices.emplace_back(IdxVM{this->LaserIdMap.find(point.laser_id)->second,
                                          static_cast<int>(azimuth / this->AzimuthalResolution)});
  }
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::CreateVertexMap()
{
  // Clear previous Vertex Map
  for (auto& row : this->VertexMap)
  {
    for (auto& ptr : row)
      ptr.reset(); // ptr = nullptr
  }

  this->WidthVM = 2.f * M_PI / this->AzimuthalResolution + 1;
  this->HeightVM = this->NbLaserRings;

  this->VertexMap.resize(this->HeightVM, std::vector<std::shared_ptr<PtFeat>>(this->WidthVM));
  // Fill Vertex Map with index and depth values
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int i = 0; i < this->Scan->size(); ++i)
  {
    const Point& point = this->Scan->at(i);
    auto& indices = this->Pc2VmIndices[i];

    this->VertexMap[indices.Row][indices.Col] = std::make_shared<PtFeat>();
    this->VertexMap[indices.Row][indices.Col]->Index = i;
    this->VertexMap[indices.Row][indices.Col]->Depth = point.getVector3fMap().norm();
  }
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputeKeyPoints(const PointCloud::Ptr& pc)
{
  this->Scan = pc;

  this->InitInternalParameters();

  this->CreateVertexMap();
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputeCurvature()
{
  //TODO
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::AddKptsUsingCriterion(Keypoint k)
{
  //TODO
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputePlanes()
{
  //TODO
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputeEdges()
{
  //TODO
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputeIntensityEdges()
{
  //TODO
}

//-----------------------------------------------------------------------------
std::unordered_map<std::string, std::vector<float>> DenseSpinningSensorKeypointExtractor::GetDebugArray() const
{
  //TODO
}

}// end of LidarSlam namespace