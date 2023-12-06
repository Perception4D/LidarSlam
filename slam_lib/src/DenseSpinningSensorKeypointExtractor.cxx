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

#include <fstream>

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
#define OUTPUT_FEATURE(filename, featName, maxValue, specialValue, minValue)                                      \
{                                                                                                                 \
  int maxPix = 255;                                                                                               \
  int specialPix = 30;                                                                                            \
  std::ofstream file(filename);                                                                                   \
  file << "P2\n";                                                                                                 \
  file << this->WidthVM << " " << this->HeightVM << "\n";                                                         \
  file << maxPix << "\n";                                                                                         \
  for (int i = 0; i < this->HeightVM; ++i)                                                                        \
  {                                                                                                               \
    for (int j = 0; j < this->WidthVM; ++j)                                                                       \
    {                                                                                                             \
      if (this->VertexMap[i][j] == nullptr)                                                                       \
        file << 0 << " ";                                                                                         \
      else                                                                                                        \
      {                                                                                                           \
        float value = this->VertexMap[i][j]->featName;                                                            \
        if (std::abs(value - specialValue) < 1e-6)                                                                \
          file << specialPix << " ";                                                                              \
        else                                                                                                      \
        {                                                                                                         \
          value = std::min(static_cast<int>(value), static_cast<int>(maxValue));                                  \
          value = std::max(static_cast<int>(value), static_cast<int>(minValue));                                  \
          file << int((value - maxValue) * (maxPix - (specialPix + 1)) / (maxValue - minValue)) + maxPix << " ";  \
        }                                                                                                         \
      }                                                                                                           \
    }                                                                                                             \
    file << "\n";                                                                                                 \
  }                                                                                                               \
  file.close();                                                                                                   \
}                                                                                                                 \

void DenseSpinningSensorKeypointExtractor::OutputFeatures(std::string path)
{
  int totalSize = this->WidthVM * this->HeightVM;
  OUTPUT_FEATURE(path + "Index.pgm", Index, totalSize, 0., 0.);
  OUTPUT_FEATURE(path + "Depth.pgm", Depth, 20., 0., 0.);
  OUTPUT_FEATURE(path + "SpaceGap.pgm", SpaceGap, 1., -1., 0.);
  OUTPUT_FEATURE(path + "DepthGap.pgm", DepthGap, 10., 0., -15.);
  OUTPUT_FEATURE(path + "IntensityGap.pgm", IntensityGap, 35., -1., 0.);
  OUTPUT_FEATURE(path + "Angles.pgm", Angle, 1., 1., -1.);
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputeKeyPoints(const PointCloud::Ptr& pc)
{
  this->Scan = pc;

  this->InitInternalParameters();

  this->CreateVertexMap();

  this->ComputeCurvature();
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputeCurvature()
{
  // Init random distribution
  std::mt19937 gen(2023); // Fix seed for deterministic processes
  std::uniform_real_distribution<> dis(0.0, 1.0);

  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int i = 0; i < this->HeightVM; ++i)
  {
    // If the line is almost empty, skip it
    int nPoints = this->GetScanLineSize(this->VertexMap[i]);
    if (this->IsScanLineAlmostEmpty(nPoints))
      continue;

    // Useful index to skip first and last points of the scan line
    int idxInLine = 0;

    for (unsigned int j = 0; j < this->WidthVM; ++j)
    {
      // PtFeat struct associated with current point
      const auto& currentFeat = this->VertexMap[i][j];

      // Ignore empty pixels
      if (!currentFeat)
        continue;

      // Count every valid point in the scan line, to be compared with nPoints later
      idxInLine++;

      // Random sampling to decrease keypoints extraction
      // computation time
      if (this->InputSamplingRatio < 1.f && dis(gen) > this->InputSamplingRatio)
        continue;

      // Central point
      const Point& currentPoint = this->Scan->at(currentFeat->Index);
      const Eigen::Vector3f& centralPoint = currentPoint.getVector3fMap();

      if (!this->CheckDistanceToSensor(currentFeat->Depth))
        continue;

      if (!this->CheckAzimuthAngle(centralPoint))
        continue;

      // Fill neighbors (vectors of indices) for each side (left and right)
      // Those points must be more numerous than MinNeighNb and occupy more space than MinNeighRadius
      auto getNeighbors = [&](bool right, std::vector<int>& neighbors)
      {
        neighbors.reserve(nPoints);
        neighbors.emplace_back(currentFeat->Index);
        int rightOrLeft = right ? 1 : -1;
        int idxNeigh = 1;
        float lineLength = 0.f;
        while ((lineLength < this->MinNeighRadius ||
               int(neighbors.size()) < this->MinNeighNb) &&
               int(neighbors.size()) < nPoints)
        {
          const auto& ptrFeat = this->VertexMap[i][(j + rightOrLeft * idxNeigh + this->WidthVM) % this->WidthVM];
          if (ptrFeat != nullptr)
          {
            neighbors.emplace_back(ptrFeat->Index);
            if (lineLength < MinNeighRadius)
              lineLength = (this->Scan->at(neighbors.back()).getVector3fMap() - this->Scan->at(neighbors.front()).getVector3fMap()).norm();
          }
          ++idxNeigh;
        }
        neighbors.shrink_to_fit();
      };

      std::vector<int> leftNeighbors, rightNeighbors;
      getNeighbors(false, leftNeighbors);
      getNeighbors(true, rightNeighbors);

      if (this->Enabled[EDGE])
      {
        // ---- Compute horizontal space gap ----

        // Find the first empty neighbor on the right and on the left
        auto& idxRightNeigh = this->Pc2VmIndices[rightNeighbors[1]];
        int nbEmptyRightNeigh = idxRightNeigh.Col - j;

        auto& idxLeftNeigh = this->Pc2VmIndices[leftNeighbors[1]];
        int nbEmptyLeftNeigh = j - idxLeftNeigh.Col;

        const float cosMinBeamSurfaceAngle = std::cos(Utils::Deg2Rad(this->MinBeamSurfaceAngle));

        float distRight = -1.f;
        float distLeft = -1.f;

        if (nbEmptyRightNeigh >= this->EdgeNbGapPoints)
        {
          const auto& rightPt = this->Scan->at(rightNeighbors[1]).getVector3fMap();
          float diffRightNorm = (rightPt - centralPoint).norm();
          float cosBeamLineAngleRight = std::abs((rightPt - centralPoint).dot(centralPoint) / (diffRightNorm * currentFeat->Depth));
          if (cosBeamLineAngleRight < cosMinBeamSurfaceAngle)
            distRight = diffRightNorm;
        }
        if (nbEmptyLeftNeigh >= this->EdgeNbGapPoints)
        {
          const auto& leftPt = this->Scan->at(leftNeighbors[1]).getVector3fMap();
          float diffLeftNorm = (leftPt - centralPoint).norm();
          float cosBeamLineAngleLeft = std::abs((leftPt - centralPoint).dot(centralPoint) / (diffLeftNorm * currentFeat->Depth));
          if (cosBeamLineAngleLeft < cosMinBeamSurfaceAngle)
            distLeft = diffLeftNorm;
        }
        currentFeat->SpaceGap = std::max(distLeft, distRight);
      }

      // Stop search for first and last points of the scan line
      // because the discontinuity may alter the other criteria detection
      if (idxInLine < int(leftNeighbors.size()) || idxInLine >= nPoints - int(rightNeighbors.size()))
        continue;

      if (this->Enabled[EDGE])
      {
        // ---- Compute horizontal depth gap ----

        int idxNext = this->VertexMap[i][j + 1] ? j + 1 : this->VertexMap[i][j + 2] ? j + 2 : -1;
        int idxPrev = this->VertexMap[i][j - 1] ? j - 1 : this->VertexMap[i][j - 2] ? j - 2 : -1;
        if (idxNext > 0 && idxPrev > 0)
        {
          auto& directRightNeigh = this->VertexMap[i][idxNext];
          float distRight = directRightNeigh->Depth - currentFeat->Depth;
          auto& directLeftNeigh = this->VertexMap[i][idxPrev];
          float distLeft = directLeftNeigh->Depth - currentFeat->Depth;

          auto& postRightNeigh = this->VertexMap[i][idxNext + 1];
          auto& preLeftNeigh = this->VertexMap[i][idxPrev - 1];
          if (postRightNeigh != nullptr)
          {
            float distPostRight = postRightNeigh->Depth - directRightNeigh->Depth;
            if (distRight < distPostRight)
              distRight = 0.0f;
          }
          if (preLeftNeigh != nullptr)
          {
            float distPreLeft = preLeftNeigh->Depth - directLeftNeigh->Depth;
            if (distLeft < distPreLeft)
              distLeft = 0.0f;
          }
          currentFeat->DepthGap = std::abs(distLeft) > std::abs(distRight) ? distLeft : distRight;
        }
      }

      if (currentFeat->SpaceGap > this->EdgeDepthGapThreshold || currentFeat->DepthGap > this->EdgeDepthGapThreshold)
        continue;

      // ---- Compute intensity gap ----

      LineFitting leftLine, rightLine;
      // Fit line on the left and right neighborhoods and
      // skip point if they are not usable
      if (!leftLine.FitLineAndCheckConsistency(*this->Scan, leftNeighbors) ||
          !rightLine.FitLineAndCheckConsistency(*this->Scan, rightNeighbors))
        continue;

      if (!this->IsBeamAngleValid(centralPoint, currentFeat->Depth, rightLine) ||
          !this->IsBeamAngleValid(centralPoint, currentFeat->Depth, leftLine))
        continue;

      if (this->Enabled[INTENSITY_EDGE])
      {
        if (std::abs(this->Scan->at(rightNeighbors[1]).intensity - this->Scan->at(leftNeighbors[1]).intensity) > this->EdgeIntensityGapThreshold)
        {
          // Compute mean intensity on the left
          // We sample neighborhoods for computation time concerns
          float meanIntensityLeft = 0.f;
          int step = leftNeighbors.size() > this->MinNeighNb ? leftNeighbors.size() / this->MinNeighNb : 1;
          int cptMean = 0;
          // The first element of the neighborhood is the central point itself so we skip it
          for (int i = 1; i < leftNeighbors.size(); i += step)
          {
            meanIntensityLeft += this->Scan->at(leftNeighbors[i]).intensity;
            cptMean++;
          }
          meanIntensityLeft /= cptMean;
          // Compute mean intensity on the right
          float meanIntensityRight = 0.f;
          step = rightNeighbors.size() > this->MinNeighNb ? rightNeighbors.size() / this->MinNeighNb : 1;
          cptMean = 0;
          for (int i = 1; i < rightNeighbors.size(); i += step)
          {
            meanIntensityRight += this->Scan->at(rightNeighbors[i]).intensity;
            cptMean++;
          }
          meanIntensityRight /= cptMean;
          currentFeat->IntensityGap = std::abs(meanIntensityLeft - meanIntensityRight);

          // Remove neighbor points to get the best intensity discontinuity locally
          auto neighPtr = this->GetPtFeat(leftNeighbors[1]);
          if (neighPtr->IntensityGap < currentFeat->IntensityGap)
            neighPtr->IntensityGap = -1;
          else
            currentFeat->IntensityGap = -1;
        }
      }

      // ---- Compute angle ----

      if (this->Enabled[PLANE] || this->Enabled[EDGE])
      {
        // Compute angles
        currentFeat->Angle = leftLine.Direction.dot(rightLine.Direction);
        // Remove angles too small to be edges
        if (currentFeat->Angle > -this->PlaneCosAngleThreshold)
        {
          currentFeat->Angle = 1.f;
          continue;
        }

        // Remove previous point from angle inspection if the angle is not maximal locally
        if (this->Enabled[EDGE] && currentFeat->Angle > this->EdgeCosAngleThreshold)
        {
          // Check previously computed angle to keep only the maximal angle keypoint locally
          for (int idx = 1; idx < leftNeighbors.size(); idx++)
          {
            const std::shared_ptr<PtFeat>& neighPtr = this->GetPtFeat(leftNeighbors[idx]);
            if (neighPtr->Angle > this->EdgeCosAngleThreshold &&
                neighPtr->Angle < -this->PlaneCosAngleThreshold)
            {
              if (neighPtr->Angle > currentFeat->Angle)
                currentFeat->Angle = 1.f;
              else
                neighPtr->Angle = 1.f;
              break;
            }
          }
        }
      }
    }
  }
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