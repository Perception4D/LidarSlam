//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Guilbert Pierre (Kitware SAS)
//         Cadart Nicolas (Kitware SAS)
// Creation date: 2019-12-13
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

#include "LidarSlam/RollingGrid.h"
#include "LidarSlam/Utilities.h"

#include <pcl/common/common.h>
#include <unordered_set>

namespace LidarSlam
{

//==============================================================================
//   Initialization and parameters setters
//==============================================================================

//------------------------------------------------------------------------------
RollingGrid::RollingGrid(const Eigen::Vector3f& position)
{
  this->SubMap.reset(new PointCloud);
  this->GridInSize = int(this->VoxelResolution / this->LeafSize);
  this->VoxelWidth = this->GridInSize * this->LeafSize;
  this->Reset(position);
}

//------------------------------------------------------------------------------
void RollingGrid::Reset(const Eigen::Vector3f& position)
{
  // Clear/reset empty voxel grid
  this->Clear();

  // Initialize VoxelGrid center position
  // Position is rounded down to be a multiple of resolution
  this->VoxelGridPosition = (position.array() / this->VoxelWidth).floor() * this->VoxelWidth;
}

//------------------------------------------------------------------------------
void RollingGrid::Clear()
{
  this->NbPoints = 0;
  this->Voxels.clear();
  this->SubMap->clear();
  this->KdTree.Reset();
}

//------------------------------------------------------------------------------
void RollingGrid::SetGridSize(int size)
{
  // Resize voxel grid
  this->GridSize = size;

  // Clear current voxel grid and fill it back with points so that they now lie
  // in the right voxel
  PointCloud::Ptr prevMap = this->Get();
  this->Clear();
  if (!prevMap->empty())
    this->Add(prevMap);
}

//------------------------------------------------------------------------------
void RollingGrid::SetVoxelResolution(double resolution)
{
  // We cannot compel the leaf size (inner voxel grid resolution) with the
  // outer voxel grid resolution if we want equally sized inner voxels.
  // Therefore, the outer voxel resolution (VoxelWidth) might be a bit lower
  // than supplied (VoxelResolution) to get to the closest multiple of the leaf size
  this->VoxelResolution = resolution;
  this->GridInSize = int(resolution / this->LeafSize);
  this->VoxelWidth = this->GridInSize * this->LeafSize;

  // Round down VoxelGrid center position to be a multiple of resolution
  this->VoxelGridPosition = (this->VoxelGridPosition / this->VoxelWidth).floor() * this->VoxelWidth;

  // Move points so that they now lie in the right voxel
  PointCloud::Ptr prevMap = this->Get();
  this->Clear();
  if (!prevMap->empty())
    this->Add(prevMap);
}

//------------------------------------------------------------------------------
void RollingGrid::SetLeafSize(double ls)
{
  this->LeafSize = ls;

  // Update relative values
  // Reminder : the outer voxel resolution (VoxelWidth) might be a bit lower
  // than supplied (VoxelResolution) to get to the closest multiple of the leaf size
  this->GridInSize = int(this->VoxelResolution / this->LeafSize);
  this->VoxelWidth = this->GridInSize * this->LeafSize;
  this->VoxelGridPosition = (this->VoxelGridPosition / this->VoxelWidth).floor() * this->VoxelWidth;
}

//==============================================================================
//   Main use
//==============================================================================

//------------------------------------------------------------------------------
RollingGrid::PointCloud::Ptr RollingGrid::Get(bool clean) const
{
  // Merge all points into a single pointcloud
  PointCloud::Ptr pc(new PointCloud);
  pc->reserve(this->NbPoints);
  // Loop on the outer voxels (rolling vg)
  for (const auto& kvOut : this->Voxels)
  {
    // Loop on the inner voxels (sampling vg)
    for (const auto& kvIn : kvOut.second)
    {
      // If all points can be used or if the point
      // does not lie in a moving object, extract it.
      if (!clean || kvIn.second.count >= this->MinFramesPerVoxel)
        pc->emplace_back(kvIn.second.point);
    }
  }

  return pc;
}

//------------------------------------------------------------------------------
void RollingGrid::Roll(const Eigen::Array3f& minPoint, const Eigen::Array3f& maxPoint)
{
  // Very basic implementation where the grid is not circular.
  // This only moves VoxelGrid so that the given bounding box can entirely fit in rolled map.

  // Compute how much the new frame does not fit in current grid
  double halfGridSize = static_cast<double>(this->GridSize) / 2 * this->VoxelWidth;
  Eigen::Array3f downOffset = minPoint - (this->VoxelGridPosition - halfGridSize);
  Eigen::Array3f upOffset   = maxPoint - (this->VoxelGridPosition + halfGridSize);
  Eigen::Array3f offset = (upOffset + downOffset) / 2;

  // Clamp the rolling movement so that it only moves what is really necessary
  offset = offset.max(downOffset.min(0.f)).min(upOffset.max(0.f));
  Eigen::Array3i voxelsOffset = (offset / this->VoxelWidth).round().cast<int>();

  // Exit if there is no need to roll
  if ((voxelsOffset == 0).all())
    return;

  // Fill new voxel grid
  unsigned int newNbPoints = 0;
  RollingVG newVoxels;
  for (const auto& kvOut : this->Voxels)
  {
    // Compute new voxel position
    Eigen::Array3i newIdx3d = this->To3d(kvOut.first, this->GridSize) - voxelsOffset;

    // Move voxel and keep it only if it lies within bounds
    if (((0 <= newIdx3d) && (newIdx3d < this->GridSize)).all())
    {
      int newIdx1d = this->To1d(newIdx3d, this->GridSize);
      newNbPoints += kvOut.second.size();
      newVoxels[newIdx1d] = std::move(kvOut.second);
    }
  }

  // Update the voxel grid
  this->NbPoints = newNbPoints;
  this->Voxels.swap(newVoxels);
  this->VoxelGridPosition += voxelsOffset.cast<float>() * this->VoxelWidth;
}

//------------------------------------------------------------------------------
void RollingGrid::Add(const PointCloud::Ptr& pointcloud, bool fixed, bool roll)
{
  if (pointcloud->empty())
  {
    PRINT_WARNING("Pointcloud is empty, voxel grid not updated.");
    return;
  }

  // Optionally roll the map so that all new points can fit in rolled map
  if (roll)
  {
    Eigen::Vector4f minPoint, maxPoint;
    pcl::getMinMax3D(*pointcloud, minPoint, maxPoint);
    this->Roll(minPoint.head<3>().cast<float>().array(), maxPoint.head<3>().cast<float>().array());
  }

  // Compute the 3D position of the center of the first voxel
  Eigen::Array3f voxelGridOrigin = this->VoxelGridPosition - int(this->GridSize / 2) * this->VoxelWidth;

  // Boolean grid to check if a voxel has already been reached by another
  // added point to decide whether to update the count attribute or not
  std::unordered_map<int, std::unordered_map<int, bool>> seen;
  // Voxels' states info (for CENTROID sampling mode) :
  // mean point of current added points in each voxel
  std::unordered_map<int, std::unordered_map<int, Voxel>> meanPts;
  // Boolean to check if the tree will need update
  bool updated = false;
  // Add points in the rolling grid
  for (const Point& point : *pointcloud)
  {
    // Find the outer voxel containing this point
    Eigen::Array3i voxelCoordOut = Utils::PositionToVoxel<Eigen::Array3f>(point.getArray3fMap(), voxelGridOrigin, this->VoxelWidth);

    // Add point to grid if it is indeed within bounds
    if (((0 <= voxelCoordOut) && (voxelCoordOut < this->GridSize)).all())
    {
      // Compute the position of the center of the inner voxel grid (=sampling voxel grid)
      // which is the center of the outer voxel (from the rolling voxel grid)
      Eigen::Array3f voxelGridCenterIn = voxelCoordOut.cast<float>() * this->VoxelWidth + voxelGridOrigin;
      // Find the inner voxel containing this point (from the sampling vg)
      Eigen::Array3i voxelCoordIn = Utils::PositionToVoxel<Eigen::Array3f>(point.getArray3fMap(), voxelGridCenterIn, this->LeafSize);
      unsigned int idxOut = this->To1d(voxelCoordOut, this->GridSize);
      unsigned int idxIn = this->To1d(voxelCoordIn, this->GridInSize);
      // If the outer voxel or the inner voxel are empty, add new point
      if (!this->Voxels.count(idxOut) ||
          !this->Voxels[idxOut].count(idxIn))
      {
        this->Voxels[idxOut][idxIn].point = point;
        ++this->NbPoints;
        // Notify that the voxel point has been updated
        updated = true;
      }
      else
      {
        // Shortcut to voxel
        auto& voxel = this->Voxels[idxOut][idxIn];

        // Check if the voxel contains a fixed point
        if (voxel.point.label == static_cast<int>(LidarSlam::LidarPointLabel::FIXED))
          continue;

        // If the point is not fixed, force label with new point label
        voxel.point.label = point.label;

        switch(this->Sampling)
        {
          // If first mode enabled,
          // use the first acquired keypoint in the voxel
          case SamplingMode::FIRST:
          {
            // keep the previous point
            break;
          }
          // If last mode enabled,
          // use the last acquired keypoint in the voxel
          case SamplingMode::LAST:
          {
            // Update the point
            voxel.point = point;
            // Notify that the voxel point has been updated
            updated = true;
            break;
          }
          // If max_intensity mode enabled,
          // keep the keypoint with maximum intensity
          case SamplingMode::MAX_INTENSITY:
          {
            if (point.intensity > voxel.point.intensity)
            {
              voxel.point = point;
              // Notify that the voxel point has been updated
              updated = true;
            }
            break;
          }
          // If center point mode enabled,
          // keep the closest point to the voxel center
          case SamplingMode::CENTER_POINT:
          {
            // Check if the new point is closer to the voxel center than the current voxel point
            Eigen::Vector3f voxelCenter = voxelGridCenterIn - this->VoxelWidth / 2.f + this->LeafSize * voxelCoordIn.cast<float>();
            if ((point.getVector3fMap()- voxelCenter).norm() < (voxel.point.getVector3fMap() - voxelCenter).norm())
            {
              voxel.point = point;
              // Notify that the voxel point has been updated
              updated = true;
            }
            break;
          }
          // If centroid mode enabled,
          // compute the mean point of the added points laying in this voxel.
          // Then, the voxel point will be the average of all the mean points computed in this voxel.
          case SamplingMode::CENTROID:
          {
            // Shortcut to voxel of added keypoints cloud
            Voxel& vMean = meanPts[idxOut][idxIn];
            // Compute mean point of current added points in the voxel
            vMean.point.getVector3fMap() = (vMean.point.getVector3fMap() * vMean.count + point.getVector3fMap()) / (vMean.count + 1);
            ++vMean.count;

            // Notify that the voxel point has been updated
            updated = true;
            break;
          }
        }
      }

      // For centroid mode, compute average point
      if (this->Sampling == SamplingMode::CENTROID)
      {
        for (auto& vOut : meanPts)
        {
          // Extract coordinates of voxel
          unsigned int idxOut = vOut.first;
          for (auto& vIn : vOut.second)
          {
            unsigned int idxIn = vIn.first;
            // Get voxel using its coordinates
            auto& voxel = this->Voxels[idxOut][idxIn];
            // Update the voxel point computing the centroid of all mean points laying in it
            voxel.point.getVector3fMap() = (voxel.point.getVector3fMap() * voxel.count + vIn.second.point.getVector3fMap()) / (voxel.count + 1);
          }
        }
      }

      // Shortcut to voxel
      auto& voxel = this->Voxels[idxOut][idxIn];
      voxel.point.time = Utils::PclStampToSec(pointcloud->header.stamp) + point.time;

      // If fixed is set, force the voxel to be fixed
      if (fixed)
          voxel.point.label = static_cast<int>(LidarSlam::LidarPointLabel::FIXED);

      if (!seen.count(idxOut) || !seen[idxOut].count(idxIn))
      {
        ++voxel.count;
        seen[idxOut][idxIn] = true;
      }
    }
  }

  // Clear the deprecated KD-tree if the map has been updated
  if (updated)
    this->KdTree.Reset();
}

//------------------------------------------------------------------------------
void RollingGrid::LabelNewPoints(PointCloud::Ptr& pointcloud, bool expand) const
{
  std::vector<Eigen::Array3i> neighbors = expand ?
                                            std::vector<Eigen::Array3i>({{ 1,  1, 1}, { 1,  1, 0}, { 1,  1, -1},
                                                                         { 1,  0, 1}, { 1,  0, 0}, { 1,  0, -1},
                                                                         { 1, -1, 1}, { 1, -1, 0}, { 1, -1, -1},
                                                                         { 0,  1, 1}, { 0,  1, 0}, { 0,  1, -1},
                                                                         { 0,  0, 1}, { 0,  0, 0}, { 0,  0, -1},
                                                                         { 0, -1, 1}, { 0, -1, 0}, { 0, -1, -1},
                                                                         {-1,  1, 1}, {-1,  1, 0}, {-1,  1, -1},
                                                                         {-1,  0, 1}, {-1,  0, 0}, {-1,  0, -1},
                                                                         {-1, -1, 1}, {-1, -1, 0}, {-1, -1, -1}}) :
                                            std::vector<Eigen::Array3i>({{ 0,  0, 0}});

  // Compute the 3D position of the center of the first voxel
  Eigen::Array3f voxelGridOrigin = this->VoxelGridPosition - int(this->GridSize / 2) * this->VoxelWidth;

  // Sample points which are different from fixed points in the rolling grid
  for (Point& point : *pointcloud)
  {
    // Init label
    point.label = static_cast<std::uint16_t>(LidarSlam::LidarPointLabel::DEFAULT);
    // Find the outer voxel containing this point
    Eigen::Array3i voxelCoordOut = Utils::PositionToVoxel<Eigen::Array3f>(point.getArray3fMap(), voxelGridOrigin, this->VoxelWidth);

    // Check whether or not the point is within bounds
    if (((0 <= voxelCoordOut) && (voxelCoordOut < this->GridSize)).all())
    {
      // Compute the position of the center of the inner voxel grid (=sampling voxel grid)
      // which is the center of the outer voxel (from the rolling voxel grid)
      Eigen::Array3f voxelGridCenterIn = voxelCoordOut.cast<float>() * this->VoxelWidth + voxelGridOrigin;
      // Find the inner voxel containing this point (from the sampling vg)
      Eigen::Array3i voxelCoordIn = Utils::PositionToVoxel<Eigen::Array3f>(point.getArray3fMap(), voxelGridCenterIn, this->LeafSize);
      unsigned int idxOut = this->To1d(voxelCoordOut, this->GridSize);
      if (!this->Voxels.count(idxOut))
        continue;

      for (auto& neigh : neighbors)
      {
        Eigen::Array3i studiedVox = voxelCoordIn + neigh;
        unsigned int idxIn = this->To1d(studiedVox, this->GridInSize);
        // If the inner voxel is occupied, label it as the voxel.
        if (this->Voxels.at(idxOut).count(idxIn))
        {
          point.label = this->Voxels.at(idxOut).at(idxIn).point.label;
          break;
        }
      }
    }
  }
}

//==============================================================================
//   Sub map use
//==============================================================================

//------------------------------------------------------------------------------
void RollingGrid::ClearPoints(double currentTime, bool clearOldPoints)
{
  // Loop on the outer voxels (rolling vg)
  auto itVoxelsOut = this->Voxels.begin();
  while(itVoxelsOut != this->Voxels.end())
  {
    this->Erase(itVoxelsOut->first,
                [&](const Point& pt)
                   {return clearOldPoints ? currentTime - pt.time > this->DecayingThreshold
                                          : pt.time > currentTime;});
    // Remove empty outer voxels
    if (itVoxelsOut->second.empty())
      itVoxelsOut = this->Voxels.erase(itVoxelsOut);
    else
      ++itVoxelsOut;
  }
}

//------------------------------------------------------------------------------
void RollingGrid::EmptyAroundPoint(double distThreshold, const Eigen::Array3f& position)
{
  // Extract the voxels around positions so distTreshold lays inside
  Eigen::Array3f voxelGridOrigin = this->VoxelGridPosition - int(this->GridSize / 2) * this->VoxelWidth;
  Eigen::Array3i voxelCoordOut = Utils::PositionToVoxel<Eigen::Array3f>(position, voxelGridOrigin, this->VoxelWidth);
  // Shortcuts
  int xPos = voxelCoordOut.x();
  int yPos = voxelCoordOut.y();
  int zPos = voxelCoordOut.z();
  // Compute radius of voxels to examinate
  int radius = 1 + distThreshold / this->VoxelWidth;
  Eigen::Vector3f positionFloat = position.cast<float>();
  for (int xIdx = std::max(xPos - radius, 0); xIdx <= std::min(xPos + radius, this->GridSize - 1); ++xIdx)
  {
    for (int yIdx = std::max(yPos - radius, 0); yIdx <= std::min(yPos + radius, this->GridSize - 1); ++yIdx)
    {
      for (int zIdx = std::max(zPos - radius, 0); zIdx <= std::min(zPos + radius, this->GridSize - 1); ++zIdx)
      {
        int voxIdx = this->To1d({xIdx, yIdx, zIdx}, this->GridSize);
        if (this->Voxels.count(voxIdx))
        {
          this->Erase(voxIdx,
                      [&](const Point& pt)
                         {return (pt.getVector3fMap() - positionFloat).norm() < distThreshold;});
          if (this->Voxels[voxIdx].empty())
            this->Voxels.erase(voxIdx);
        }
      }
    }
  }
}

//------------------------------------------------------------------------------
void RollingGrid::Erase(int outVoxIdx, std::function<bool(const Point&)> heuristic)
{
  auto itVoxelsIn = this->Voxels[outVoxIdx].begin();
  while(itVoxelsIn != this->Voxels[outVoxIdx].end())
  {
    // Shortcut to voxel
    Voxel& voxel = itVoxelsIn->second;
    // If voxel is removable and in sphere, remove it
    if (voxel.point.label != static_cast<int>(LidarSlam::LidarPointLabel::FIXED) &&
        heuristic(itVoxelsIn->second.point))
    {
      itVoxelsIn = this->Voxels[outVoxIdx].erase(itVoxelsIn);
      --this->NbPoints;
    }
    else
      ++itVoxelsIn;
  }
}

//------------------------------------------------------------------------------
void RollingGrid::BuildSubMap()
{
  // Get all points from all voxels
  this->SubMap = this->Get();
}

//------------------------------------------------------------------------------
void RollingGrid::BuildSubMap(const Eigen::Array3f& minPoint, const Eigen::Array3f& maxPoint, int minNbPoints)
{
  // Compute the position of the origin cell (0, 0, 0) of the grid
  Eigen::Array3f voxelGridOrigin = this->VoxelGridPosition - int(this->GridSize / 2) * this->VoxelWidth;

  // Get sub-VoxelGrid bounds
  Eigen::Array3i intersectionMin = Utils::PositionToVoxel<Eigen::Array3f>(minPoint, voxelGridOrigin, this->VoxelWidth).max(0);
  Eigen::Array3i intersectionMax = Utils::PositionToVoxel<Eigen::Array3f>(maxPoint, voxelGridOrigin, this->VoxelWidth).min(this->GridSize - 1);

  // Intersection points
  this->SubMap.reset(new PointCloud);
  // reserve too much space to not have to reallocate memory
  this->SubMap->reserve(this->NbPoints);

  // If we don't want to filter moving objects
  if (minNbPoints < 0 || this->MinFramesPerVoxel <= 1)
  {
    // Loop on the outer voxels
    // to extract all intersecting voxels
    for (const auto& kvOut : this->Voxels)
    {
      // Check if the voxel lies within bounds
      Eigen::Array3i idx3d = this->To3d(kvOut.first, this->GridSize);
      if (((intersectionMin <= idx3d) && (idx3d <= intersectionMax)).all())
      {
        for (const auto& kvIn : kvOut.second)
          this->SubMap->push_back(kvIn.second.point);
      }
    }
  }
  // If we want to reject moving objects
  else
  {
    // Loop on the outer voxels (rolling vg)
    // to extract intersecting voxels which do not contain moving objects
    for (const auto& kvOut : this->Voxels)
    {
     // Check if the voxel lies within bounds
     Eigen::Array3i idx3d = this->To3d(kvOut.first, this->GridSize);
     if (((intersectionMin <= idx3d) && (idx3d <= intersectionMax)).all())
     {
       // Loop on the inner voxels (sampling vg)
       for (const auto& kvIn : kvOut.second)
       {
         // Check if enough points lie in the voxel
         // or if the points are fixed before adding it
         if (kvIn.second.count >= this->MinFramesPerVoxel ||
             kvIn.second.point.label == static_cast<int>(LidarSlam::LidarPointLabel::FIXED))
          this->SubMap->push_back(kvIn.second.point);
       }
     }
    }

    // If the constraint was too strong
    // remove the constraint
    if (int(this->SubMap->size()) < minNbPoints)
    {
      PRINT_WARNING("Moving objects constraint was too strong, removing constraint");
      // Loop on the outer voxels
      // to extract intersecting voxels which contains a potential moving objects
      for (const auto& kvOut : this->Voxels)
      {
       // Check if the voxel lies within bounds
       Eigen::Array3i idx3d = this->To3d(kvOut.first, this->GridSize);
       if (((intersectionMin <= idx3d) && (idx3d <= intersectionMax)).all())
       {
         for (const auto& kvIn : kvOut.second)
         {
           // Invert constraint to add the other points
           if (kvIn.second.count < this->MinFramesPerVoxel && kvIn.second.point.label != 1)
              this->SubMap->push_back(kvIn.second.point);
         }
       }
      }
    }
  }

  if (this->SubMap->empty())
    PRINT_WARNING("No intersecting voxels found with current scan");
}

//------------------------------------------------------------------------------
void RollingGrid::BuildSubMap(const PointCloud& pc, int minNbPoints)
{
  // Compute the position of the origin cell (0, 0, 0) of the grid
  Eigen::Array3f voxelGridOrigin = this->VoxelGridPosition - int(this->GridSize / 2) * this->VoxelWidth;

  // Reset the submap
  this->SubMap.reset(new PointCloud);
  // reserve too much space to not have to reallocate memory
  this->SubMap->reserve(this->NbPoints);

  // Storage for voxels in which there is at least one point of pc
  std::unordered_map<int, Eigen::Array3i> visited;
  visited.reserve(pc.size());

  // Extract points' voxels in visited container
  for (const auto& pt : pc)
  {
    // Get outer voxel of the current point
    Eigen::Array3i voxelCoordOut = Utils::PositionToVoxel<Eigen::Array3f>(pt.getArray3fMap(), voxelGridOrigin, this->VoxelWidth);
    int idxVoxOut = this->To1d(voxelCoordOut, this->GridSize);

    // Tag the voxel as visited
    if (this->Voxels.count(idxVoxOut))
      visited.insert({idxVoxOut, voxelCoordOut});
  }

  // Storage for voxels to add to the submap
  std::unordered_set<int> toAdd;
  toAdd.reserve(pc.size());

  const std::vector<Eigen::Array3i> neighbors = {{ 1,  1, 1}, { 1,  1, 0}, { 1,  1, -1},
                                                 { 1,  0, 1}, { 1,  0, 0}, { 1,  0, -1},
                                                 { 1, -1, 1}, { 1, -1, 0}, { 1, -1, -1},
                                                 { 0,  1, 1}, { 0,  1, 0}, { 0,  1, -1},
                                                 { 0,  0, 1},              { 0,  0, -1},
                                                 { 0, -1, 1}, { 0, -1, 0}, { 0, -1, -1},
                                                 {-1,  1, 1}, {-1,  1, 0}, {-1,  1, -1},
                                                 {-1,  0, 1}, {-1,  0, 0}, {-1,  0, -1},
                                                 {-1, -1, 1}, {-1, -1, 0}, {-1, -1, -1}};

  // Extract the visited voxels and their neighbors in the toAdd container
  for (const auto& vxIdxToCoords : visited)
  {
    // Add the visited voxel
    toAdd.insert(vxIdxToCoords.first);

    // Add the voxel neighbors
    // if they have not been added before
    for (const auto& neigh : neighbors)
    {
      int idxVoxOutN = this->To1d(vxIdxToCoords.second + neigh, this->GridSize);

      // Tag the voxel as to add to the submap
      if (this->Voxels.count(idxVoxOutN))
        toAdd.insert(idxVoxOutN);
    }
  }

  // If we don't want to filter moving objects
  if (minNbPoints < 0 || this->MinFramesPerVoxel <= 1)
  {
    // Add the selected voxels' points to the submap
    for (const auto& vxIdx : toAdd)
    {
      // Add all points in this outer voxel
      for (const auto& kvIn : this->Voxels[vxIdx])
        this->SubMap->push_back(kvIn.second.point);
    }
  }
  // If we want to filter moving objects
  else
  {
    // Add the selected voxels' points to the submap
    for (const auto& vxIdx : toAdd)
    {
      // Add inner voxels that have been seen enough
      for (const auto& kvIn : this->Voxels[vxIdx])
      {
        if (kvIn.second.count > this->MinFramesPerVoxel ||
            kvIn.second.point.label == static_cast<int>(LidarSlam::LidarPointLabel::FIXED))
          this->SubMap->push_back(kvIn.second.point);
      }
    }

    // If the constraint was too strong
    // remove the constraint and add the missing points
    if (int(this->SubMap->size()) < minNbPoints)
    {
      for (const auto& vxIdx : toAdd)
      {
        // Re-add missing inner voxels
        for (const auto& kvIn : this->Voxels[vxIdx])
        {
          if (kvIn.second.count < this->MinFramesPerVoxel &&
              kvIn.second.point.label != 1)
            this->SubMap->push_back(kvIn.second.point);
        }
      }
    }
  }

  if (this->SubMap->empty())
    PRINT_WARNING("No intersecting voxels found with current scan");
}

//------------------------------------------------------------------------------
void RollingGrid::BuildKdTree()
{
  if (!this->SubMap)
  {
    PRINT_WARNING("RollingGrid: no submap, Kdtree cannot be built");
    return;
  }

  this->KdTree.Reset(this->SubMap);
}

//------------------------------------------------------------------------------
void RollingGrid::BuildKdTreeOnAllPts()
{
  this->BuildSubMap();
  this->BuildKdTree();
}

//==============================================================================
//   Helpers
//==============================================================================

//------------------------------------------------------------------------------
int RollingGrid::To1d(const Eigen::Array3i& voxelId3d, int gridSize) const
{
  return voxelId3d.z() * gridSize * gridSize + voxelId3d.y() * gridSize + voxelId3d.x();
}

//------------------------------------------------------------------------------
Eigen::Array3i RollingGrid::To3d(int voxelId1d, int gridSize) const
{
  int z = voxelId1d / (gridSize * gridSize);
  voxelId1d -= z * gridSize * gridSize;
  int y = voxelId1d / gridSize;
  voxelId1d -= y * gridSize;
  int x = voxelId1d;
  return {x, y, z};
}

} // end of LidarSlam namespace