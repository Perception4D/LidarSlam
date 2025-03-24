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

#pragma once

#include "LidarSlam/Enums.h"
#include "LidarSlam/LidarPoint.h"
#include "LidarSlam/KDTreePCLAdaptor.h"
#include <unordered_map>

#define SetMacro(name,type) void Set##name (type _arg) { name = _arg; }
#define GetMacro(name,type) type Get##name () const { return name; }

namespace LidarSlam
{

namespace Utils
{

//! Compute the voxel coordinates in which a point lies
//! Origin must be the center of the first voxel (0, 0, 0)
template<typename T>
inline Eigen::Array3i PositionToVoxel(const T& position, const T& origin, double resolution)
{
  return ( (position - origin) / resolution ).array().round().template cast<int>();
}

} // end of Utils namespace

// Define an alias for RollingGrid pointer
class RollingGrid;
using RollingGridPtr = std::shared_ptr<RollingGrid>;

/*!
 * @brief Rolling voxel grid to store and access pointclouds of specific areas.
 *
 * The map reconstructed from the SLAM algorithm is stored in a voxel grid
 * which splits the space in different regions. From this voxel grid, it is
 * possible to only load the parts of the map which are pertinent when we run
 * the localization optimization step. Morevover, when a region of the space is
 * too far from the current sensor position, it is possible to remove the points
 * stored in this region and to move the voxel grid in a closest region of the
 * sensor position. This is used to decrease the memory used by the algorithm.
 */
class RollingGrid
{
public:

  // Useful types
  using Point = LidarPoint;
  using PointCloud = pcl::PointCloud<Point>;
  using KDTree = KDTreePCLAdaptor<Point>;

  // Voxel structure to store the remaining point
  // after downsampling and to count the number
  // of updates that have been performed on the voxel
  struct Voxel
  {
    Point point;
    unsigned int count = 0;
  };

  using SamplingVG = std::unordered_map<int, Voxel>;
  using RollingVG  = std::unordered_map<int, SamplingVG>;

  //============================================================================
  //   Initialization and parameters setters
  //============================================================================

  //! Init a Rolling grid centered near a given position
  RollingGrid(const Eigen::Vector3f& position = Eigen::Vector3f::Zero());

  //! Reset map (clear voxels, reset position, ...)
  void Reset(const Eigen::Vector3f& position = Eigen::Vector3f::Zero());

  //! Remove all points from all voxels and clear the submap KD-tree
  void Clear();

  //! Set grid size (number of voxels in each direction)
  //! NOTE: this may remove some points from the grid if size is decreased
  //! The sub-map KD-tree is cleared during the process.
  void SetGridSize(int size);
  GetMacro(GridSize, int)

  //! Set voxel resolution (resolution of each voxel, in meters)
  //! NOTE: this may remove some points from the grid if resolution is decreased
  //! The sub-map KD-tree is cleared during the process.
  void SetVoxelResolution(double resolution);
  GetMacro(VoxelResolution, double)

  void SetLeafSize(double ls);
  GetMacro(LeafSize, double)

  SetMacro(MinFramesPerVoxel, unsigned int)
  GetMacro(MinFramesPerVoxel, unsigned int)

  SetMacro(Sampling, SamplingMode)
  GetMacro(Sampling, SamplingMode)

  SetMacro(DecayingThreshold, double)
  GetMacro(DecayingThreshold, double)

  // Check if keypoints time decaying is enabled
  bool IsTimeThreshold() const {return DecayingThreshold > 0;}
  //============================================================================
  //   Main rolling grid use
  //============================================================================

  //! Get all points in the grid
  //! clean allows to remove the moving objects
  PointCloud::Ptr Get(bool clean = false) const;

  //! Label non-fixed points in a pointcloud with respect to a fixed current map
  //! Non-fixed points are labeled as 0, others as 1.
  //! The sampling mode is to take the first point
  void LabelNewPoints(PointCloud::Ptr& pointcloud, bool expand = false) const;

  //! Get the total number of points in rolling grid
  unsigned int Size() const {return this->NbPoints; }

  //! Roll the grid so that input bounding box can fit it in rolled map
  void Roll(const Eigen::Array3f& minPoint, const Eigen::Array3f& maxPoint);

  //! Add some points to the grid.
  //! If fixed is true, the points added will not be modified afterwards.
  //! If roll is true, the map is rolled first so that all new points to add can fit in rolled map.
  //! If points are added, the sub-map KD-tree is cleared.
  void Add(const PointCloud::Ptr& pointcloud, bool fixed = false, bool roll = true);

  //============================================================================
  //   Sub map use
  //============================================================================

  //! Extract the submaps rejecting moving object using the MinFramesPerVoxel criterion
  //! Build the submap using all the points
  void BuildSubMap();

  //! Extract the submap as the map points laying in the bounding box formed by minPoint and maxPoint.
  //! minNbPoints allows to not reject moving objects if the extracted submap is not dense enough
  //! if minNbPoints is negative, all points are taken (no moving objects rejection)
  void BuildSubMap(const Eigen::Array3f& minPoint, const Eigen::Array3f& maxPoint, int minNbPoints = -1);

  //! Extract the submap as the map points laying in the same voxels as the one containing pc points.
  //! minNbPoints allows to not reject moving objects if the extracted submap is not dense enough
  //! if minNbPoints is negative, all points are taken (no moving objects rejection)
  void BuildSubMap(const PointCloud& pc, int minNbPoints = -1);

  //! Build a KD-tree from the submap
  //! This KD-tree is implicitly used for fast NN queries
  //! /!\ the function will fail if the submap has not been created
  void BuildKdTree();

  //! Build a KD-tree from all the points in the grid
  //! This KD-tree is implicitly used for fast NN queries
  //! In this case, the submap contains all of the points
  void BuildKdTreeOnAllPts();

  //! Check if the KD-tree built on top of the submap is valid or if it needs to be updated.
  //! The KD-tree is cleared every time the map is modified.
  bool IsSubMapKdTreeValid() const {return this->KdTree.GetInputCloud() && !this->KdTree.GetInputCloud()->empty();}

  //! Get the internal pointcloud used by the KD-Tree
  const PointCloud::Ptr GetKdTreePcl() const{return this->KdTree.GetInputCloud();}

  //! Do a KnnSearch on the RollingGrid Submap
  inline size_t KnnSearch(const float queryPoint[3], int knearest, int* knnIndices, float* knnSqDistances) const
  {return this->KdTree.KnnSearch(queryPoint, knearest, knnIndices, knnSqDistances);}

  //! Do a KnnSearch on the RollingGrid Submap
  inline size_t KnnSearch(const double queryPoint[3], int knearest, std::vector<int>& knnIndices, std::vector<float>& knnSqDistances) const
  {return this->KdTree.KnnSearch(queryPoint, knearest, knnIndices, knnSqDistances);}

  //! Get the sub map lastly computed
  const PointCloud::Ptr GetSubMap() const {return this->SubMap;}

  //! Remove the oldest voxels from the map relatively to the current time using the decaying time threshold member
  //! If clearOldPoints is false, remove voxels newer than the currentTime
  void ClearPoints(double currentTime, bool clearOldPoints = true);

  //! Remove the points around position
  //! useful to remove the points around a specific point of view
  void EmptyAroundPoint(double distThreshold, const Eigen::Array3f& position = Eigen::Array3f::Zero());

  //============================================================================
  //   Attributes and helper methods
  //============================================================================

private:
  // Helpers
  // Erase the points in the outer voxel referenced
  // by the index outVoxIdx and complying with the heuristic
  void Erase(int outVoxIdx, std::function<bool(const Point&)> heuristic);

  //! [voxels] Max size of the outer voxel grid: n*n*n voxels
  int GridSize = 50;

  //! [voxels] Max size of the inner voxel grid: n*n*n voxels
  int GridInSize;

  //! [m/voxel] Resolution of an outer voxel
  //! It stores the parameter supplied by outside process
  double VoxelResolution = 10.;
  //! [m] Width of an outer voxel. Equivalent to VoxelResolution but
  //! might be a bit lower as it is the closest multiple of the leaf size
  double VoxelWidth = 10.;

  //! [m] Size of the leaf used to downsample the pointcloud with a VoxelGrid filter within each voxel
  double LeafSize = 0.2;

  //! Outer voxelGrid to roll map, build a target submap and add keypoints efficiently.
  //! Each voxel contains an inner voxel grid (=sampling vg) that has at most one point per voxel
  //! These sampling vg are used to downsample the grid when adding new keypoints
  //! and to filter moving objects if required.
  //! Each outer voxel can be accessed using a flattened 1D index.
  RollingVG Voxels;

  //! [m, m, m] Current position of the center of the outer VoxelGrid
  Eigen::Array3f VoxelGridPosition;

  //! Total number of points stored in the rolling grid
  unsigned int NbPoints;

  //! KD-Tree built on top of local sub-map for fast NN queries in sub-map
  KDTree KdTree;

  //! Local sub-map stored for further visualization
  PointCloud::Ptr SubMap;

  //! Minimum number of points in a voxel
  //! to extract it in a submap
  unsigned int MinFramesPerVoxel = 0;

  //! The grid is filtered to contain at most one point per inner voxel
  //! This mode parameter allows to choose how to select the remaining point
  //! It can be : taking the first/last acquired point, taking the max intensity point,
  //! considering the closest point to the voxel center or averaging the points.
  SamplingMode Sampling = SamplingMode::MAX_INTENSITY;

  //! Time threshold to discard removable keypoints
  //! If negative, the keypoints are never removed
  double DecayingThreshold = -1;

private:

  //! Conversion from 3D voxel index to 1D flattened index
  int To1d(const Eigen::Array3i& voxelId3d, int gridSize) const;

  //! Conversion from 1D flattened voxel index to 3D index
  Eigen::Array3i To3d(int voxelId1d, int gridSize) const;
};

} // end of LidarSlam namespace